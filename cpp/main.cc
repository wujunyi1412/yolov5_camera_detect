// Copyright (c) 2023 by Rockchip Electronics Co., Ltd. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*-------------------------------------------
                Includes
-------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <chrono>
#include <deque>
#include <ctime>
#include <iomanip>
#include <sstream>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <fstream>

#include "yolov5.h"
#include "image_utils.h"
#include "file_utils.h"
#include "image_drawing.h"
// #include "v4l2_camera.hpp"
#include "udp_sender.hpp"
#include "rga_drm.hpp"
#include "video_capture.h"
#include "sample_comm.h"

#if defined(RV1106_1103) 
    #include "dma_alloc.hpp"
#endif


static bool g_running = true;

/**
 * Ctrl + C 信号处理
 */
void signal_handler(int sig) {
    g_running = false;
}

void init_infer_buffer(image_buffer_t& buf, VIDEO_FRAME_INFO_S& cam_image)
{
    int size = cam_image.stVFrame.u32VirWidth * cam_image.stVFrame.u32VirHeight * 3 ;
    // int size = cam_image.stVFrame.u32VirWidth * cam_image.stVFrame.u32VirHeight * 3 / 2 ;

    buf.width         = cam_image.stVFrame.u32Width;
    buf.height        = cam_image.stVFrame.u32Height;
    buf.width_stride  = cam_image.stVFrame.u32VirWidth;
    buf.height_stride = cam_image.stVFrame.u32VirHeight;
    buf.format        = IMAGE_FORMAT_RGB888;
    buf.virt_addr = nullptr;
    buf.size          = size;
    buf.fd            = -1; // CPU 内存，没有 dmabuf
}


cv::Mat yuv420sp_to_bgr(image_buffer_t &buf) {
    cv::Mat yuv(buf.height + buf.height/2, buf.width, CV_8UC1, buf.virt_addr);
    cv::Mat bgr;
    cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_NV12);
    return bgr;
}

std::string gen_time_filename(const std::string& dir,
                              const std::string& prefix,
                              const std::string& ext)
{
    using namespace std::chrono;

    auto now = system_clock::now();
    auto sec = system_clock::to_time_t(now);
    auto ms  = duration_cast<milliseconds>(
                   now.time_since_epoch()) % 1000;

    std::tm tm{};
    localtime_r(&sec, &tm);

    std::ostringstream oss;
    oss << dir << "/"
        << prefix << "_"
        << std::put_time(&tm, "%Y%m%d_%H%M%S")
        << "_" << std::setw(3) << std::setfill('0') << ms.count()
        << ext;

    return oss.str();
}


void save_nv12_as_jpg(unsigned char* nv12_data, int width, int height)
{
    // NV12 → BGR
    cv::Mat yuv(height + height / 2, width, CV_8UC1, nv12_data);
    cv::Mat bgr;
    cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_NV12);

    std::string filename = gen_time_filename("images", "out", ".jpg");
    cv::imwrite(filename, bgr);
    std::cout << "Saved: " << filename << " width=" << width
            << " height=" << height << " channel=" << bgr.channels() << std::endl;
}


bool playAudio(const std::string &filePath) {
            std::string cmd = "aplay \"" + filePath + "\"";
            return system(cmd.c_str()) == 0;
}


void print_meminfo() {
    std::cout << "------ 系统内存状态（MemFree/CMA） ------" << std::endl;

    std::string cmd = "cat /proc/meminfo | grep -i 'memfree\\|cma'";
    std::array<char, 128> buffer;
    FILE* pipe = popen(cmd.c_str(), "r");

    if (!pipe) {
        std::cerr << "popen 失败！" << std::endl;
        return;
    }

    while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
        std::cout << buffer.data();
    }

    pclose(pipe);
}


static int parse_int(const char *s, int *out)
{
    char *end;
    long v;

    if (!s) return -1;

    v = strtol(s, &end, 10);

    if (*end != '\0') return -1;
    if (v <= 0 || v > INT_MAX) return -1;

    *out = (int)v;
    return 0;
}


/*-------------------------------------------
                  Main Function
-------------------------------------------*/
int main(int argc, char **argv)
{

    print_meminfo();
    system("RkLunch-stop.sh");
    sleep(5);
    std::cout << "停止 RkLunch 服务，释放内存..." << std::endl;
    print_meminfo();
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    auto lastSend = std::chrono::steady_clock::now();

    if (argc < 2) {
        printf("Usage: %s <model_path> [width] [height]\n", argv[0]);
        return -1;
    }

    const char *model_path = argv[1];

    int dma_buff_width = 640;
    int dma_buff_height = 640;

    if (argc >= 3) {
        if (parse_int(argv[2], &dma_buff_width) < 0) {
            printf("invalid width\n");
            return -1;
        }
    }

    if (argc >= 4) {
        if (parse_int(argv[3], &dma_buff_height) < 0) {
            printf("invalid height\n");
            return -1;
        }
    }
    
    int ret;

    rknn_app_context_t rknn_app_ctx;
    memset(&rknn_app_ctx, 0, sizeof(rknn_app_context_t));

    init_post_process();

    ret = init_yolov5_model(model_path, &rknn_app_ctx);
    if (ret != 0)
    {
        printf("init_yolov5_model fail! ret=%d model_path=%s\n", ret, model_path);
        return -1;
    }
    print_meminfo();


    RK_S32 s32Ret = 0;
    
    VIDEO_FRAME_INFO_S vitr_frame;

    // rkmpi init
	if (RK_MPI_SYS_Init() != RK_SUCCESS) {
		RK_LOGE("rk mpi sys init fail!");
		return -1;
	}

    // rkaiq init
	RK_BOOL multi_sensor = RK_FALSE;	
	const char *iq_dir = "/etc/iqfiles";
	rk_aiq_working_mode_t hdr_mode = RK_AIQ_WORKING_MODE_NORMAL;
	//hdr_mode = RK_AIQ_WORKING_MODE_ISP_HDR2;
	SAMPLE_COMM_ISP_Init(0, hdr_mode, multi_sensor, iq_dir);
	SAMPLE_COMM_ISP_Run(0);

    std::cout << "Camera and ISP initialized." << std::endl;
    print_meminfo();

    VideoCapture cap;

    if (!cap.init(dma_buff_width, dma_buff_height)) {
        std::cerr << "Failed to init video capture" << std::endl;
        return -1;
    }

    std::cout << "Video capture initialized." << std::endl;
    print_meminfo();

//UDP 初始化
    UdpSender sender("192.168.4.1", 4210);
//UDP 初始化

//Framebuffer 初始化
    FramebufferDisplay fb("/dev/fb0", 480, 480);
//Framebuffer 初始化

    object_detect_result_list od_results;

    image_buffer_t src_image;
 
    memset(&src_image, 0, sizeof(image_buffer_t));

    if (!cap.read(vitr_frame, -1)) {
            std::cerr << "Failed to init frame from video capture" << std::endl;
            return -1;
        }

    init_infer_buffer(src_image, vitr_frame);

    cap.release(vitr_frame);


    MB_POOL_CONFIG_S PoolCfg;
	memset(&PoolCfg, 0, sizeof(MB_POOL_CONFIG_S));
	PoolCfg.u64MBSize = src_image.width_stride * src_image.height_stride * 3 ;
	PoolCfg.u32MBCnt = 1;
	PoolCfg.enAllocType = MB_ALLOC_TYPE_DMA;
	//PoolCfg.bPreAlloc = RK_FALSE;
	MB_POOL src_Pool = RK_MPI_MB_CreatePool(&PoolCfg);
	printf("Create Pool success !\n");	

	// Get MB from Pool 
	MB_BLK src_Blk = RK_MPI_MB_GetMB(src_Pool, src_image.width_stride * src_image.height_stride * 3, RK_TRUE);

    unsigned char *dataBGR = (unsigned char *)RK_MPI_MB_Handle2VirAddr(src_Blk);

    cv::Mat frameBGR(cv::Size(src_image.width,src_image.height),CV_8UC3, dataBGR, src_image.width_stride * 3);

    int frameBGR_size = src_image.width_stride * src_image.height_stride * 3;

    ret = dma_buf_alloc(RV1106_CMA_HEAP_PATH, frameBGR_size, &rknn_app_ctx.img_dma_buf.dma_buf_fd, 
                       (void **) & (rknn_app_ctx.img_dma_buf.dma_buf_virt_addr));


    if (ret != 0) {
        printf("dma_buf_alloc fail! ret=%d size=%d\n", ret, src_image.size);
        return -1;  
    }

    while (g_running){
    
        auto t1 = std::chrono::high_resolution_clock::now();

        if (!cap.read(vitr_frame, -1)) {
            std::cerr << "Failed to read frame from video capture" << std::endl;
            break;
        }
   
        void* vitr_frame_virt = RK_MPI_MB_Handle2VirAddr(vitr_frame.stVFrame.pMbBlk);

        //nv12
        cv::Mat yuv420sp(vitr_frame.stVFrame.u32Height + vitr_frame.stVFrame.u32Height / 2, vitr_frame.stVFrame.u32Width, CV_8UC1, vitr_frame_virt, vitr_frame.stVFrame.u32VirWidth);
   
        //nv12 to bgr
        cv::cvtColor(yuv420sp, frameBGR, cv::COLOR_YUV2BGR_NV12);

        auto t2 = std::chrono::high_resolution_clock::now();  // 抓帧结束，拷贝前

        memcpy(rknn_app_ctx.img_dma_buf.dma_buf_virt_addr, dataBGR, frameBGR_size);

        dma_sync_cpu_to_device(rknn_app_ctx.img_dma_buf.dma_buf_fd);
     
        src_image.virt_addr = (unsigned char *)rknn_app_ctx.img_dma_buf.dma_buf_virt_addr;
    
        rknn_app_ctx.img_dma_buf.size = frameBGR_size;
    
        src_image.fd = rknn_app_ctx.img_dma_buf.dma_buf_fd;
    
        cap.release(vitr_frame);
     
        auto t3 = std::chrono::high_resolution_clock::now();  // 拷贝结束，推理前

        ret = inference_yolov5_model(&rknn_app_ctx, &src_image, &od_results);

        auto t4 = std::chrono::high_resolution_clock::now();  // 推理结束

        if (ret != 0)
        {
            printf("inference_yolov5_model fail! ret=%d\n", ret);
            // goto out;
            deinit_post_process();

            ret = release_yolov5_model(&rknn_app_ctx);
            if (ret != 0)
            {
                printf("release_yolov5_model fail! ret=%d\n", ret);

            } 
        }

        char text[256];
        for (int i = 0; i < od_results.count; i++)
        {
            object_detect_result *det_result = &(od_results.results[i]);
            printf("%s @ (%d %d %d %d) %.3f\n", coco_cls_to_name(det_result->cls_id),
                det_result->box.left, det_result->box.top,
                det_result->box.right, det_result->box.bottom,
                det_result->prop);
            int x1 = det_result->box.left;
            int y1 = det_result->box.top;
            int x2 = det_result->box.right;
            int y2 = det_result->box.bottom;
            draw_rectangle(&src_image, x1, y1, x2 - x1, y2 - y1, COLOR_BLUE, 3);

            sprintf(text, "%s %.1f%%", coco_cls_to_name(det_result->cls_id), det_result->prop * 100);
            draw_text(&src_image, text, x1, y1 - 20, COLOR_RED, 10);
           
        }

        auto t5 = std::chrono::high_resolution_clock::now();  // 推理结束
        fb.display(&src_image);
        auto t6 = std::chrono::high_resolution_clock::now();  // 推理结束

        auto grab_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
        auto copy_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();
        auto infer_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count();
        auto display_fd = std::chrono::duration_cast<std::chrono::milliseconds>(t6 - t5).count();
        auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t6 - t1).count();

        printf("抓帧: %lld ms, 拷贝: %lld ms, 推理: %lld ms, 显示: %lld ms, 总耗时: %lld ms\n",
            grab_ms, copy_ms, infer_ms, display_fd, total_ms);
        
        
//*************************************************************************************************************** */
        // if (od_results.count > 0) {
        //     std::cout << "检测到 " << od_results.count << " 个目标。" << std::endl;
        //     auto now = std::chrono::steady_clock::now();
        //     if (std::chrono::duration_cast<std::chrono::seconds>(now - lastSend).count() >= 30) {
        //         sender.send("push:90");
        //         lastSend = std::chrono::steady_clock::now();
        //     }
            
        //     if (!playAudio("alert.wav")) {
        //         std::cout << "播放提示音。" << std::endl;
        //     } else {
        //         std::cout << "提示音播放失败！" << std::endl;  
        //     } 
        // } 
        // else {
        //     std::cout << "未检测到目标。" << std::endl;
        // }
//*************************************************************************************************************** */
        // save_nv12_as_jpg(src_image.virt_addr, src_image.width, src_image.height);
    }

    deinit_post_process();

    ret = release_yolov5_model(&rknn_app_ctx);
    if (ret != 0)
    {
        printf("release_yolov5_model fail! ret=%d\n", ret);
    }

    if (src_image.virt_addr != NULL)
    {
    dma_buf_free(rknn_app_ctx.img_dma_buf.size, &rknn_app_ctx.img_dma_buf.dma_buf_fd, 
                rknn_app_ctx.img_dma_buf.dma_buf_virt_addr);
    }

    return 0;
}

