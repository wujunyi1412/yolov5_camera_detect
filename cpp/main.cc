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






#include "yolov5.h"
#include "image_utils.h"
#include "file_utils.h"
#include "image_drawing.h"
#include "v4l2_camera.hpp"
#include "AudioPlayer.hpp"

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

void init_infer_buffer(image_buffer_t& buf,
                       std::vector<uint8_t>& mem,
                       int width,
                       int height)
{
    int size = width * height * 3 / 2; // NV12

    mem.resize(size);

    buf.width         = width;
    buf.height        = height;
    buf.width_stride  = width;
    buf.height_stride = height;
    buf.format        = IMAGE_FORMAT_YUV420SP_NV12;
    buf.virt_addr     = mem.data();
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


/*-------------------------------------------
                  Main Function
-------------------------------------------*/
int main(int argc, char **argv)
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // using clock = std::chrono::steady_clock;

    // clock::time_point last_time = clock::now();
    // std::deque<double> fps_window;
    // const size_t FPS_WIN = 30; 

    if (argc != 2)
    {
        printf("%s <model_path> \n", argv[0]);
        return -1;
    }

//摄像头初始化
    V4L2Camera cam("/dev/video11", 1920, 1080);
    if (!cam.open()) {
        std::cerr << "打开相机失败！\n";
        return -1;
    }

    AudioPlayer player;

    const char *model_path = argv[1];

    int ret;
    rknn_app_context_t rknn_app_ctx;
    memset(&rknn_app_ctx, 0, sizeof(rknn_app_context_t));

    init_post_process();

    ret = init_yolov5_model(model_path, &rknn_app_ctx);
    if (ret != 0)
    {
        printf("init_yolov5_model fail! ret=%d model_path=%s\n", ret, model_path);
        goto out;
    }

   
    image_buffer_t cam_image;
    image_buffer_t src_image;
    static std::vector<uint8_t> infer_mem;
    // 初始化一次
    init_infer_buffer(src_image, infer_mem, 1920, 1080);

    object_detect_result_list od_results;

    while (g_running){
        usleep(1000000);
        // auto t1 = std::chrono::high_resolution_clock::now();
        if (!cam.grab_buffer(cam_image)) {
            std::cerr << "抓帧失败！\n";
            continue;
        }
        // auto t2 = std::chrono::high_resolution_clock::now();  // 抓帧结束，拷贝前

         // 打印摄像头返回缓冲大小
        std::cout << "cam_image.size = " << cam_image.size << " bytes" << std::endl;
        std::cout << "src_image.size = " << src_image.size << " bytes" << std::endl;

        size_t copy_size = std::min(src_image.size, cam_image.size);
        std::cout << "copy_size = " << copy_size << " bytes" << std::endl;

        memcpy(src_image.virt_addr, cam_image.virt_addr, copy_size);

#if defined(RV1106_1103) 
        //RV1106 rga requires that input and output bufs are memory allocated by dma
        ret = dma_buf_alloc(RV1106_CMA_HEAP_PATH, src_image.size, &rknn_app_ctx.img_dma_buf.dma_buf_fd, 
                        (void **) & (rknn_app_ctx.img_dma_buf.dma_buf_virt_addr));
        memcpy(rknn_app_ctx.img_dma_buf.dma_buf_virt_addr, src_image.virt_addr, src_image.size);
        dma_sync_cpu_to_device(rknn_app_ctx.img_dma_buf.dma_buf_fd);
        // free(src_image.virt_addr);
        src_image.virt_addr = (unsigned char *)rknn_app_ctx.img_dma_buf.dma_buf_virt_addr;
        src_image.fd = rknn_app_ctx.img_dma_buf.dma_buf_fd;
        rknn_app_ctx.img_dma_buf.size = src_image.size;
#endif

        // auto t3 = std::chrono::high_resolution_clock::now();  // 拷贝结束，推理前

        ret = inference_yolov5_model(&rknn_app_ctx, &src_image, &od_results);

        // auto t4 = std::chrono::high_resolution_clock::now();  // 推理结束

        // auto grab_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
        // auto copy_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();
        // auto infer_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count();
        // auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t1).count();

        // printf("抓帧: %lld ms, 拷贝: %lld ms, 推理: %lld ms, 总耗时: %lld ms\n",
        //     grab_ms, copy_ms, infer_ms, total_ms);
        
   

        if (ret != 0)
        {
            printf("inference_yolov5_model fail! ret=%d\n", ret);
            goto out;
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
            // draw_rectangle(&src_image, x1, y1, x2 - x1, y2 - y1, COLOR_BLUE, 3);

            // sprintf(text, "%s %.1f%%", coco_cls_to_name(det_result->cls_id), det_result->prop * 100);
            // draw_text(&src_image, text, x1, y1 - 20, COLOR_RED, 10);
           
        }
        if (od_results.count > 0) {
            std::cout << "检测到 " << od_results.count << " 个目标。" << std::endl;
            
            player.setFile("biu.wav");  // 替换成你的 WAV 文件路径
            if (!player.play()) {
                std::cerr << "Audio playback failed!" << std::endl;
            }
        } 
        else {
            std::cout << "未检测到目标。" << std::endl;
        }
       
        // save_nv12_as_jpg(src_image.virt_addr, src_image.width, src_image.height);
  

    }

out:
    deinit_post_process();

    ret = release_yolov5_model(&rknn_app_ctx);
    if (ret != 0)
    {
        printf("release_yolov5_model fail! ret=%d\n", ret);
    }

    if (src_image.virt_addr != NULL)
    {
#if defined(RV1106_1103) 
        dma_buf_free(rknn_app_ctx.img_dma_buf.size, &rknn_app_ctx.img_dma_buf.dma_buf_fd, 
                rknn_app_ctx.img_dma_buf.dma_buf_virt_addr);
#else
        free(src_image.virt_addr);
#endif
    }

    return 0;
}

