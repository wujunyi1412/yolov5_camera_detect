#pragma once

#include <stdio.h>
#include <string.h>
#include "common.h"
#include <stdlib.h>


#include "im2d.hpp"
#include "RgaUtils.h"


#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <cstring>
#include <algorithm>
#include <iostream>

#include <sys/ioctl.h>
#include <linux/fb.h>
// #include <rga/RgaApi.h>

#include <sys/mman.h>

#include "common.h"






class FramebufferDisplay {
public:
    FramebufferDisplay(const char* fb_path, int screen_w, int screen_h)
        : fb_fd(-1), fb_mem(nullptr), screen_w(screen_w), screen_h(screen_h), fb_stride(0)
    {
        fb_fd = open(fb_path, O_RDWR);
        if (fb_fd < 0) { perror("open framebuffer"); return; }

        struct fb_fix_screeninfo finfo;
        if (ioctl(fb_fd, FBIOGET_FSCREENINFO, &finfo) == -1) {
            perror("FBIOGET_FSCREENINFO");
            close(fb_fd);
            fb_fd = -1;
            return;
        }
        fb_stride = finfo.line_length;

        size_t fb_size = fb_stride * screen_h;
        fb_mem = mmap(nullptr, fb_size, PROT_WRITE, MAP_SHARED, fb_fd, 0);
        if (fb_mem == MAP_FAILED) {
            perror("mmap framebuffer");
            fb_mem = nullptr;
            close(fb_fd);
            fb_fd = -1;
            return;
        }

        // ---------- 初始化 framebuffer 为黑色 ----------
        memset(fb_mem, 0, fb_size);
    }

    ~FramebufferDisplay() {
        if (fb_mem) munmap(fb_mem, fb_stride * screen_h);
        if (fb_fd >= 0) close(fb_fd);
    }

    void display(const image_buffer_t* img_buf) {
        if (!fb_mem || !img_buf || !img_buf->virt_addr) return;

        int src_w = img_buf->width;
        int src_h = img_buf->height;

        // ---------- 计算 letterbox 区域 ----------
        float scale_w = float(screen_w) / src_w;
        float scale_h = float(screen_h) / src_h;
        float scale = std::min(scale_w, scale_h);

        int dst_w = int(src_w * scale);
        int dst_h = int(src_h * scale);

        int offset_x = (screen_w - dst_w) / 2;
        int offset_y = (screen_h - dst_h) / 2;

        // ---------- 更新图像区域 ----------
        for (int j = 0; j < dst_h; ++j) {
            unsigned char* dst = (unsigned char*)fb_mem + (j + offset_y) * fb_stride;
            for (int i = 0; i < dst_w; ++i) {
                int src_x = std::min(int(i / scale), src_w - 1);
                int src_y = std::min(int(j / scale), src_h - 1);

                unsigned char R = 0, G = 0, B = 0;

                switch (img_buf->format) {
                    case IMAGE_FORMAT_YUV420SP_NV12: {
                        unsigned char* y_plane  = img_buf->virt_addr;
                        unsigned char* uv_plane = img_buf->virt_addr + img_buf->width_stride * img_buf->height_stride;
                        int uv_stride = img_buf->width_stride;

                        int y_idx  = src_y * img_buf->width_stride + src_x;
                        int uv_idx = (src_y / 2) * uv_stride + (src_x & ~1);

                        int Y = y_plane[y_idx];
                        int U = uv_plane[uv_idx];
                        int V = uv_plane[uv_idx + 1];

                        int C = Y - 16;
                        int D = U - 128;
                        int E = V - 128;

                        R = std::min(std::max(int(1.164f*C + 1.596f*E), 0), 255);
                        G = std::min(std::max(int(1.164f*C - 0.392f*D - 0.813f*E), 0), 255);
                        B = std::min(std::max(int(1.164f*C + 2.017f*D), 0), 255);
                        break;
                    }
                    case IMAGE_FORMAT_RGB888: {
                        unsigned char* src = (unsigned char*)img_buf->virt_addr + (src_y * src_w + src_x) * 3;
                        R = src[0];
                        G = src[1];
                        B = src[2];
                        break;
                    }
                    case IMAGE_FORMAT_RGBA8888: {
                        unsigned char* src = (unsigned char*)img_buf->virt_addr + (src_y * src_w + src_x) * 4;
                        R = src[0];
                        G = src[1];
                        B = src[2];
                        break;
                    }
                    default:
                        std::cerr << "Unsupported format" << std::endl;
                        return;
                }

                dst[i * 4 + 0] = R;
                dst[i * 4 + 1] = G;
                dst[i * 4 + 2] = B;
                dst[i * 4 + 3] = 255; // alpha 不透明
            }
        }
    }

private:
    int fb_fd;
    void* fb_mem;
    int screen_w;
    int screen_h;
    int fb_stride;
};









// // 初始化 dst_image 内存
// static inline int init_image_buffer(
//     image_buffer_t &img,
//     int width,
//     int height,
//     image_format_t format
// )
// {
//     if (width <= 0 || height <= 0) {
//         printf("init_image_buffer: invalid width/height\n");
//         return -1;
//     }

//     img.width  = width;
//     img.height = height;
//     img.format = format;
//     img.fd = -1;
//     // img.vector_virt_addr = nullptr;

//     // 根据 format 设置 stride 和 size
//     switch (format) {
//     case IMAGE_FORMAT_RGBA8888:
//         img.width_stride  = width * 4;
//         img.height_stride = height;
//         img.size = img.width_stride * img.height_stride;
//         break;

//     case IMAGE_FORMAT_RGB888:
//         img.width_stride  = width * 3;
//         img.height_stride = height;
//         img.size = img.width_stride * img.height_stride;
//         break;

//     case IMAGE_FORMAT_YUV420SP_NV12:
//     case IMAGE_FORMAT_YUV420SP_NV21:
//         img.width_stride  = width;
//         img.height_stride = height;
//         img.size = width * height * 3 / 2;  // Y + UV
//         break;

//     default:
//         printf("init_image_buffer: unsupported format\n");
//         return -1;
//     }

//     img.virt_addr = (unsigned char *)malloc(img.size);
//     if (!img.virt_addr) {
//         printf("init_image_buffer: malloc failed\n");
//         return -1;
//     }

//     // 清零
//     memset(img.virt_addr, 0, img.size);

//     return 0;
// }

// // 释放 image_buffer 内存
// static inline void free_image_buffer(image_buffer_t &img)
// {
//     if (img.virt_addr) {
//         free(img.virt_addr);
//         img.virt_addr = nullptr;
//     }
//     img.size = 0;
//     img.fd = -1;
//     // img.vector_virt_addr = nullptr;
// }





// /* ================= format mapping ================= */
// static inline int image_format_to_rk(image_format_t fmt)
// {
//     switch (fmt) {
//     case IMAGE_FORMAT_RGBA8888:
//         return IMAGE_FORMAT_RGBA8888;
//     case IMAGE_FORMAT_RGB888:
//         return IMAGE_FORMAT_RGB888;
//     case IMAGE_FORMAT_YUV420SP_NV12:
//         return IMAGE_FORMAT_YUV420SP_NV12;
//     case IMAGE_FORMAT_YUV420SP_NV21:
//         return IMAGE_FORMAT_YUV420SP_NV21;
//     default:
//         return -1;
//     }
// }

// /* ===================================================
//  * RGA letterbox resize
//  *
//  * src : input image
//  * dst : output image (memory must be allocated)
//  * dst_width / dst_height : target size (e.g. 640x640)
//  * =================================================== */
// static inline int rga_letterbox(
//     image_buffer_t &src,
//     image_buffer_t &dst,
//     int dst_width,
//     int dst_height
// )
// {
//     if (!src.virt_addr || !dst.virt_addr) {
//         printf("rga_letterbox: null virt_addr\n");
//         return -1;
//     }

//     int src_fmt = image_format_to_rk(src.format);
//     int dst_fmt = image_format_to_rk(dst.format);
//     if (src_fmt < 0 || dst_fmt < 0) {
//         printf("rga_letterbox: unsupported format\n");
//         return -1;
//     }

//     /* ---------- 1. calculate scale ---------- */
//     float scale_w = static_cast<float>(dst_width)  / src.width;
//     float scale_h = static_cast<float>(dst_height) / src.height;
//     float scale   = scale_w < scale_h ? scale_w : scale_h;

//     if (scale <= 0.f) {
//         printf("rga_letterbox: invalid scale\n");
//         return -1;
//     }

//     int resize_w = static_cast<int>(src.width * scale) & ~1;   // ensure even
//     int resize_h = static_cast<int>(src.height * scale) & ~1;  // ensure even

//     int offset_x = ((dst_width  - resize_w) / 2) & ~1;
//     int offset_y = ((dst_height - resize_h) / 2) & ~1;

//     /* update dst info */
//     dst.width  = dst_width;
//     dst.height = dst_height;

//     /* ---------- 2. import buffer ---------- */
//     rga_buffer_handle_t src_handle = importbuffer_virtualaddr(src.virt_addr, src.size);
//     rga_buffer_handle_t dst_handle = importbuffer_virtualaddr(dst.virt_addr, dst.size);

//     if (!src_handle || !dst_handle) {
//         printf("rga_letterbox: importbuffer failed\n");
//         if (src_handle) releasebuffer_handle(src_handle);
//         if (dst_handle) releasebuffer_handle(dst_handle);
//         return -1;
//     }

//     rga_buffer_t src_img = wrapbuffer_handle(
//         src_handle,
//         src.width,
//         src.height,
//         src_fmt,
//         src.width_stride,
//         src.height_stride
//     );

//     rga_buffer_t dst_img = wrapbuffer_handle(
//         dst_handle,
//         dst.width,
//         dst.height,
//         dst_fmt,
//         dst.width_stride,
//         dst.height_stride
//     );

//     /* ---------- 3. fill black background ---------- */
//     int ret = imfill(dst_img, im_rect(), 0x00000000); // RGBA black
//     if (ret != IM_STATUS_SUCCESS) {
//         printf("rga_letterbox: imfill failed: %s\n", imStrError((IM_STATUS)ret));
//         releasebuffer_handle(src_handle);
//         releasebuffer_handle(dst_handle);
//         return -1;
//     }

//     /* ---------- 4. resize using fx/fy ---------- */
//     double fx = static_cast<double>(resize_w) / src.width;
//     double fy = static_cast<double>(resize_h) / src.height;

//     ret = imresize(src_img, dst_img, fx, fy, 1, 1, nullptr);
//     if (ret != IM_STATUS_SUCCESS) {
//         printf("rga_letterbox: imresize failed: %s\n", imStrError((IM_STATUS)ret));
//         releasebuffer_handle(src_handle);
//         releasebuffer_handle(dst_handle);
//         return -1;
//     }

//     /* ---------- 5. letterbox centering ---------- */
//     if (src_fmt == IMAGE_FORMAT_RGBA8888 || src_fmt == IMAGE_FORMAT_RGB888) {
//         // 每像素 4 bytes (RGBA) 或 3 bytes (RGB)
//         int pixel_size = (src_fmt == IMAGE_FORMAT_RGBA8888) ? 4 : 3;

//         unsigned char *tmp_buf = new unsigned char[resize_h * resize_w * pixel_size];
//         memcpy(tmp_buf, dst.virt_addr, resize_h * resize_w * pixel_size);

//         for (int y = 0; y < dst_height; y++) {
//             unsigned char *dst_line = dst.virt_addr + y * dst.width_stride;
//             if (y < offset_y || y >= offset_y + resize_h) {
//                 // 黑色行已由 imfill 填充
//                 continue;
//             }
//             unsigned char *src_line = tmp_buf + (y - offset_y) * resize_w * pixel_size;
//             memcpy(dst_line + offset_x * pixel_size, src_line, resize_w * pixel_size);
//         }
//         delete[] tmp_buf;

//     } else if (src_fmt == IMAGE_FORMAT_YUV420SP_NV12 || src_fmt == IMAGE_FORMAT_YUV420SP_NV21) {
//         // Y平面
//         int y_stride = dst.width_stride;
//         int uv_stride = dst.width_stride;
//         unsigned char *tmp_y = new unsigned char[resize_h * resize_w];
//         unsigned char *tmp_uv = new unsigned char[resize_h / 2 * resize_w];

//         // copy Y plane
//         for (int y = 0; y < resize_h; y++) {
//             memcpy(tmp_y + y * resize_w, dst.virt_addr + y * y_stride, resize_w);
//         }
//         // copy UV plane
//         for (int y = 0; y < resize_h / 2; y++) {
//             memcpy(tmp_uv + y * resize_w, dst.virt_addr + y_stride * dst.height + y * uv_stride, resize_w);
//         }

//         // paste到居中位置
//         for (int y = 0; y < resize_h; y++) {
//             memcpy(dst.virt_addr + (y + offset_y) * y_stride + offset_x, tmp_y + y * resize_w, resize_w);
//         }
//         for (int y = 0; y < resize_h / 2; y++) {
//             memcpy(dst.virt_addr + y_stride * dst.height + (y + offset_y / 2) * uv_stride + offset_x, tmp_uv + y * resize_w, resize_w);
//         }

//         delete[] tmp_y;
//         delete[] tmp_uv;
//     }

//     /* ---------- 6. release ---------- */
//     releasebuffer_handle(src_handle);
//     releasebuffer_handle(dst_handle);

//     return 0;
// }
