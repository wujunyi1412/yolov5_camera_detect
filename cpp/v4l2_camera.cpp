#include "v4l2_camera.hpp"
#include <fcntl.h>          // open
#include <unistd.h>         // close
#include <sys/ioctl.h>      // ioctl
#include <sys/mman.h>       // mmap / munmap
#include <linux/videodev2.h>// V4L2 定义
#include <cstring>
#include <iostream>
// #include "rkmedia_api.h"
/**
 * 构造函数
 * @param dev  摄像头设备节点，如 "/dev/video0"
 * @param w    期望采集的图像宽
 * @param h    期望采集的图像高
 */
V4L2Camera::V4L2Camera(const char* dev, int w, int h)
    : dev_name_(dev), width_(w), height_(h) {}

/**
 * 析构函数
 * 负责释放 V4L2 资源，保证程序退出时设备状态干净
 */
V4L2Camera::~V4L2Camera() {
    if (fd_ >= 0) {
        // 1. 停止视频流
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        ioctl(fd_, VIDIOC_STREAMOFF, &type);

        // 2. 解除 mmap 映射
        if (buffer_)
            munmap(buffer_, buffer_len_);

        // 3. 关闭设备文件
        close(fd_);
    }
}

/**
 * 打开摄像头并完成初始化
 * @return true 成功，false 失败
 */
bool V4L2Camera::open() {
    // 1. 打开设备节点
    fd_ = ::open(dev_name_, O_RDWR);
    if (fd_ < 0) {
        perror("open");
        return false;
    }

    // 2. 设置设备格式
    // 3. 申请缓冲区并 mmap
    return init_device() && init_mmap();
}

/**
 * 配置摄像头的采集格式
 * 包括分辨率、像素格式、plane 数量
 */
bool V4L2Camera::init_device() {
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));

    // 使用多平面采集接口（RK3588 常用）
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

    // 设置分辨率
    fmt.fmt.pix_mp.width  = width_;
    fmt.fmt.pix_mp.height = height_;

    // 设置像素格式：NV12（YUV420SP）
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;

    // NV12 在 V4L2 里通常只占 1 个 plane
    fmt.fmt.pix_mp.num_planes  = 1;

    // 应用格式到驱动
    if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
        perror("VIDIOC_S_FMT");
        return false;
    }
    return true;
}

/**
 * 申请缓冲区并通过 mmap 映射到用户空间
 */
bool V4L2Camera::init_mmap() {
    struct v4l2_requestbuffers req{};
    req.count  = 1;   // 只申请 1 个 buffer（最简单示例）
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req.memory = V4L2_MEMORY_MMAP;

    // 向驱动申请 buffer
    if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
        perror("VIDIOC_REQBUFS");
        return false;
    }

    // v4l2_buffer 描述一个 buffer
    static struct v4l2_plane planes[1];
    static struct v4l2_buffer buf;

    memset(&buf, 0, sizeof(buf));
    buf.type     = req.type;
    buf.memory   = req.memory;
    buf.index    = 0;         // 只有一个 buffer
    buf.m.planes = planes;
    buf.length   = 1;         // plane 数量

    // 查询 buffer 的物理信息（offset、长度）
    if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
        perror("VIDIOC_QUERYBUF");
        return false;
    }

    // 保存 buffer 大小
    buffer_len_ = buf.m.planes[0].length;

    // mmap：把内核 buffer 映射到用户空间
    buffer_ = (unsigned char*)mmap(
        NULL,
        buffer_len_,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        fd_,
        buf.m.planes[0].m.mem_offset
    );

    if (buffer_ == MAP_FAILED) {
        perror("mmap");
        return false;
    }

    // 将 buffer 放入采集队列
    ioctl(fd_, VIDIOC_QBUF, &buf);

    // 开启视频流
    int type = buf.type;
    ioctl(fd_, VIDIOC_STREAMON, &type);

    // 丢弃前几帧（ISP/曝光未稳定）
    warmup(10);
    return true;
}

/**
 * 预热摄像头
 * 连续取出并放回 buffer，不做任何处理
 */
void V4L2Camera::warmup(int frames) {
    struct v4l2_buffer buf{};
    struct v4l2_plane planes[1]{};

    buf.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.memory   = V4L2_MEMORY_MMAP;
    buf.m.planes = planes;
    buf.length   = 1;

    for (int i = 0; i < frames; ++i) {
        ioctl(fd_, VIDIOC_DQBUF, &buf); // 取出一帧
        ioctl(fd_, VIDIOC_QBUF, &buf);  // 放回队列
    }
}

/**
 * 抓取一帧并转为 OpenCV 的 BGR 图像
 */
bool V4L2Camera::grab(cv::Mat& out_bgr) {
    struct v4l2_buffer buf{};
    struct v4l2_plane planes[1]{};

    buf.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.memory   = V4L2_MEMORY_MMAP;
    buf.m.planes = planes;
    buf.length   = 1;

    // 从驱动取出一帧
    if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0)
        return false;

    // NV12 数据组织：Y(height) + UV(height/2)
    cv::Mat yuv(height_ + height_ / 2, width_, CV_8UC1, buffer_);

    // NV12 → BGR
    cv::cvtColor(yuv, out_bgr, cv::COLOR_YUV2BGR_NV12);

    // 用完后放回 buffer
    ioctl(fd_, VIDIOC_QBUF, &buf);
    return true;
}

/**
 * 抓取一帧原始 NV12 数据，直接交给下游（如 MPP / RGA）
 */
bool V4L2Camera::grab_buffer(image_buffer_t& out_buf) {
    struct v4l2_buffer buf{};
    struct v4l2_plane planes[1]{};

    buf.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.memory   = V4L2_MEMORY_MMAP;
    buf.m.planes = planes;
    buf.length   = 1;

    // 取出一帧
    if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0)
        return false;

    // 填充 image_buffer_t 结构
    out_buf.width          = width_;
    out_buf.height         = height_;
    out_buf.width_stride   = width_;
    out_buf.height_stride  = height_;
    out_buf.format         = IMAGE_FORMAT_YUV420SP_NV12;
    out_buf.virt_addr      = buffer_;
    out_buf.vector_virt_addr = nullptr;
    out_buf.size           = buffer_len_;
    out_buf.fd             = fd_;   // 注意：这不是 dmabuf fd，仅是设备 fd

    // 放回队列，供下一帧使用
    ioctl(fd_, VIDIOC_QBUF, &buf);
    return true;
}





// class RKMPICamera {
// public:
//     RKMPICamera(int dev_id, int w, int h)
//         : dev_id_(dev_id), width_(w), height_(h), is_open_(false) {}

//     ~RKMPICamera() { close(); }

//     bool open() {
//         RK_MPI_SYS_Init();

//         memset(&cam_attr_, 0, sizeof(cam_attr_));
//         cam_attr_.u32DeviceId = dev_id_;
//         cam_attr_.u32Width    = width_;
//         cam_attr_.u32Height   = height_;
//         cam_attr_.enPixFmt    = PIX_FMT_NV12;
//         cam_attr_.u32FrmRate  = 25;

//         if (RK_MPI_Camera_Init(&cam_attr_) != 0) {
//             std::cerr << "RK_MPI_Camera_Init failed!" << std::endl;
//             return false;
//         }

//         // 手动曝光
//         RK_MPI_ISP_SetExposureMode(dev_id_, RK_ISP_EXPOSURE_MANUAL);
//         RK_MPI_ISP_SetExposureTime(dev_id_, 1.0f / 4);
//         RK_MPI_ISP_SetGain(dev_id_, 2.0f);

//         // 手动白平衡
//         RK_MPI_ISP_SetWhiteBalanceMode(dev_id_, RK_ISP_WB_MANUAL);
//         RK_MPI_ISP_SetWhiteBalanceGain(dev_id_, 1.2f, 1.0f, 1.0f);

//         is_open_ = true;
//         return true;
//     }

//     void close() {
//         if (is_open_) {
//             RK_MPI_Camera_DeInit(dev_id_);
//             is_open_ = false;
//         }
//     }

//     // 抓取一帧 NV12 数据，类似原来的 grab_buffer
//     bool grab_buffer(image_buffer_t& out_buf, int timeout_ms = 1000) {
//         if (!is_open_) return false;

//         IMAGE_BUFFER_S buf;
//         if (RK_MPI_Camera_GetFrame(dev_id_, &buf, timeout_ms) != 0)
//             return false;

//         // 填充 image_buffer_t 结构
//         out_buf.width           = buf.u32Width;
//         out_buf.height          = buf.u32Height;
//         out_buf.width_stride    = buf.u32Stride[0];
//         out_buf.height_stride   = buf.u32Height;
//         out_buf.format          = IMAGE_FORMAT_YUV420SP_NV12;
//         out_buf.virt_addr       = buf.pVirAddr;
//         out_buf.vector_virt_addr = nullptr;
//         out_buf.size            = buf.u32BufSize;
//         out_buf.fd              = -1; // RKMPI 不是 dmabuf，这里填 -1

//         // 放回帧队列
//         RK_MPI_Camera_ReleaseFrame(dev_id_, &buf);
//         return true;
//     }

// private:
//     int dev_id_;
//     int width_;
//     int height_;
//     bool is_open_;
//     CAMERA_ATTR_S cam_attr_;
// };
