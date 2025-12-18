#pragma once
/**
 * @file v4l2_camera.hpp
 * @brief 基于 V4L2 的摄像头采集封装类
 *
 * 特点：
 * 1. 使用 V4L2 多平面（MPLANE）接口
 * 2. 使用 MMAP 方式获取图像数据
 * 3. 支持：
 *    - 直接输出 OpenCV BGR 图像
 *    - 输出原始 NV12 buffer，供 MPP / RGA / NPU 使用
 */

#include <opencv2/opencv.hpp>   // cv::Mat、cvtColor 等
#include "image_utils.h"        // image_buffer_t 定义（图像描述结构体）

/**
 * @class V4L2Camera
 * @brief V4L2 摄像头采集类（NV12 / MMAP）
 *
 * 使用流程：
 *   1. V4L2Camera cam("/dev/video0", 1280, 720);
 *   2. cam.open();
 *   3. cam.grab(...) 或 cam.grab_buffer(...)
 *
 * 注意：
 * - 当前实现只申请 1 个 buffer，偏向低延迟
 * - buffer_ 始终指向同一块 mmap 内存
 */
class V4L2Camera {
public:
    /**
     * @brief 构造函数
     * @param dev  摄像头设备节点（如 "/dev/video0"）
     * @param w    图像宽度
     * @param h    图像高度
     */
    V4L2Camera(const char* dev, int w, int h);

    /**
     * @brief 析构函数
     *
     * 自动完成：
     * - STREAMOFF
     * - munmap
     * - close(fd)
     */
    ~V4L2Camera();

    /**
     * @brief 打开摄像头并完成初始化
     *
     * 内部执行流程：
     *   1. open() 打开设备节点
     *   2. init_device() 设置格式（NV12、分辨率）
     *   3. init_mmap() 申请并映射 buffer
     *
     * @return true 成功，false 失败
     */
    bool open();

    /**
     * @brief 抓取一帧并转换为 OpenCV 的 BGR 图像
     *
     * 适合：
     * - 显示
     * - OpenCV 算法
     * - 快速验证
     *
     * 内部流程：
     *   DQBUF → NV12 → BGR → QBUF
     *
     * @param out_bgr 输出的 BGR 图像
     * @return true 成功，false 失败
     */
    bool grab(cv::Mat& out_bgr);

    /**
     * @brief 抓取一帧原始 NV12 buffer
     *
     * 适合：
     * - MPP 编解码
     * - RGA 图像处理
     * - NPU 推理
     *
     * 注意：
     * - 返回的是 mmap 内存地址
     * - 下一次 grab 会复用该 buffer
     *
     * @param out_buf 输出图像描述结构
     * @return true 成功，false 失败
     */
    bool grab_buffer(image_buffer_t& out_buf);

private:
    /**
     * @brief 配置 V4L2 设备格式
     *
     * 设置内容：
     * - 视频类型：VIDEO_CAPTURE_MPLANE
     * - 分辨率
     * - 像素格式：NV12
     */
    bool init_device();

    /**
     * @brief 申请 buffer 并进行 mmap 映射
     *
     * 流程：
     *   VIDIOC_REQBUFS →
     *   VIDIOC_QUERYBUF →
     *   mmap →
     *   VIDIOC_QBUF →
     *   VIDIOC_STREAMON
     */
    bool init_mmap();

    /**
     * @brief 摄像头预热
     *
     * 丢弃前几帧，避免曝光、白平衡未稳定的问题
     *
     * @param frames 丢弃的帧数
     */
    void warmup(int frames);

private:
    /// 摄像头设备节点路径（如 "/dev/video0"）
    const char* dev_name_;

    /// 期望图像宽度
    int width_;

    /// 期望图像高度
    int height_;

    /// 摄像头设备文件描述符
    int fd_ = -1;

    /// mmap 后的图像数据虚拟地址（NV12）
    unsigned char* buffer_ = nullptr;

    /// mmap buffer 的总大小（字节）
    size_t buffer_len_ = 0;

    /**
     * @brief V4L2 buffer 结构指针（当前版本未使用）
     *
     * 预留字段：
     * - 方便后续扩展为多 buffer / 环形队列
     * - 可用于保存 DQBUF/QBUF 的状态
     */
    struct v4l2_buffer* v4l2_buf_;
};
