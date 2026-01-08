#ifndef VIDEO_CAPTURE_H
#define VIDEO_CAPTURE_H

#include <cstdint>
#include "rk_mpi_vi.h"

class VideoCapture {
public:
    VideoCapture();
    ~VideoCapture();

    // 初始化设备和通道
    bool init(int width, int height, int devId = 0, int chnId = 0);

    // 获取一帧视频，阻塞方式
    bool read(VIDEO_FRAME_INFO_S& frame, int timeoutMs = -1);

    // 释放一帧视频
    bool release(const VIDEO_FRAME_INFO_S& frame);

    // 获取微秒时间
    static uint64_t getNowUs();

private:
    int m_devId;
    int m_pipeId;
    int m_chnId;
    VI_CHN_ATTR_S m_chnAttr;
    bool m_initialized;
};

#endif // VIDEO_CAPTURE_H
