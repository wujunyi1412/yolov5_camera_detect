#include "video_capture.h"
#include <cstring>
#include <ctime>
#include "rk_mpi_vi.h"
#include "rk_mpi_sys.h"

VideoCapture::VideoCapture()
    : m_devId(0), m_pipeId(0), m_chnId(0), m_initialized(false) {
    memset(&m_chnAttr, 0, sizeof(VI_CHN_ATTR_S));
}

VideoCapture::~VideoCapture() {
    if (m_initialized) {
        RK_MPI_VI_DisableChn(m_devId, m_chnId);
        RK_MPI_VI_DisableDev(m_devId);
        // RK_MPI_SYS_Exit();
        m_initialized = false;
    }
}


uint64_t VideoCapture::getNowUs() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000 + ts.tv_nsec / 1000;
}

bool VideoCapture::init(int width, int height, int devId, int chnId) {
    m_devId = devId;
    m_pipeId = devId;
    m_chnId = chnId;

    // 1. 获取设备属性
    VI_DEV_ATTR_S stDevAttr;
    VI_DEV_BIND_PIPE_S stBindPipe;
    memset(&stDevAttr, 0, sizeof(stDevAttr));
    memset(&stBindPipe, 0, sizeof(stBindPipe));

    int ret = RK_MPI_VI_GetDevAttr(m_devId, &stDevAttr);
    if (ret == RK_ERR_VI_NOT_CONFIG) {
        ret = RK_MPI_VI_SetDevAttr(m_devId, &stDevAttr);
        if (ret != RK_SUCCESS) return false;
    }

    ret = RK_MPI_VI_GetDevIsEnable(m_devId);
    if (ret != RK_SUCCESS) {
        ret = RK_MPI_VI_EnableDev(m_devId);
        if (ret != RK_SUCCESS) return false;

        stBindPipe.u32Num = 1;
        stBindPipe.PipeId[0] = m_pipeId;
        ret = RK_MPI_VI_SetDevBindPipe(m_devId, &stBindPipe);
        if (ret != RK_SUCCESS) return false;
    }

    // 2. 设置通道属性
    memset(&m_chnAttr, 0, sizeof(VI_CHN_ATTR_S));
    m_chnAttr.stIspOpt.u32BufCount = 2; // 缓冲区数量
    m_chnAttr.stIspOpt.enMemoryType = VI_V4L2_MEMORY_TYPE_DMABUF;
    m_chnAttr.stSize.u32Width = width;
    m_chnAttr.stSize.u32Height = height;
    m_chnAttr.enPixelFormat = RK_FMT_YUV420SP;
    m_chnAttr.enCompressMode = COMPRESS_MODE_NONE; // 不压缩
    m_chnAttr.u32Depth = 2; // 获取队列深度

    ret = RK_MPI_VI_SetChnAttr(m_devId, m_chnId, &m_chnAttr);
    if (ret != RK_SUCCESS) return false;

    ret = RK_MPI_VI_EnableChn(m_devId, m_chnId);
    if (ret != RK_SUCCESS) return false;


    m_initialized = true;
    return true;
}

bool VideoCapture::read(VIDEO_FRAME_INFO_S& frame, int timeoutMs) {
    if (!m_initialized) return false;
    int ret = RK_MPI_VI_GetChnFrame(m_devId, m_chnId, &frame, timeoutMs); // 阻塞获取
    return ret == RK_SUCCESS;
}

bool VideoCapture::release(const VIDEO_FRAME_INFO_S& frame) {
    if (!m_initialized) return false;
    int ret = RK_MPI_VI_ReleaseChnFrame(m_devId, m_chnId, &frame);
    return ret == RK_SUCCESS;
}
