# -----------------------
# 1. 设置 CMake 最低版本和项目名
# -----------------------
cmake_minimum_required(VERSION 3.10)
project(rknn_yolov5_demo)

# -----------------------
# 2. AddressSanitizer (内存检测工具) 支持
# -----------------------
if (ENABLE_ASAN)
    message(STATUS "BUILD WITH ADDRESS SANITIZER")
    # Debug 编译时启用 ASAN，添加必要的编译和链接选项
    set (CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
    set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
    set (CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
endif ()

# -----------------------
# 3. 根据目标 SOC 设置对应的 yolov5 源文件和宏
# -----------------------
set(rknpu_yolov5_file rknpu2/yolov5.cc)

if (TARGET_SOC STREQUAL "rv1106" OR TARGET_SOC STREQUAL "rv1103")
    add_definitions(-DRV1106_1103)  # 定义宏，编译时可用
    set(rknpu_yolov5_file rknpu2/yolov5_rv1106_1103.cc)
    # 为 RV1106/1103 包含 DMA allocator 头文件
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/allocator/dma)

elseif(TARGET_SOC STREQUAL "rk1808" OR TARGET_SOC STREQUAL "rv1109" OR TARGET_SOC STREQUAL "rv1126")
    add_definitions(-DRKNPU1)       # 定义 RKNPU1 宏
    set(rknpu_yolov5_file rknpu1/yolov5.cc)
endif()

# -----------------------
# 4. 添加子目录
# -----------------------
# 3rdparty 和 utils 可能包含库或头文件
# 这里 add_subdirectory 会编译这些子项目并生成 targets
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/ 3rdparty.out)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../utils/ utils.out)

set(OpenCV_DIR  /mnt/c/Users/11198/Documents/C++_libs/opencv-4.5.3/arm64_rk3588_install/lib/cmake/opencv4/)
include_directories(/mnt/c/Users/11198/Documents/C++_libs/opencv-4.5.3/arm64_rk3588_install/include/opencv4/)
include_directories(/usr/include/)

find_package(OpenCV REQUIRED)
# -----------------------
# 5. 设置可执行文件 rpath（运行时库查找路径）
# -----------------------
set(CMAKE_INSTALL_RPATH "$ORIGIN/lib")

# -----------------------
# 6. 搜索当前目录下所有 .cc 文件（如果需要）
# -----------------------
file(GLOB SRCS ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)

# -----------------------
# 7. 定义可执行文件
# -----------------------
add_executable(${PROJECT_NAME}
    main.cc
    postprocess.cc
    v4l2_camera.cpp
    AudioPlayer.cpp
    ${rknpu_yolov5_file}  # 根据 SOC 选择的 yolov5 源文件
)

# -----------------------
# 8. 链接库
# -----------------------
target_link_libraries(${PROJECT_NAME}
    imageutils      # utils 子项目库
    fileutils       # utils 子项目库
    imagedrawing    # utils 子项目库
    ${LIBRKNNRT}    # RKNN 运行时库
    dl              # Linux 动态链接库
    /mnt/c/Users/11198/Documents/C++_libs/opencv-4.5.3/arm64_rk3588_install/lib/libopencv_core.so
    /mnt/c/Users/11198/Documents/C++_libs/opencv-4.5.3/arm64_rk3588_install/lib/libopencv_imgcodecs.so
    /mnt/c/Users/11198/Documents/C++_libs/opencv-4.5.3/arm64_rk3588_install/lib/libopencv_imgproc.so
    /mnt/c/Users/11198/Documents/C++_libs/opencv-4.5.3/arm64_rk3588_install/lib/libopencv_highgui.so
    /mnt/c/Users/11198/Documents/C++_libs/zlib-1.2.11/arm64_rk3588_install/lib/libz.so
)

# Android 平台额外链接 log 库
if (CMAKE_SYSTEM_NAME STREQUAL "Android")
    target_link_libraries(${PROJECT_NAME}
        log
    )
endif()

# Linux 平台链接线程库
if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(THREADS_PREFER_PTHREAD_FLAG ON)
    find_package(Threads REQUIRED)
    target_link_libraries(${PROJECT_NAME} Threads::Threads)
endif()

# -----------------------
# 9. 添加头文件搜索路径
# -----------------------
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}  # 当前目录
    ${LIBRKNNRT_INCLUDES}        # RKNN 头文件路径
)

# -----------------------
# 10. 安装规则
# -----------------------
install(TARGETS ${PROJECT_NAME} DESTINATION .)  # 安装可执行文件到当前目录
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../model/bus.jpg DESTINATION ./model)  # 安装模型相关 jpg
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../model/coco_80_labels_list.txt DESTINATION ./model)  # 标签文件
file(GLOB RKNN_FILES "${CMAKE_CURRENT_SOURCE_DIR}/../model/*.rknn")  # 找到所有 .rknn 模型文件
install(FILES ${RKNN_FILES} DESTINATION model)  # 安装模型文件
