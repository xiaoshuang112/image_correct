cmake_minimum_required(VERSION 3.10)

project(hand-eye-app)

# 查找 Qt5
find_package(Qt5 REQUIRED COMPONENTS Core Quick QuickControls2)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# ============================================================
#  架构检测（与父 CMakeLists.txt 保持一致）
# ============================================================
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64")
    set(ARCH_TYPE "x86_64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(ARCH_TYPE "ARM64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "armv7|armv7l")
    set(ARCH_TYPE "ARMv7")
else()
    set(ARCH_TYPE "Unknown")
endif()
message("-- [${PROJECT_NAME}] Architecture: ${ARCH_TYPE}")

# ============================================================
#  CGX SDK 路径
# ============================================================
set(CGX_SDK_DIR ${CMAKE_SOURCE_DIR}/libs/cr_sdk)

if(ARCH_TYPE STREQUAL "x86_64")
    set(CGX_SDK_LIBS
        ${CGX_SDK_DIR}/libs/x64/libcr_sdk.so
        ${CGX_SDK_DIR}/libs/x64/libLogStorageDll.so
        ${CGX_SDK_DIR}/libs/x64/libRobotConfigDll.so
        ${CGX_SDK_DIR}/libs/x64/libz.so
        ${CGX_SDK_DIR}/libs/x64/libprotobuf-c.so
        ${CGX_SDK_DIR}/libs/x64/libprotobuf-rpc.so
    )
elseif(ARCH_TYPE STREQUAL "ARM64")
    set(CGX_SDK_LIBS
        ${CGX_SDK_DIR}/libs/libcr_sdk.so
        ${CGX_SDK_DIR}/libs/libLogStorageDll.so
        ${CGX_SDK_DIR}/libs/libRobotConfigDll.so
        ${CGX_SDK_DIR}/libs/libz.so
        ${CGX_SDK_DIR}/libs/libprotobuf-c.so
        ${CGX_SDK_DIR}/libs/libprotobuf-rpc.so
    )
endif()

# ============================================================
#  检测 MVS SDK（父目录变量 useMVS 不传递到子目录，自行检测）
# ============================================================
if(EXISTS /opt/MVS/include)
    set(useMVS_QT TRUE)
    message("-- [${PROJECT_NAME}] MVS SDK detected at /opt/MVS")
else()
    set(useMVS_QT FALSE)
    message("-- [${PROJECT_NAME}] MVS SDK not found")
endif()

# ============================================================
#  源文件收集（排除 calibration.cpp，它有 main）
# ============================================================
set(GUI_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/CalibrationController.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/CameraImageProvider.cpp
)

# MOC 需要处理的头文件
set(GUI_HDRS
    ${CMAKE_CURRENT_SOURCE_DIR}/CalibrationController.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/CameraImageProvider.hpp
)

# QML 资源文件（编译进二进制，不依赖外部路径）
set(QRC_FILE
    ${CMAKE_CURRENT_SOURCE_DIR}/resources.qrc
)

file(GLOB TOP_SRCS
    ${CMAKE_SOURCE_DIR}/*.cpp
    ${CMAKE_SOURCE_DIR}/circlesampler/*.cpp
    ${CMAKE_SOURCE_DIR}/piecesampler/*.cpp
    ${CMAKE_SOURCE_DIR}/testcameradeepth/*.cpp
    ${CMAKE_SOURCE_DIR}/utils/*.cpp
)
# 排除 calibration.cpp（含 main 函数）
list(REMOVE_ITEM TOP_SRCS "${CMAKE_SOURCE_DIR}/calibration.cpp")

# 相机源文件：只添加当前启用的相机
file(GLOB CAM_SRCS ${CMAKE_SOURCE_DIR}/cameras/*.cpp)
if(useMVS_QT)
    list(APPEND TOP_SRCS ${CMAKE_SOURCE_DIR}/cameras/MVSCamera.cpp)
endif()
# Orbbec 和 OAK 默认不启用（与父项目 useORB=FALSE, useOAK=FALSE 一致）

# ============================================================
#  可执行文件
# ============================================================
add_executable(${PROJECT_NAME}
    ${GUI_SRCS}
    ${GUI_HDRS}
    ${TOP_SRCS}
    ${QRC_FILE}
)

# ============================================================
#  Include 目录
# ============================================================
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}                         # my_base.h, det.h 等
    ${CGX_SDK_DIR}/include                      # robotapi.h, basestruct.h
    ${OpenCV_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Quick_INCLUDE_DIRS}
    ${Qt5QuickControls2_INCLUDE_DIRS}
)

# MVS SDK include
if(useMVS_QT)
    target_include_directories(${PROJECT_NAME} PRIVATE /opt/MVS/include)
endif()

# ============================================================
#  链接库
# ============================================================
set(QT_LIBS Qt5::Core Qt5::Quick Qt5::QuickControls2)

target_link_libraries(${PROJECT_NAME}
    ${QT_LIBS}
    ${OpenCV_LIBS}
    # --no-as-needed 强制保留全部 CGX 库到 NEEDED，
    # 避免 libcr_sdk 的传递依赖在双击启动时找不到
    # --disable-new-dtags 用 RPATH 替代 RUNPATH（RPATH 对传递依赖也生效）
    -Wl,--no-as-needed
    ${CGX_SDK_LIBS}
    -Wl,--as-needed
    -Wl,--disable-new-dtags
)

# MVS SDK
if(useMVS_QT)
    if(ARCH_TYPE STREQUAL "x86_64")
        target_link_directories(${PROJECT_NAME} PRIVATE /opt/MVS/lib/64)
    else()
        target_link_directories(${PROJECT_NAME} PRIVATE /opt/MVS/lib/32)
    endif()
    target_link_libraries(${PROJECT_NAME} MvCameraControl)
endif()

# ============================================================
#  编译选项
# ============================================================
set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

# 继承父目录的宏定义（useMVS 等）
add_definitions(-DQT_GUI)
