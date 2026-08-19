cmake_minimum_required(VERSION 3.10)

project(hand-eye-app)

# 查找 Qt5
find_package(Qt5 REQUIRED COMPONENTS Core Quick QuickControls2 Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# ============================================================
#  架构检测（与父 CMakeLists.txt 保持一致）
# ============================================================
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64")
    set(ARCH_TYPE "x86_64")
    set(ARCH_FLAG "-DX86_64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(ARCH_TYPE "ARM64")
    set(ARCH_FLAG "-DARM64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "armv7|armv7l")
    set(ARCH_TYPE "ARMv7")
    set(ARCH_FLAG "-DARM")
else()
    set(ARCH_TYPE "Unknown")
endif()


message("-- [${PROJECT_NAME}] Architecture: ${ARCH_TYPE}")
add_definitions(${ARCH_FLAG})


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
#  相机 SDK 检测（ORB 和 MVS 互斥）
# ============================================================

option(USE_ORB "Build with Orbbec camera (disable MVS)" ON)

if(USE_ORB)
    # ---- ORB 模式 ----
    if(EXISTS /usr/local/include/libobsensor)
        set(useORB_QT TRUE)
        set(useMVS_QT FALSE)
        set(CAM_SUFFIX "orb")
        set(ORBBEC_SDK_V2_DIR "/usr/local")
        add_definitions(-DuseORB -DDEFAULT_CAM_WIDTH=1280 -DDEFAULT_CAM_HEIGHT=800)
        message("-- [${PROJECT_NAME}] 相机: Orbbec (1280×800)")
    else()
        message(FATAL_ERROR "-- [${PROJECT_NAME}] USE_ORB=ON 但未找到 Orbbec SDK (/usr/local/include/libobsensor)")
    endif()
else()
    # ---- MVS 模式（显式 USE_ORB=OFF） ----
    set(useORB_QT FALSE)
    if(EXISTS /opt/MVS/include)
        set(useMVS_QT TRUE)
        set(CAM_SUFFIX "mvs")
        add_definitions(-DuseMVS -DDEFAULT_CAM_WIDTH=2448 -DDEFAULT_CAM_HEIGHT=2048)
        message("-- [${PROJECT_NAME}] 相机: MVS (2448×2048)")
    else()
        set(useMVS_QT FALSE)
        set(CAM_SUFFIX "nocam")
        add_definitions(-DDEFAULT_CAM_WIDTH=1280 -DDEFAULT_CAM_HEIGHT=800)
        message(WARNING "-- [${PROJECT_NAME}] MVS SDK 未找到，无相机支持")
    endif()
endif()

# ============================================================
#  源文件收集（排除 calibration.cpp，它有 main）
# ============================================================
set(GUI_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/DiagnosticLogger.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/CalibrationController.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/CameraImageProvider.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/auth_check.cpp
)

# MOC 需要处理的头文件
set(GUI_HDRS
    ${CMAKE_CURRENT_SOURCE_DIR}/DiagnosticLogger.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/CalibrationController.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/CameraImageProvider.hpp
)

# QML 资源文件（编译进二进制，不依赖外部路径）
set(QRC_FILE
    ${CMAKE_CURRENT_SOURCE_DIR}/resources.qrc
)

file(GLOB TOP_SRCS
    ${CMAKE_SOURCE_DIR}/*.cpp
    ${CMAKE_SOURCE_DIR}/calibration/*.cpp
    ${CMAKE_SOURCE_DIR}/circlesampler/*.cpp
    ${CMAKE_SOURCE_DIR}/piecesampler/*.cpp
    ${CMAKE_SOURCE_DIR}/testcameradeepth/*.cpp
    ${CMAKE_SOURCE_DIR}/utils/*.cpp
)
# 排除 calibration.cpp（含 main 函数）
list(REMOVE_ITEM TOP_SRCS "${CMAKE_SOURCE_DIR}/calibration.cpp")

# 相机源文件：ORB / MVS 互斥
if(useMVS_QT)
    list(APPEND TOP_SRCS ${CMAKE_SOURCE_DIR}/cameras/MVSCamera.cpp)
elseif(useORB_QT)
    list(APPEND TOP_SRCS ${CMAKE_SOURCE_DIR}/cameras/OrbbecCamera.cpp)
endif()

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

# MVS / ORB SDK include
if(useMVS_QT)
    target_include_directories(${PROJECT_NAME} PRIVATE /opt/MVS/include)
endif()
if(useORB_QT)
    target_include_directories(${PROJECT_NAME} PRIVATE ${ORBBEC_SDK_V2_DIR}/include)
    target_link_directories(${PROJECT_NAME} PRIVATE ${ORBBEC_SDK_V2_DIR}/lib)
endif()

# ============================================================
#  链接库
# ============================================================
set(QT_LIBS Qt5::Core Qt5::Quick Qt5::QuickControls2 Qt5::Widgets)

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
    elseif(ARCH_TYPE STREQUAL "ARM64")
        target_link_directories(${PROJECT_NAME} PRIVATE /opt/MVS/lib/aarch64)
    else()
        target_link_directories(${PROJECT_NAME} PRIVATE /opt/MVS/lib/32)
    endif()
    target_link_libraries(${PROJECT_NAME} MvCameraControl)
endif()

# Orbbec SDK
if(useORB_QT)
    target_link_libraries(${PROJECT_NAME} OrbbecSDK)
endif()

# ============================================================
#  编译选项
# ============================================================
set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    OUTPUT_NAME "hand-eye-${CAM_SUFFIX}"
)

# 构建身份必须在每次 build 时重新获取。若只在 configure 阶段执行，提交或
# 修改工作区后直接 cmake --build 会把旧 commit/dirty 状态编进新程序。
set(APP_BUILD_INFO_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(APP_BUILD_INFO_HEADER "${APP_BUILD_INFO_DIR}/AppBuildInfo.hpp")
add_custom_target(hand_eye_app_build_info ALL
    COMMAND ${CMAKE_COMMAND}
        -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
        -DOUTPUT_FILE=${APP_BUILD_INFO_HEADER}
        -DAPP_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/GenerateBuildInfo.cmake
    BYPRODUCTS ${APP_BUILD_INFO_HEADER}
    COMMENT "Refreshing hand-eye build metadata"
    VERBATIM
)
add_dependencies(${PROJECT_NAME} hand_eye_app_build_info)
target_include_directories(${PROJECT_NAME} PRIVATE ${APP_BUILD_INFO_DIR})
target_compile_definitions(${PROJECT_NAME} PRIVATE QT_MESSAGELOGCONTEXT)

# 继承父目录的宏定义（useMVS 等）
add_definitions(-DQT_GUI)
