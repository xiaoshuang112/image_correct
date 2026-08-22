cmake_minimum_required(VERSION 3.10)
project(caitu)

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# ============================================================
#  相机后端选择：CAMERA_BACKEND=ORB|MVS（默认 ORB，保持现有产线产物）
#  兼容旧入口 -DUSE_ORB=ON/OFF；两者同时给出且矛盾时直接报错。
# ============================================================
set(CAMERA_BACKEND "" CACHE STRING "Camera backend: ORB or MVS (default ORB)")
if(CAMERA_BACKEND STREQUAL "")
    if(DEFINED USE_ORB)
        if(USE_ORB)
            set(CAMERA_BACKEND "ORB")
        else()
            set(CAMERA_BACKEND "MVS")
        endif()
    else()
        set(CAMERA_BACKEND "ORB")
    endif()
    set(CAMERA_BACKEND "${CAMERA_BACKEND}" CACHE STRING "Camera backend: ORB or MVS (default ORB)" FORCE)
else()
    string(TOUPPER "${CAMERA_BACKEND}" CAMERA_BACKEND)
    set(CAMERA_BACKEND "${CAMERA_BACKEND}" CACHE STRING "Camera backend: ORB or MVS (default ORB)" FORCE)
    if(DEFINED USE_ORB)
        if((USE_ORB AND NOT CAMERA_BACKEND STREQUAL "ORB") OR
           (NOT USE_ORB AND NOT CAMERA_BACKEND STREQUAL "MVS"))
            message(FATAL_ERROR
                "USE_ORB=${USE_ORB} 与 CAMERA_BACKEND=${CAMERA_BACKEND} 矛盾，请只保留一种写法")
        endif()
    endif()
endif()
if(NOT CAMERA_BACKEND STREQUAL "ORB" AND NOT CAMERA_BACKEND STREQUAL "MVS")
    message(FATAL_ERROR "CAMERA_BACKEND 只支持 ORB 或 MVS，当前值: '${CAMERA_BACKEND}'")
endif()

SET(useORB FALSE)
SET(useOAK FALSE)
SET(useMVS FALSE)
if(CAMERA_BACKEND STREQUAL "ORB")
    SET(useORB TRUE)
elseif(CAMERA_BACKEND STREQUAL "MVS")
    SET(useMVS TRUE)
endif()
message(STATUS "Camera backend: ${CAMERA_BACKEND}")


if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

# 禁用 MSVC 安全警告
if(MSVC)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
endif()

find_package(OpenCV REQUIRED)

# 获取系统架构信息
message("System Processor: ${CMAKE_SYSTEM_PROCESSOR}")

# 判断架构
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

message("Architecture Type: ${ARCH_TYPE}")

 
# Orbbec SDK v2 路径
if(WIN32)
	set(ORBBEC_SDK_V2_DIR "F:\Program Files\OrbbecSDK 2.4.11")
    set(CGX_SDK_LIBS )      
else()
    # # public
    set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR})
    include_directories(${SRC_DIR})
    include_directories(${SRC_DIR}/calibration)

    set(depthai_DIR "/usr/local")

    # 根据架构设置不同的编译选项
    if(ARCH_TYPE STREQUAL "x86_64")
        set(ORBBEC_SDK_V2_DIR "/usr/local")
        # Include directory for your project
        set(CGX_SDK_DIR  ${SRC_DIR}/libs/cr_sdk)
        include_directories(${CGX_SDK_DIR}/include)      

        set(CGX_SDK_LIBS
            ${CGX_SDK_DIR}/libs/x64/libcr_sdk.so
            ${CGX_SDK_DIR}/libs/x64/libLogStorageDll.so
            ${CGX_SDK_DIR}/libs/x64/libRobotConfigDll.so
            ${CGX_SDK_DIR}/libs/x64/libz.so
            ${CGX_SDK_DIR}/libs/x64/libprotobuf-c.so
            ${CGX_SDK_DIR}/libs/x64/libprotobuf-rpc.so
        )

    elseif(ARCH_TYPE STREQUAL "ARM64")
        set(ORBBEC_SDK_V2_DIR "/usr/local")

        set(CGX_SDK_DIR  ${SRC_DIR}/libs/cr_sdk)
        include_directories(${CGX_SDK_DIR}/include)    
        set(CGX_SDK_LIBS
            ${CGX_SDK_DIR}/libs/libcr_sdk.so
            ${CGX_SDK_DIR}/libs/libLogStorageDll.so
            ${CGX_SDK_DIR}/libs/libRobotConfigDll.so
            ${CGX_SDK_DIR}/libs/libz.so
            ${CGX_SDK_DIR}/libs/libprotobuf-c.so
            ${CGX_SDK_DIR}/libs/libprotobuf-rpc.so
        )        

    endif()

endif()


include_directories(${ORBBEC_SDK_V2_DIR}/include)
link_directories(${ORBBEC_SDK_V2_DIR}/lib)

include_directories(${depthai_DIR}/include)
link_directories(${depthai_DIR}/lib)

# MVS SDK：头文件和当前架构的运行库都必须存在，否则配置直接失败，
# 不允许生成名为 MVS、实际无相机支持的二进制。
if(useMVS)
    set(MVS_SDK_DIR "/opt/MVS" CACHE PATH "MVS SDK install prefix")
    if(NOT EXISTS "${MVS_SDK_DIR}/include/MvCameraControl.h")
        message(FATAL_ERROR
            "CAMERA_BACKEND=MVS 但未找到 MVS SDK 头文件: "
            "${MVS_SDK_DIR}/include/MvCameraControl.h")
    endif()

    if(ARCH_TYPE STREQUAL "x86_64")
        set(MVS_LIB_DIR "${MVS_SDK_DIR}/lib/64")
    elseif(ARCH_TYPE STREQUAL "ARM64")
        set(MVS_LIB_DIR "${MVS_SDK_DIR}/lib/aarch64")
    else()
        set(MVS_LIB_DIR "${MVS_SDK_DIR}/lib/32")
    endif()
    if(NOT EXISTS "${MVS_LIB_DIR}/libMvCameraControl.so")
        message(FATAL_ERROR
            "CAMERA_BACKEND=MVS 但未找到 ${ARCH_TYPE} 架构的 MVS 运行库: "
            "${MVS_LIB_DIR}/libMvCameraControl.so")
    endif()

    include_directories(${MVS_SDK_DIR}/include)
    link_directories(${MVS_LIB_DIR})
    message(STATUS "MVS SDK: ${MVS_SDK_DIR} (lib: ${MVS_LIB_DIR})")
endif()


SET(SRC_LISTS
calibration/charuco_detector.cpp calibration/auto_eye_in_hand_charuco_calibration.cpp calibration/eye_to_hand_calibration.cpp calibration/camera_calibration.cpp calibration/calibration_utils.cpp calibration/calibration_session.cpp calibration/intrinsic_identity.cpp calibration/semi_auto_eye_in_hand_calibration.cpp
    calibration.cpp utils/exposure_fusion.cpp
    circlesampler/CircleSampler.cpp circlesampler/Config.cpp
    piecesampler/PieceSampler.cpp piecesampler/Config.cpp
    testcameradeepth/DepthTestConfig.cpp testcameradeepth/DepthRemedy.cpp testcameradeepth/DepthTester.cpp
    utils/putText.cpp utils/Util.cpp utils/Texts.cpp utils/robot_util.cpp utils/calib_transform.cpp
)

SET(SRC_LIBS 
)
set(CAITU_COMPILE_DEFINITIONS)

if(useORB)
    list(APPEND CAITU_COMPILE_DEFINITIONS useORB)
    list(APPEND SRC_LISTS
        cameras/OrbbecCamera.cpp
    )
    list(APPEND SRC_LIBS
        OrbbecSDK
    )
endif()

if(useOAK)
    list(APPEND CAITU_COMPILE_DEFINITIONS useOAK)
    list(APPEND SRC_LISTS
       cameras/OAKCamera.cpp
        # cameras/OrbbecCamera.cpp
    )
    list(APPEND SRC_LIBS
        depthai-core
    )
endif()

if(useMVS)
    list(APPEND CAITU_COMPILE_DEFINITIONS useMVS)
    list(APPEND SRC_LISTS
        cameras/MVSCamera.cpp
        cameras/mvs_device_selector.cpp
        cameras/mvs_error.cpp
    )
    list(APPEND SRC_LIBS
        MvCameraControl
    )
endif()


add_executable(${PROJECT_NAME} ${SRC_LISTS})
target_compile_definitions(${PROJECT_NAME} PRIVATE ${CAITU_COMPILE_DEFINITIONS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${CGX_SDK_LIBS} ${SRC_LIBS} )

# MVS 产物优先从二进制旁的 lib/mvs 加载运行库（发布包内捆绑），
# 开发机上回退到 SDK 安装目录。ORB 产物不引用任何 MVS 路径。
if(useMVS)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        BUILD_WITH_INSTALL_RPATH OFF
        INSTALL_RPATH "$ORIGIN/lib/mvs"
        BUILD_RPATH "$ORIGIN/lib/mvs;${MVS_LIB_DIR}")
endif()

# 标定审计报告需要记录构建身份。与 GUI 使用同一个生成脚本，并在每次
# build 时刷新，避免复用 configure 阶段的旧 commit/dirty 状态。
set(CALIBRATION_BUILD_INFO_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(CALIBRATION_BUILD_INFO_HEADER "${CALIBRATION_BUILD_INFO_DIR}/AppBuildInfo.hpp")
add_custom_target(calibration_build_info ALL
    COMMAND ${CMAKE_COMMAND}
        -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
        -DOUTPUT_FILE=${CALIBRATION_BUILD_INFO_HEADER}
        -DAPP_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        -DAPP_CAMERA_BACKEND=${CAMERA_BACKEND}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/hand_eye_gui/GenerateBuildInfo.cmake
    BYPRODUCTS ${CALIBRATION_BUILD_INFO_HEADER}
    COMMENT "Refreshing calibration audit build metadata"
    VERBATIM
)
add_dependencies(${PROJECT_NAME} calibration_build_info)
target_include_directories(${PROJECT_NAME} PRIVATE ${CALIBRATION_BUILD_INFO_DIR})

# ============================================================
#  反畸变工具
# ============================================================
add_executable(anti_distortion tools/anti_distortion.cpp)
target_link_libraries(anti_distortion ${OpenCV_LIBS})

# ============================================================
#  FK 验证工具
# ============================================================
add_executable(verify_fk tools/verify_fk.cpp)
target_link_libraries(verify_fk ${CGX_SDK_LIBS})

# ============================================================
# 眼在手内离线复算工具（不连接机器人/相机）
# ============================================================
add_executable(offline_eye_in_hand
    tools/offline_eye_in_hand.cpp
    calibration/charuco_detector.cpp
    calibration/camera_calibration.cpp
    calibration/calibration_utils.cpp
    calibration/calibration_session.cpp
    calibration/semi_auto_eye_in_hand_calibration.cpp
    utils/calib_transform.cpp
    utils/robot_util.cpp
)
target_include_directories(offline_eye_in_hand PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/calibration
    ${CGX_SDK_DIR}/include
    ${CALIBRATION_BUILD_INFO_DIR}
)
target_link_libraries(offline_eye_in_hand ${OpenCV_LIBS} ${CGX_SDK_LIBS})
add_dependencies(offline_eye_in_hand calibration_build_info)

# ============================================================
#  无硬件单元测试（纯 C++ 助手：选机策略/错误码/内参门禁/配置解析）
# ============================================================
option(BUILD_TESTS "Build no-hardware unit tests" ON)
if(BUILD_TESTS)
    add_executable(caitu_tests
        tests/run_tests.cpp
        tests/test_mvs_device_selector.cpp
        tests/test_mvs_error.cpp
        tests/test_intrinsic_identity.cpp
        tests/test_config_resolver.cpp
        cameras/mvs_device_selector.cpp
        cameras/mvs_error.cpp
        calibration/intrinsic_identity.cpp
        hand_eye_gui/config_resolver.cpp
    )
    target_include_directories(caitu_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests
    )
    target_link_libraries(caitu_tests ${OpenCV_LIBS})
    enable_testing()
    add_test(NAME caitu_tests COMMAND caitu_tests)
endif()

# ============================================================
#  Qt5 QML 手眼标定 GUI（可选）
# ============================================================
option(BUILD_QT_GUI "Build Qt QML GUI for hand-eye calibration" ON)
if(BUILD_QT_GUI)
    find_package(Qt5 QUIET COMPONENTS Core Quick QuickControls2 Widgets)
    if(Qt5_FOUND)
        add_subdirectory(hand_eye_gui)
        message(STATUS "Qt5 GUI enabled → target 'hand-eye-app'")
    else()
        message(WARNING "Qt5 not found, skipping GUI target")
    endif()
endif()
