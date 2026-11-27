cmake_minimum_required(VERSION 3.10)
project(caitu VERSION 1.0.0)

# 对外发布的软件版本号。提交号和工作区状态用于追溯，版本号用于现场界面和日志。
set(CAITU_VERSION "${PROJECT_VERSION}" CACHE STRING "Caitu application version")

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# MVS is the only supported camera backend. Reject stale backend selections.
set(CAMERA_BACKEND "MVS" CACHE STRING "Camera backend (MVS only)")
string(TOUPPER "${CAMERA_BACKEND}" CAMERA_BACKEND)
if(NOT CAMERA_BACKEND STREQUAL "MVS")
    message(FATAL_ERROR "Only CAMERA_BACKEND=MVS is supported; clear the old backend selection")
endif()
if(USE_ORB)
    message(FATAL_ERROR "USE_ORB is no longer supported; remove it and use MVS")
endif()
message(STATUS "Camera backend: ${CAMERA_BACKEND}")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

# 禁用 MSVC 安全警告
if(MSVC)
    # Windows.h 的 min/max 宏会破坏 std::max 与 std::min，MSVC 构建统一加 NOMINMAX。
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS NOMINMAX)
    # 源码是无 BOM 的 UTF-8（含中文字面量），MSVC 默认按系统 ANSI 代码页解析。
    add_compile_options(/utf-8)
    if(CMAKE_GENERATOR MATCHES "Ninja")
        # Decode the localized /showIncludes prefix explicitly for Ninja.
        execute_process(COMMAND cmd /c chcp 65001 OUTPUT_QUIET ERROR_QUIET)
        set(INCLUDE_PROBE "${CMAKE_BINARY_DIR}/CMakeFiles/caitu_include_probe.cpp")
        file(WRITE "${INCLUDE_PROBE}" "#include <stddef.h>\n")
        execute_process(COMMAND "${CMAKE_CXX_COMPILER}" /nologo /showIncludes /c "${INCLUDE_PROBE}"
                        "/Fo${CMAKE_BINARY_DIR}/CMakeFiles/caitu_include_probe.obj"
                        OUTPUT_VARIABLE INCLUDE_PROBE_OUTPUT
                        ERROR_VARIABLE INCLUDE_PROBE_ERROR
                        RESULT_VARIABLE INCLUDE_PROBE_RESULT ENCODING UTF-8)
        string(REGEX MATCH "(^|[\r\n])([^\r\n]+: +)[A-Za-z]:[/\\\\]"
               INCLUDE_PROBE_MATCH "${INCLUDE_PROBE_OUTPUT}${INCLUDE_PROBE_ERROR}")
        if(NOT INCLUDE_PROBE_RESULT EQUAL 0 OR NOT INCLUDE_PROBE_MATCH)
            message(FATAL_ERROR "Cannot detect MSVC header dependency prefix: ${INCLUDE_PROBE_ERROR}")
        endif()
        set(CMAKE_CL_SHOWINCLUDES_PREFIX "${CMAKE_MATCH_2}")
    endif()
endif()

option(BUILD_QT_GUI "Build Qt QML collection GUI" ON)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d photo)

# 获取系统架构信息
message("System Processor: ${CMAKE_SYSTEM_PROCESSOR}")

# 判断架构
string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" CAITU_SYSTEM_PROCESSOR_LOWER)
if(CAITU_SYSTEM_PROCESSOR_LOWER MATCHES "x86_64|amd64")
    set(ARCH_TYPE "x86_64")
elseif(CAITU_SYSTEM_PROCESSOR_LOWER MATCHES "aarch64|arm64")
    set(ARCH_TYPE "ARM64")
elseif(CAITU_SYSTEM_PROCESSOR_LOWER MATCHES "armv7|armv7l")
    set(ARCH_TYPE "ARMv7")
elseif(MSVC AND CMAKE_SIZEOF_VOID_P EQUAL 8)
    # Ninja + MSVC 下 CMAKE_SYSTEM_PROCESSOR 可能为空或写成 AMD64，用指针宽度判断架构。
    set(ARCH_TYPE "x86_64")
elseif(MSVC AND CMAKE_SIZEOF_VOID_P EQUAL 4)
    set(ARCH_TYPE "x86_32")
else()
    set(ARCH_TYPE "Unknown")
endif()

message("Architecture Type: ${ARCH_TYPE}")

# IPv4 代码（inet_pton/ntohl）在 MSVC 上需要显式链接 Ws2_32；Linux 由 libc 提供。
if(MSVC)
    set(CAITU_PLATFORM_LIBS ws2_32)
else()
    set(CAITU_PLATFORM_LIBS)
endif()

 
# 公共源码路径
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

if(BUILD_QT_GUI)
# Resolve the robot SDK for the collection GUI on Windows x64 or Linux x86_64/ARM64.
set(CGX_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libs/cr_sdk" CACHE PATH "CGXi robot SDK root")
if(WIN32)
    if(NOT MSVC OR NOT ARCH_TYPE STREQUAL "x86_64" OR NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
        message(FATAL_ERROR "Windows builds require MSVC x64")
    endif()
    set(CGX_DEFAULT_LIB_DIR "${CGX_SDK_DIR}/libs/windows/x64")
else()
    if(ARCH_TYPE STREQUAL "x86_64")
        if(EXISTS "${CGX_SDK_DIR}/libs/linux/x64/libcr_sdk.so")
            set(CGX_DEFAULT_LIB_DIR "${CGX_SDK_DIR}/libs/linux/x64")
        else()
            set(CGX_DEFAULT_LIB_DIR "${CGX_SDK_DIR}/libs/x64")
        endif()
    elseif(ARCH_TYPE STREQUAL "ARM64")
        if(EXISTS "${CGX_SDK_DIR}/libs/arm/aarch64/libcr_sdk.so")
            set(CGX_DEFAULT_LIB_DIR "${CGX_SDK_DIR}/libs/arm/aarch64")
        else()
            set(CGX_DEFAULT_LIB_DIR "${CGX_SDK_DIR}/libs")
        endif()
    endif()
endif()

set(CGX_SDK_LIB_DIR "${CGX_DEFAULT_LIB_DIR}" CACHE PATH "CGXi libraries for target architecture")
if(NOT EXISTS "${CGX_SDK_DIR}/include/robotapi.h")
    message(FATAL_ERROR "Robot SDK missing: set -DCGX_SDK_DIR=<SDK root containing include/robotapi.h>")
endif()
set(CGX_SDK_LIBS)
if(MSVC)
    set(sdk_library "${CGX_SDK_LIB_DIR}/cr_sdk.lib")
    if(NOT EXISTS "${sdk_library}")
        message(FATAL_ERROR "Missing ${ARCH_TYPE} robot import library: ${sdk_library}; set CGX_SDK_LIB_DIR")
    endif()
    list(APPEND CGX_SDK_LIBS "${sdk_library}")
elseif(UNIX)
    foreach(lib cr_sdk LogStorageDll RobotConfigDll z protobuf-c protobuf-rpc)
        set(sdk_library "${CGX_SDK_LIB_DIR}/lib${lib}.so")
        if(NOT EXISTS "${sdk_library}")
            message(FATAL_ERROR "Missing ${ARCH_TYPE} robot library: ${sdk_library}; set CGX_SDK_LIB_DIR")
        endif()
        list(APPEND CGX_SDK_LIBS "${sdk_library}")
    endforeach()
endif()
include_directories("${CGX_SDK_DIR}/include")
message(STATUS "Robot SDK: ${CGX_SDK_DIR} (lib: ${CGX_SDK_LIB_DIR})")

# MVS SDK：头文件和当前架构的导入库或运行库都必须存在，否则配置直接失败，
# 不允许生成名为 MVS、实际无相机支持的二进制。
if(WIN32)
    set(MVS_SDK_DIR "C:/Program Files (x86)/MVS" CACHE PATH "MVS SDK install prefix")
    set(MVS_INCLUDE_DIR "${MVS_SDK_DIR}/Development/Includes")
    set(MVS_LIB_DIR "${MVS_SDK_DIR}/Development/Libraries/win64")
else()
    set(MVS_SDK_DIR "/opt/MVS" CACHE PATH "MVS SDK install prefix")
    set(MVS_INCLUDE_DIR "${MVS_SDK_DIR}/include")
    if(ARCH_TYPE STREQUAL "x86_64")
        set(MVS_LIB_DIR "${MVS_SDK_DIR}/lib/64")
    elseif(ARCH_TYPE STREQUAL "ARM64")
        set(MVS_LIB_DIR "${MVS_SDK_DIR}/lib/aarch64")
    else()
        set(MVS_LIB_DIR "${MVS_SDK_DIR}/lib/32")
    endif()
endif()

if(NOT EXISTS "${MVS_INCLUDE_DIR}/MvCameraControl.h")
    message(FATAL_ERROR
        "CAMERA_BACKEND=MVS 但未找到 MVS SDK 头文件: "
        "${MVS_INCLUDE_DIR}/MvCameraControl.h")
endif()

if(MSVC)
    if(NOT EXISTS "${MVS_LIB_DIR}/MvCameraControl.lib")
        message(FATAL_ERROR
            "CAMERA_BACKEND=MVS 但未找到 ${ARCH_TYPE} 架构的 MVS 导入库: "
            "${MVS_LIB_DIR}/MvCameraControl.lib")
    endif()
elseif(UNIX)
    if(NOT EXISTS "${MVS_LIB_DIR}/libMvCameraControl.so")
        message(FATAL_ERROR
            "CAMERA_BACKEND=MVS 但未找到 ${ARCH_TYPE} 架构的 MVS 运行库: "
            "${MVS_LIB_DIR}/libMvCameraControl.so")
    endif()
endif()

include_directories(${MVS_INCLUDE_DIR})
link_directories(${MVS_LIB_DIR})
set(CAITU_CAMERA_INCLUDE_DIR "${MVS_INCLUDE_DIR}")
set(CAITU_CAMERA_LIB_DIR "${MVS_LIB_DIR}")
message(STATUS "MVS SDK: ${MVS_SDK_DIR} (lib: ${MVS_LIB_DIR})")

endif()

# Prepare collection output configuration; never overwrite existing settings.
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/param")
if(NOT EXISTS "${CMAKE_CURRENT_BINARY_DIR}/config.yaml")
    configure_file(config/collect.example.yaml "${CMAKE_CURRENT_BINARY_DIR}/config.yaml" COPYONLY)
endif()
set(CAITU_CAMERA_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/cameras/MVSCamera.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cameras/mvs_sdk_lifecycle.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cameras/mvs_device_selector.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cameras/mvs_error.cpp
)
if(MSVC)
    # MSVC 用 SDK 导入库的全路径；Linux 保留 -lMvCameraControl 加 link_directories 的行为。
    set(CAITU_CAMERA_LIBRARIES "${MVS_LIB_DIR}/MvCameraControl.lib")
else()
    set(CAITU_CAMERA_LIBRARIES MvCameraControl)
endif()

# Collection build identity, refreshed on each build.
set(COLLECTION_BUILD_INFO_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(COLLECTION_BUILD_INFO_HEADER "${COLLECTION_BUILD_INFO_DIR}/AppBuildInfo.hpp")
add_custom_target(collection_build_info ALL
    COMMAND ${CMAKE_COMMAND}
        -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
        -DOUTPUT_FILE=${COLLECTION_BUILD_INFO_HEADER}
        -DAPP_BUILD_TYPE=$<CONFIG>
        -DAPP_CAMERA_BACKEND=${CAMERA_BACKEND}
        -DAPP_VERSION=${CAITU_VERSION}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/hand_eye_gui/GenerateBuildInfo.cmake
    BYPRODUCTS ${COLLECTION_BUILD_INFO_HEADER}
    COMMENT "Refreshing collection build metadata"
    VERBATIM
)

option(BUILD_TESTS "Build no-hardware unit tests" ON)
if(BUILD_TESTS)
    add_executable(caitu_tests
        tests/run_tests.cpp
        tests/test_mvs_device_selector.cpp
        tests/test_mvs_error.cpp
        tests/test_config_resolver.cpp
        cameras/mvs_device_selector.cpp
        cameras/mvs_error.cpp
        hand_eye_gui/config_resolver.cpp
    )
    target_include_directories(caitu_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests
    )
    target_link_libraries(caitu_tests ${OpenCV_LIBS} ${CAITU_PLATFORM_LIBS})
    enable_testing()
    add_test(NAME caitu_tests COMMAND caitu_tests)
    add_executable(test_collection
        tests/test_collection.cpp
        utils/collection_utils.cpp)
    target_include_directories(test_collection PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(test_collection ${OpenCV_LIBS})
    add_test(NAME collection_synthetic_bgr_fusion_png COMMAND test_collection)
    find_package(Qt5 QUIET COMPONENTS Core Quick Test)
    if(Qt5Core_FOUND)
        add_executable(test_exposure_list tests/test_exposure_list.cpp)
        target_include_directories(test_exposure_list PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
        target_link_libraries(test_exposure_list Qt5::Core)
        add_test(NAME exposure_list COMMAND test_exposure_list)
        add_executable(test_diagnostic_logger
            tests/test_diagnostic_logger.cpp
            hand_eye_gui/DiagnosticLogger.cpp)
        target_include_directories(test_diagnostic_logger PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}/tests
        )
        # DiagnosticLogger.hpp 只在 GUI 编译宏下公开声明，测试单独编译该源文件时需要同样定义。
        target_compile_definitions(test_diagnostic_logger PRIVATE QT_GUI QT_MESSAGELOGCONTEXT)
        target_link_libraries(test_diagnostic_logger Qt5::Core)
        add_test(NAME diagnostic_logger COMMAND test_diagnostic_logger)
        add_executable(test_collection_undistortion
            tests/test_collection_undistortion.cpp
            utils/collection_undistortion.cpp
            hand_eye_gui/CollectionImageRecord.cpp
            hand_eye_gui/CollectionPoseRecord.cpp
            utils/calib_transform.cpp)
        target_include_directories(test_collection_undistortion PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
        target_link_libraries(test_collection_undistortion Qt5::Core ${OpenCV_LIBS})
        add_test(NAME collection_undistortion COMMAND test_collection_undistortion)
        add_executable(test_collection_pose_record
            tests/test_collection_pose_record.cpp
            hand_eye_gui/CollectionPoseRecord.cpp
            utils/calib_transform.cpp)
        target_include_directories(test_collection_pose_record PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
        target_link_libraries(test_collection_pose_record Qt5::Core ${OpenCV_LIBS})
        add_test(NAME collection_pose_records COMMAND test_collection_pose_record)
        add_executable(test_collection_capture
            tests/test_collection_capture.cpp
            hand_eye_gui/CollectionCapture.cpp
            hand_eye_gui/CollectionImageRecord.cpp
            hand_eye_gui/CollectionPoseRecord.cpp
            utils/collection_utils.cpp
            utils/collection_undistortion.cpp
            utils/calib_transform.cpp)
        target_include_directories(test_collection_capture PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
        target_link_libraries(test_collection_capture Qt5::Core ${OpenCV_LIBS})
        add_test(NAME collection_capture COMMAND test_collection_capture)
        if(BUILD_QT_GUI)
            add_executable(test_nine_point_capture tests/test_nine_point_capture.cpp
                hand_eye_gui/NinePointCapture.cpp utils/calib_transform.cpp)
            target_include_directories(test_nine_point_capture PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CGX_SDK_DIR}/include)
            # 桩实现与 robotapi.h 的 dllimport 声明冲突：测试编译时定义 CRSDK_EXPORT，
            # CRSDKAPI 变成 dllexport，桩定义才能与声明共存。测试不链接真实 SDK 导入库。
            if(MSVC)
                target_compile_definitions(test_nine_point_capture PRIVATE CRSDK_EXPORT)
            endif()
            target_link_libraries(test_nine_point_capture ${OpenCV_LIBS})
            add_test(NAME nine_point_capture COMMAND test_nine_point_capture)
            add_executable(test_robot_tcp tests/test_robot_tcp.cpp hand_eye_gui/RobotTcp.cpp)
            target_include_directories(test_robot_tcp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CGX_SDK_DIR}/include)
            if(MSVC)
                target_compile_definitions(test_robot_tcp PRIVATE CRSDK_EXPORT)
            endif()
            target_link_libraries(test_robot_tcp Qt5::Core)
            add_test(NAME robot_tcp COMMAND test_robot_tcp)
            add_executable(test_manual_tcp_motion tests/test_manual_tcp_motion.cpp
                hand_eye_gui/ManualTcpMotion.cpp hand_eye_gui/TeachMotion.cpp hand_eye_gui/RobotTcp.cpp
                hand_eye_gui/NinePointCapture.cpp utils/calib_transform.cpp)
            target_include_directories(test_manual_tcp_motion PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CGX_SDK_DIR}/include)
            if(MSVC)
                target_compile_definitions(test_manual_tcp_motion PRIVATE CRSDK_EXPORT)
            endif()
            target_link_libraries(test_manual_tcp_motion Qt5::Core ${OpenCV_LIBS})
            add_test(NAME manual_tcp_motion COMMAND test_manual_tcp_motion)
        endif()
    endif()
    if(Qt5Test_FOUND)
        add_executable(test_collection_ui tests/test_collection_ui.cpp hand_eye_gui/resources.qrc)
        set_target_properties(test_collection_ui PROPERTIES AUTORCC ON AUTOMOC ON)
        target_link_libraries(test_collection_ui Qt5::Core Qt5::Quick Qt5::Test)
        add_test(NAME collection_ui COMMAND test_collection_ui)
    endif()


endif()

# ============================================================
#  Qt5 QML collection GUI (optional)
# ============================================================
if(BUILD_QT_GUI)
    # 显式请求 GUI 时不允许静默降级：找不到 Qt5 直接配置失败。
    find_package(Qt5 REQUIRED COMPONENTS Core Quick QuickControls2 Widgets)
    add_subdirectory(hand_eye_gui)
    message(STATUS "Qt5 GUI enabled → target 'hand-eye-app'")
endif()
