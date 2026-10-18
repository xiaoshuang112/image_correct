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
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
endif()

option(BUILD_QT_GUI "Build Qt QML collection GUI" ON)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d photo)

# 获取系统架构信息
message("System Processor: ${CMAKE_SYSTEM_PROCESSOR}")

# 判断架构
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64")
    set(ARCH_TYPE "x86_64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(ARCH_TYPE "ARM64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "armv7|armv7l")
    set(ARCH_TYPE "ARMv7")
else()
    set(ARCH_TYPE "Unknown")
endif()

message("Architecture Type: ${ARCH_TYPE}")

 
# 公共源码路径
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

if(BUILD_QT_GUI)
# Resolve the robot SDK for the collection GUI on Linux x86_64/ARM64.
set(CGX_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libs/cr_sdk" CACHE PATH "CGXi robot SDK root")
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
if(UNIX)
    set(CGX_SDK_LIB_DIR "${CGX_DEFAULT_LIB_DIR}" CACHE PATH "CGXi libraries for target architecture")
    if(NOT EXISTS "${CGX_SDK_DIR}/include/robotapi.h")
        message(FATAL_ERROR "Robot SDK missing: set -DCGX_SDK_DIR=<SDK root containing include/robotapi.h>")
    endif()
    set(CGX_SDK_LIBS)
    foreach(lib cr_sdk LogStorageDll RobotConfigDll z protobuf-c protobuf-rpc)
        set(sdk_library "${CGX_SDK_LIB_DIR}/lib${lib}.so")
        if(NOT EXISTS "${sdk_library}")
            message(FATAL_ERROR "Missing ${ARCH_TYPE} robot library: ${sdk_library}; set CGX_SDK_LIB_DIR")
        endif()
        list(APPEND CGX_SDK_LIBS "${sdk_library}")
    endforeach()
    include_directories("${CGX_SDK_DIR}/include")
    message(STATUS "Robot SDK: ${CGX_SDK_DIR} (lib: ${CGX_SDK_LIB_DIR})")
endif()

# MVS SDK：头文件和当前架构的运行库都必须存在，否则配置直接失败，
# 不允许生成名为 MVS、实际无相机支持的二进制。
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
set(CAITU_CAMERA_INCLUDE_DIR "${MVS_SDK_DIR}/include")
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
    ${CMAKE_CURRENT_SOURCE_DIR}/cameras/mvs_device_selector.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cameras/mvs_error.cpp
)
set(CAITU_CAMERA_LIBRARIES MvCameraControl)

# Collection build identity, refreshed on each build.
set(COLLECTION_BUILD_INFO_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(COLLECTION_BUILD_INFO_HEADER "${COLLECTION_BUILD_INFO_DIR}/AppBuildInfo.hpp")
add_custom_target(collection_build_info ALL
    COMMAND ${CMAKE_COMMAND}
        -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
        -DOUTPUT_FILE=${COLLECTION_BUILD_INFO_HEADER}
        -DAPP_BUILD_TYPE=${CMAKE_BUILD_TYPE}
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
    target_link_libraries(caitu_tests ${OpenCV_LIBS})
    enable_testing()
    add_test(NAME caitu_tests COMMAND caitu_tests)
    add_executable(test_collection
        tests/test_collection.cpp
        utils/collection_utils.cpp
        utils/calib_transform.cpp)
    target_include_directories(test_collection PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(test_collection ${OpenCV_LIBS})
    add_test(NAME collection_geometry_synthetic_bgr_fusion_png COMMAND test_collection)
    find_package(Qt5 QUIET COMPONENTS Core Quick Test)
    if(Qt5Core_FOUND)
        add_executable(test_exposure_list tests/test_exposure_list.cpp)
        target_include_directories(test_exposure_list PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
        target_link_libraries(test_exposure_list Qt5::Core)
        add_test(NAME exposure_list COMMAND test_exposure_list)
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
    find_package(Qt5 QUIET COMPONENTS Core Quick QuickControls2)
    if(Qt5_FOUND)
        add_subdirectory(hand_eye_gui)
        message(STATUS "Qt5 GUI enabled → target 'hand-eye-app'")
    else()
        message(WARNING "Qt5 not found, skipping GUI target")
    endif()
endif()
