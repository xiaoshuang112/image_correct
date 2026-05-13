cmake_minimum_required(VERSION 3.10)
project(caitu)

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_BUILD_TYPE "Release")

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

        set(CGX_SDK_DIR  "/work/soft/CGXi/aarch64")
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


SET(useORB TRUE)
SET(useOAK FALSE)


SET(SRC_LISTS 
    calibration.cpp exposure_fusion.cpp  
    hand_eye_calibration.cpp auto_eye_in_hand_charuco_calibration.cpp 
    putText.cpp Util.cpp Texts.cpp
)

SET(SRC_LIBS 
)

if(useORB)
    add_definitions(-DuseORB)
    list(APPEND SRC_LISTS
        OrbbecCamera.cpp
    )
    list(APPEND SRC_LIBS
        OrbbecSDK
    )
endif()

if(useOAK)
    add_definitions(-DuseOAK)
    list(APPEND SRC_LISTS
        OAKCamera.cpp    
    )
    list(APPEND SRC_LIBS
        depthai-core
    )
endif()


add_executable(${PROJECT_NAME} ${SRC_LISTS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${CGX_SDK_LIBS} ${SRC_LIBS} )