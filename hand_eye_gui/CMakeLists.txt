cmake_minimum_required(VERSION 3.10)

project(hand-eye-app)

# 查找 Qt5
find_package(Qt5 REQUIRED COMPONENTS Core Quick QuickControls2)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# 架构、相机后端及 SDK 路径由父项目统一解析和校验。
message(STATUS "[${PROJECT_NAME}] Architecture: ${ARCH_TYPE}; Camera backend: ${CAMERA_BACKEND}")

# ============================================================
#  GUI 源文件
# ============================================================
set(GUI_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/DiagnosticLogger.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/CalibrationController.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/CollectionActions.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/CollectionPoseRecord.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/CollectionImageRecord.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/CameraImageProvider.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/config_resolver.cpp
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

# 显式列出采集与标定界面使用的公共源码；采样器和深度测试仅由 CLI 构建。
set(GUI_CORE_SRCS
    ${CMAKE_SOURCE_DIR}/calibration/auto_eye_in_hand_charuco_calibration.cpp
    ${CMAKE_SOURCE_DIR}/calibration/calibration_session.cpp
    ${CMAKE_SOURCE_DIR}/calibration/calibration_utils.cpp
    ${CMAKE_SOURCE_DIR}/calibration/camera_calibration.cpp
    ${CMAKE_SOURCE_DIR}/calibration/candidate_planner.cpp
    ${CMAKE_SOURCE_DIR}/calibration/charuco_detector.cpp
    ${CMAKE_SOURCE_DIR}/calibration/eye_to_hand_calibration.cpp
    ${CMAKE_SOURCE_DIR}/calibration/intrinsic_identity.cpp
    ${CMAKE_SOURCE_DIR}/calibration/mechanical_settling_config.cpp
    ${CMAKE_SOURCE_DIR}/calibration/semi_auto_eye_in_hand_calibration.cpp
    ${CMAKE_SOURCE_DIR}/calibration/solver_consensus.cpp
    ${CMAKE_SOURCE_DIR}/utils/calib_transform.cpp
    ${CMAKE_SOURCE_DIR}/utils/collection_undistortion.cpp
    ${CMAKE_SOURCE_DIR}/utils/collection_utils.cpp
    ${CMAKE_SOURCE_DIR}/utils/robot_util.cpp
)

# ============================================================
#  可执行文件
# ============================================================
add_executable(${PROJECT_NAME}
    ${GUI_SRCS}
    ${GUI_HDRS}
    ${GUI_CORE_SRCS}
    ${CAITU_CAMERA_SOURCES}
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

target_include_directories(${PROJECT_NAME} PRIVATE ${CAITU_CAMERA_INCLUDE_DIR})
target_link_directories(${PROJECT_NAME} PRIVATE ${CAITU_CAMERA_LIB_DIR})

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
    ${CAITU_CAMERA_LIBRARIES}
)

# ============================================================
#  编译选项
# ============================================================
set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    OUTPUT_NAME "collect-${CAM_SUFFIX}"
)

# MVS 产物优先从二进制旁的 lib/mvs 加载运行库（发布包内捆绑），
# 开发机上回退到 SDK 安装目录；ORB 产物不引用任何 MVS 路径。
if(useMVS)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        INSTALL_RPATH "$ORIGIN/lib/mvs"
        BUILD_RPATH "$ORIGIN/lib/mvs;${MVS_LIB_DIR}")
endif()

# 构建身份必须在每次 build 时重新获取。若只在 configure 阶段执行，提交或
# 修改工作区后直接 cmake --build 会把旧 commit/dirty 状态编进新程序。
set(APP_BUILD_INFO_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(APP_BUILD_INFO_HEADER "${APP_BUILD_INFO_DIR}/AppBuildInfo.hpp")
add_custom_target(hand_eye_app_build_info ALL
    COMMAND ${CMAKE_COMMAND}
        -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
        -DOUTPUT_FILE=${APP_BUILD_INFO_HEADER}
        -DAPP_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        -DAPP_CAMERA_BACKEND=${CAMERA_BACKEND}
        -DAPP_VERSION=${CAITU_VERSION}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/GenerateBuildInfo.cmake
    BYPRODUCTS ${APP_BUILD_INFO_HEADER}
    COMMENT "Refreshing hand-eye build metadata"
    VERBATIM
)
add_dependencies(${PROJECT_NAME} hand_eye_app_build_info)
target_include_directories(${PROJECT_NAME} PRIVATE ${APP_BUILD_INFO_DIR})
target_compile_definitions(${PROJECT_NAME} PRIVATE
    ${CAITU_COMPILE_DEFINITIONS}
    ${ARCH_DEFINITION}
    DEFAULT_CAM_WIDTH=${CAITU_GUI_DEFAULT_CAM_WIDTH}
    DEFAULT_CAM_HEIGHT=${CAITU_GUI_DEFAULT_CAM_HEIGHT}
    QT_MESSAGELOGCONTEXT
    QT_GUI
)
