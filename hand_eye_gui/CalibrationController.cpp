#include "CalibrationController.hpp"
#include "CameraImageProvider.hpp"
#include "auto_eye_in_hand_charuco_calibration.hpp"
#include "eye_to_hand_calibration.hpp"
#include "utils/robot_util.hpp"
#include "DiagnosticLogger.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QDebug>
#include <QThread>
#include <QPointer>
#include <QElapsedTimer>
#include <QStringList>

#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/filesystem.hpp>

#ifdef useMVS
#include "cameras/MVSCamera.hpp"
#endif
#ifdef useORB
#include "cameras/OrbbecCamera.hpp"
#endif

#include <fstream>
#include <sstream>
#include <filesystem>
#include <algorithm>

namespace fs = std::filesystem;

namespace {

QString formatDoubleVector(const std::vector<double>& values)
{
    QStringList parts;
    parts.reserve(static_cast<int>(values.size()));
    for (double value : values)
        parts.push_back(QString::number(value, 'g', 8));
    return QString("[%1]").arg(parts.join(','));
}

} // namespace

// ===================================================================
//  全局变量（原在 calibration.cpp 中定义，其他模块依赖）
// ===================================================================
CONFIG g_cfg;
std::string g_appPath = "";
std::string g_paramPath = "";
std::string g_posPath = "";
std::string g_calibrationPath = "";
std::shared_ptr<AICamera> g_camera = nullptr;
RobotHandle robotHandle;
AICameraIntrinsic g_cam_param;

// ---- 静态成员 ----
CalibrationController* CalibrationController::s_instance = nullptr;

// ===================================================================
//  构造 / 析构
// ===================================================================

CalibrationController::CalibrationController(QObject* parent)
    : QObject(parent)
{
    s_instance = this;

    m_previewTimer = new QTimer(this);
    m_previewTimer->setInterval(66);  // ~15 fps
    connect(m_previewTimer, &QTimer::timeout,
            this, &CalibrationController::onPreviewTick);
    APP_LOG_INFO("controller", QString("constructed object=%1 preview_interval_ms=%2")
        .arg(reinterpret_cast<quintptr>(this), 0, 16)
        .arg(m_previewTimer->interval()));
}

CalibrationController::~CalibrationController()
{
    APP_LOG_INFO("controller", QString("destructor_begin running=%1 preview=%2 camera=%3 robot=%4")
        .arg(m_running).arg(m_previewActive).arg(m_cameraOpened).arg(m_robotConnected));
    stopCalibration();
    waitForCalibrationThread();

    stopPreview();
    closeCamera();
    disconnectRobot();

    s_instance = nullptr;
    DiagnosticLogger::instance().setUiSink({});
    APP_LOG_INFO("controller", "destructor_complete");
}

// ===================================================================
//  初始化
// ===================================================================

void CalibrationController::init(CameraImageProvider* provider)
{
    m_imageProvider = provider;
    QPointer<CalibrationController> guard(this);
    DiagnosticLogger::instance().setUiSink([guard](const QString& line) {
        if (guard)
            guard->appendLog(line);
    });
    APP_LOG_INFO("controller", QString("init image_provider=%1")
        .arg(reinterpret_cast<quintptr>(provider), 0, 16));
    setupPaths();
    loadConfigYaml();
    setStatus("就绪 — 请连接机器人");
}

void CalibrationController::setupPaths()
{
    // 获取可执行文件所在目录
    QString exePath = QCoreApplication::applicationDirPath();
    m_appPath = exePath.toStdString() + "/";

    m_paramPath      = m_appPath + "param/";
    m_calibrationPath = m_paramPath + "calibration/";

    try {
        fs::create_directories(m_calibrationPath);
    } catch (const std::exception& e) {
        APP_LOG_ERROR("filesystem", QString("create_directories_failed path=%1 error=%2")
            .arg(QString::fromStdString(m_calibrationPath), QString::fromUtf8(e.what())));
        throw;
    }

    APP_LOG_INFO("filesystem", QString("paths_ready app=%1 param=%2 calibration=%3 log=%4")
        .arg(QString::fromStdString(m_appPath),
             QString::fromStdString(m_paramPath),
             QString::fromStdString(m_calibrationPath),
             DiagnosticLogger::instance().logFilePath()));
}

void CalibrationController::loadConfigYaml()
{
    std::string cfg_file = m_appPath + "config.yaml";
    if (!QFileInfo::exists(QString::fromStdString(cfg_file))) {
        APP_LOG_WARNING("config", QString("config_missing path=%1 using_defaults=true")
            .arg(QString::fromStdString(cfg_file)));
        return;
    }
    cv::FileStorage fs(cfg_file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        APP_LOG_ERROR("config", QString("config_open_failed path=%1 using_defaults=true")
            .arg(QString::fromStdString(cfg_file)));
        return;
    }
    // 读取相机配置
    std::string cam_ip;
    int cam_port = 0, cam_w = 0, cam_h = 0;
    fs["cam_ip"] >> cam_ip;
    fs["cam_port"] >> cam_port;
    fs["cam_width"] >> cam_w;
    fs["cam_height"] >> cam_h;
    fs.release();

    if (!cam_ip.empty())  m_cameraIp  = QString::fromStdString(cam_ip);
    if (cam_port > 0)     m_cameraPort = cam_port;
    if (cam_w > 0)        m_previewWidth  = cam_w;
    if (cam_h > 0)        m_previewHeight = cam_h;

    APP_LOG_INFO("config", QString("config_loaded path=%1 camera=%2:%3 resolution=%4x%5")
        .arg(QString::fromStdString(cfg_file), m_cameraIp)
        .arg(m_cameraPort).arg(m_previewWidth).arg(m_previewHeight));
}

// ===================================================================
//  属性 setter
// ===================================================================

void CalibrationController::setRobotIp(const QString& v) {
    if (m_robotIp != v) {
        APP_LOG_INFO("config", QString("robot_ip_changed from=%1 to=%2").arg(m_robotIp, v));
        m_robotIp = v; emit robotIpChanged();
    }
}
void CalibrationController::setRobotPort(int v) {
    if (m_robotPort != v) {
        APP_LOG_INFO("config", QString("robot_port_changed from=%1 to=%2").arg(m_robotPort).arg(v));
        m_robotPort = v; emit robotPortChanged();
    }
}
void CalibrationController::setRobotPasswd(const QString& v) {
    if (m_robotPasswd != v) {
        APP_LOG_INFO("config", "robot_password_changed value_redacted=true");
        m_robotPasswd = v; emit robotPasswdChanged();
    }
}
void CalibrationController::setCameraToTcpEnd(int v) {
    if (m_cameraToTcpEnd != v) {
        APP_LOG_INFO("config", QString("camera_to_tcp_end_changed from=%1 to=%2")
            .arg(m_cameraToTcpEnd).arg(v));
        m_cameraToTcpEnd = v; emit cameraToTcpEndChanged();
    }
}
void CalibrationController::setCalibrationType(const QString& v) {
    if (m_calibrationType != v) {
        APP_LOG_INFO("config", QString("calibration_type_changed from=%1 to=%2")
            .arg(m_calibrationType, v));
        m_calibrationType = v; emit calibrationTypeChanged();
    }
}
void CalibrationController::setCameraIp(const QString& v) {
    if (m_cameraIp != v) {
        APP_LOG_INFO("config", QString("camera_ip_changed from=%1 to=%2").arg(m_cameraIp, v));
        m_cameraIp = v; emit cameraIpChanged();
    }
}
void CalibrationController::setCameraPort(int v) {
    if (m_cameraPort != v) {
        APP_LOG_INFO("config", QString("camera_port_changed from=%1 to=%2").arg(m_cameraPort).arg(v));
        m_cameraPort = v; emit cameraPortChanged();
    }
}
void CalibrationController::setPreviewWidth(int v) {
    if (m_previewWidth != v) {
        APP_LOG_INFO("config", QString("preview_width_changed from=%1 to=%2")
            .arg(m_previewWidth).arg(v));
        m_previewWidth = v; emit previewWidthChanged();
    }
}
void CalibrationController::setPreviewHeight(int v) {
    if (m_previewHeight != v) {
        APP_LOG_INFO("config", QString("preview_height_changed from=%1 to=%2")
            .arg(m_previewHeight).arg(v));
        m_previewHeight = v; emit previewHeightChanged();
    }
}

QString CalibrationController::diagnosticLogPath() const
{
    return DiagnosticLogger::instance().logFilePath();
}

// ===================================================================
//  机器人连接
// ===================================================================

void CalibrationController::connectRobot()
{
    if (m_robotConnected) {
        APP_LOG_INFO("robot", "connect_skipped reason=already_connected");
        setStatus("机器人已连接");
        return;
    }

    QElapsedTimer timer;
    timer.start();
    APP_LOG_INFO("robot", QString("connect_begin endpoint=%1:%2")
        .arg(m_robotIp).arg(m_robotPort));
    setStatus("正在连接机器人...");

    QByteArray ip   = m_robotIp.toLocal8Bit();
    QByteArray pwd  = m_robotPasswd.toLocal8Bit();

    CRresult ret = cr_create_robot(&m_robotHandle, ip.data(),
                                    m_robotPort, pwd.data());
    if (ret != sucess) {
        APP_LOG_ERROR("robot", QString("connect_failed endpoint=%1:%2 error=%3 elapsed_ms=%4")
            .arg(m_robotIp).arg(m_robotPort).arg(ret).arg(timer.elapsed()));
        setStatus(QString("连接机器人失败 (err=%1)").arg(ret));
        emit robotConnectedChanged();
        return;
    }

    m_robotConnected = true;
    emit robotConnectedChanged();

    // 上电使能
    const bool ready = checkAndRestoreRobotHand();
    APP_LOG_INFO("robot", QString("connect_complete handle=%1 ready=%2 elapsed_ms=%3")
        .arg(static_cast<qulonglong>(m_robotHandle)).arg(ready).arg(timer.elapsed()));
    setStatus("机器人已连接");
}

void CalibrationController::disconnectRobot()
{
    APP_LOG_INFO("robot", QString("disconnect_begin connected=%1 handle=%2")
        .arg(m_robotConnected).arg(static_cast<qulonglong>(m_robotHandle)));
    stopPreview();
    stopCalibration();
    waitForCalibrationThread();

    if (m_robotConnected) {
        if (m_robotHandle != 0) {
            const CRresult result = cr_destroy_robot(m_robotHandle);
            if (result != sucess)
                APP_LOG_WARNING("robot", QString("destroy_failed error=%1").arg(result));
        }
        m_robotHandle = 0;
        m_robotConnected = false;
        emit robotConnectedChanged();
    }
    setStatus("机器人已断开");
    APP_LOG_INFO("robot", "disconnect_complete");
}

bool CalibrationController::checkAndRestoreRobotHand()
{
    if (!m_robotConnected || m_robotHandle == 0) {
        APP_LOG_WARNING("robot", QString("ensure_ready_rejected connected=%1 handle=%2")
            .arg(m_robotConnected).arg(static_cast<qulonglong>(m_robotHandle)));
        return false;
    }

    enum RobotModes robotMode;
    CRresult res = cr_get_robotMode(m_robotHandle, &robotMode);
    if (res != sucess) {
        APP_LOG_ERROR("robot", QString("get_mode_failed error=%1").arg(res));
        setStatus("获取机械臂模式失败");
        return false;
    }
    APP_LOG_INFO("robot", QString("mode_received mode=%1").arg(static_cast<int>(robotMode)));
    if (robotMode != JointPowerOff) {
        APP_LOG_INFO("robot", "ensure_ready_complete action=none");
        return true;  // 已上电
    }

    setStatus("机械臂未上电，正在上电使能...");

    res = cr_FaultReset(m_robotHandle);
    if (res == sucess) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    } else {
        APP_LOG_ERROR("robot", QString("fault_reset_failed error=%1").arg(res));
        setStatus("故障复位失败");
        return false;
    }

    res = cr_poweron(m_robotHandle);
    if (res != sucess) {
        APP_LOG_ERROR("robot", QString("power_on_failed error=%1").arg(res));
        setStatus("上电失败");
        return false;
    }

    // 等待 JointIdle
    while (cr_get_robotMode(m_robotHandle, &robotMode) == sucess) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (robotMode == JointIdle) break;
    }

    res = cr_enable(m_robotHandle);
    if (res != sucess) {
        APP_LOG_ERROR("robot", QString("enable_failed error=%1").arg(res));
        setStatus("使能失败");
        return false;
    }

    // 等待 ProgramStop
    while (cr_get_robotMode(m_robotHandle, &robotMode) == sucess) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (robotMode == ProgramStop) break;
    }

    setStatus("机械臂已上电使能");
    APP_LOG_INFO("robot", QString("ensure_ready_complete mode=%1")
        .arg(static_cast<int>(robotMode)));
    return true;
}

// ===================================================================
//  相机操作
// ===================================================================

void CalibrationController::openCamera()
{
    if (m_cameraOpened) {
        APP_LOG_INFO("camera", "open_skipped reason=already_open");
        return;
    }

    QElapsedTimer timer;
    timer.start();
    APP_LOG_INFO("camera", QString("open_begin endpoint=%1:%2 requested=%3x%4 backend=%5")
        .arg(m_cameraIp).arg(m_cameraPort).arg(m_previewWidth).arg(m_previewHeight)
#ifdef useORB
        .arg("orbbec")
#else
        .arg("mvs")
#endif
    );
    try {

    // 关闭旧相机（如果存在且配置可能已变更）
    if (m_camera) {
        m_camera->close();
        m_camera.reset();
    }

    // 根据当前 UI 配置创建相机
    CONFIG cfg;
    cfg.cam_ip   = m_cameraIp.toStdString();
    cfg.cam_port = static_cast<uint16_t>(m_cameraPort);
    cfg.cam_width  = m_previewWidth;
    cfg.cam_height = m_previewHeight;

    APP_LOG_INFO("camera", QString("creating_instance endpoint=%1:%2 resolution=%3x%4")
        .arg(QString::fromStdString(cfg.cam_ip)).arg(cfg.cam_port)
        .arg(cfg.cam_width).arg(cfg.cam_height));

#ifdef useORB
    m_camera = std::make_shared<OrbbecCameraV2>(cfg);
#else
    m_camera = std::make_shared<MVSCamera>(cfg);
#endif

    setStatus("正在打开相机...");
    if (!m_camera->open()) {
        APP_LOG_ERROR("camera", QString("open_failed elapsed_ms=%1").arg(timer.elapsed()));
        setStatus("打开相机失败");
        emit cameraOpenedChanged();
        return;
    }

    m_cameraOpened = true;
    emit cameraOpenedChanged();

    // 读取并保存相机内参
    auto intrinsics = m_camera->getCameraIntrinsics(m_previewWidth, m_previewHeight);
    const bool validIntrinsics =
        intrinsics.size() >= 4 &&
        intrinsics[0].size() >= 3 &&
        intrinsics[1].size() >= 3;
    if (validIntrinsics) {
        cv::FileStorage out(m_paramPath + "camera_params.yaml",
                            cv::FileStorage::WRITE);
        cv::Mat camera_matrix = cv::Mat::zeros(3, 3, CV_64F);
        cv::Mat dist_coeffs   = cv::Mat::zeros(5, 1, CV_64F);

        camera_matrix.at<double>(0, 0) = intrinsics[0][0];
        camera_matrix.at<double>(1, 1) = intrinsics[1][1];
        camera_matrix.at<double>(2, 2) = 1.0;
        camera_matrix.at<double>(0, 2) = intrinsics[0][2];
        camera_matrix.at<double>(1, 2) = intrinsics[1][2];

        const size_t distortionCount = std::min<size_t>(5, intrinsics[3].size());
        for (size_t i = 0; i < distortionCount; i++)
            dist_coeffs.at<double>(static_cast<int>(i), 0) = intrinsics[3][i];

        out << "camera_matrix" << camera_matrix;
        out << "distortion_coefficients" << dist_coeffs;
        out.release();
        APP_LOG_INFO("camera", QString("intrinsics_saved path=%1 fx=%2 fy=%3 cx=%4 cy=%5 distortion_count=%6")
            .arg(QString::fromStdString(m_paramPath + "camera_params.yaml"))
            .arg(camera_matrix.at<double>(0, 0), 0, 'f', 4)
            .arg(camera_matrix.at<double>(1, 1), 0, 'f', 4)
            .arg(camera_matrix.at<double>(0, 2), 0, 'f', 4)
            .arg(camera_matrix.at<double>(1, 2), 0, 'f', 4)
            .arg(distortionCount));
    } else {
        APP_LOG_WARNING("camera", QString("invalid_intrinsics outer_size=%1")
            .arg(intrinsics.size()));
    }

    setStatus("相机已打开");
    APP_LOG_INFO("camera", QString("open_complete elapsed_ms=%1").arg(timer.elapsed()));
    } catch (const std::exception& e) {
        APP_LOG_ERROR("camera", QString("open_exception type=std message=%1 elapsed_ms=%2")
            .arg(QString::fromUtf8(e.what())).arg(timer.elapsed()));
        if (m_camera)
            m_camera->close();
        m_cameraOpened = false;
        emit cameraOpenedChanged();
        setStatus(QString("打开相机异常: %1").arg(QString::fromUtf8(e.what())));
    } catch (...) {
        APP_LOG_ERROR("camera", QString("open_exception type=unknown elapsed_ms=%1")
            .arg(timer.elapsed()));
        if (m_camera)
            m_camera->close();
        m_cameraOpened = false;
        emit cameraOpenedChanged();
        setStatus("打开相机发生未知异常");
    }
}

void CalibrationController::closeCamera()
{
    APP_LOG_INFO("camera", QString("close_begin opened=%1 instance=%2")
        .arg(m_cameraOpened)
        .arg(reinterpret_cast<quintptr>(m_camera.get()), 0, 16));
    stopCalibration();
    waitForCalibrationThread();
    stopPreview();

    if (m_camera) {
        m_camera->close();
    }
    m_cameraOpened = false;
    emit cameraOpenedChanged();
    setStatus("相机关闭");
    APP_LOG_INFO("camera", "close_complete");
}

// ===================================================================
//  预览
// ===================================================================

void CalibrationController::startPreview()
{
    if (m_previewActive) {
        APP_LOG_INFO("preview", "start_skipped reason=already_active");
        return;
    }
    if (!m_camera || !m_cameraOpened) {
        APP_LOG_WARNING("preview", QString("start_rejected camera_instance=%1 camera_opened=%2")
            .arg(static_cast<bool>(m_camera)).arg(m_cameraOpened));
        setStatus("请先打开相机");
        return;
    }

    try {
        APP_LOG_INFO("preview", QString("start_begin calibration_type=%1 interval_ms=%2")
            .arg(m_calibrationType).arg(m_previewTimer->interval()));
        // 创建预览用的 ChArUco 检测器
        CharucoBoardConfig board_cfg;  // 默认: 7x5, 40mm, 28mm

    if (m_calibrationType == "eye_in_hand") {
        // 眼在手内：加载已知内参做预览
        std::string cam_file = m_paramPath + "camera_params.yaml";
        m_previewDetector = std::make_unique<CharucoDetector>(board_cfg, cam_file);
    } else {
        // 眼在手外：不加载内参（内参将从标定中获得）
        m_previewDetector = std::make_unique<CharucoDetector>(board_cfg);
    }

        m_previewTimer->start();
        m_previewFrameCount = 0;
        m_previewEmptyCount = 0;
        m_haveBoardDetectionState = false;
        m_previewActive = true;
        emit previewActiveChanged();
        setStatus("预览中 — 调整位姿后点击开始标定");
        APP_LOG_INFO("preview", "start_complete");
    } catch (const std::exception& e) {
        APP_LOG_ERROR("preview", QString("start_exception type=std message=%1")
            .arg(QString::fromUtf8(e.what())));
        m_previewDetector.reset();
        setStatus(QString("启动预览异常: %1").arg(QString::fromUtf8(e.what())));
    } catch (...) {
        APP_LOG_ERROR("preview", "start_exception type=unknown");
        m_previewDetector.reset();
        setStatus("启动预览发生未知异常");
    }
}

void CalibrationController::stopPreview()
{
    const bool wasActive = m_previewActive;
    m_previewTimer->stop();
    m_previewDetector.reset();
    m_previewActive = false;
    emit previewActiveChanged();
    if (wasActive) {
        APP_LOG_INFO("preview", QString("stop_complete frames=%1 empty_frames=%2")
            .arg(m_previewFrameCount).arg(m_previewEmptyCount));
    }
}

void CalibrationController::onPreviewTick()
{
    if (!m_camera || !m_cameraOpened || m_running) return;

    try {

    cv::Mat frame = m_camera->captureOneFrame(10);
    if (frame.empty()) {
        ++m_previewEmptyCount;
        if (m_previewEmptyCount == 1 || m_previewEmptyCount % 150 == 0) {
            APP_LOG_WARNING("preview", QString("empty_frame count=%1 successful_frames=%2")
                .arg(m_previewEmptyCount).arg(m_previewFrameCount));
        }
        return;
    }
    ++m_previewFrameCount;

    // ChArUco 检测叠加
    cv::Mat display;
    bool boardDetected = false;
    if (m_previewDetector) {
        cv::Mat rvec, tvec;
        if (m_previewDetector->detectAndDraw(frame, display, rvec, tvec)) {
            boardDetected = true;
            cv::putText(display, "ChArUco OK",
                        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                        0.7, cv::Scalar(0, 255, 0), 2);
        } else {
            frame.copyTo(display);
            cv::putText(display, "No board",
                        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                        0.7, cv::Scalar(0, 0, 255), 2);
        }
    } else {
        display = frame;
    }

    if (!m_haveBoardDetectionState || boardDetected != m_lastBoardDetected) {
        APP_LOG_INFO("preview", QString("board_detection_changed detected=%1 frame=%2")
            .arg(boardDetected).arg(m_previewFrameCount));
        m_haveBoardDetectionState = true;
        m_lastBoardDetected = boardDetected;
    }
    if (m_previewFrameCount == 1 || m_previewFrameCount % 150 == 0) {
        APP_LOG_DEBUG("preview", QString("heartbeat frames=%1 empty_frames=%2 size=%3x%4 channels=%5 board=%6")
            .arg(m_previewFrameCount).arg(m_previewEmptyCount)
            .arg(frame.cols).arg(frame.rows).arg(frame.channels()).arg(boardDetected));
    }

    publishFrame(display);
    emit frameUpdated();
    } catch (const std::exception& e) {
        APP_LOG_ERROR("preview", QString("tick_exception type=std message=%1 frame=%2")
            .arg(QString::fromUtf8(e.what())).arg(m_previewFrameCount));
        stopPreview();
        setStatus(QString("预览异常: %1").arg(QString::fromUtf8(e.what())));
    } catch (...) {
        APP_LOG_ERROR("preview", QString("tick_exception type=unknown frame=%1")
            .arg(m_previewFrameCount));
        stopPreview();
        setStatus("预览发生未知异常");
    }
}

// ===================================================================
//  cv::Mat → QImage 转换
// ===================================================================

void CalibrationController::publishFrame(const cv::Mat& bgr)
{
    if (bgr.empty() || !m_imageProvider) {
        APP_LOG_WARNING("image", QString("publish_skipped empty=%1 provider=%2")
            .arg(bgr.empty())
            .arg(reinterpret_cast<quintptr>(m_imageProvider), 0, 16));
        return;
    }

    QImage qimg;
    cv::Mat converted;
    if (bgr.channels() == 3) {
        cv::cvtColor(bgr, converted, cv::COLOR_BGR2RGB);
        qimg = QImage(converted.data, converted.cols, converted.rows,
                      static_cast<int>(converted.step),
                      QImage::Format_RGB888);
    } else if (bgr.channels() == 1) {
        qimg = QImage(bgr.data, bgr.cols, bgr.rows,
                      static_cast<int>(bgr.step),
                      QImage::Format_Grayscale8);
    } else if (bgr.channels() == 4) {
        cv::cvtColor(bgr, converted, cv::COLOR_BGRA2RGBA);
        qimg = QImage(converted.data, converted.cols, converted.rows,
                      static_cast<int>(converted.step),
                      QImage::Format_RGBA8888);
    } else {
        APP_LOG_WARNING("image", QString("publish_unsupported_channels channels=%1 size=%2x%3")
            .arg(bgr.channels()).arg(bgr.cols).arg(bgr.rows));
        return;
    }

    m_imageProvider->setImage(qimg.copy());
}

// ===================================================================
//  标定线程
// ===================================================================

void CalibrationController::startCalibration()
{
    if (m_running) {
        APP_LOG_WARNING("calibration", "start_rejected reason=already_running");
        setStatus("标定已在运行中");
        return;
    }
    if (!m_robotConnected) {
        APP_LOG_WARNING("calibration", "start_rejected reason=robot_disconnected");
        setStatus("请先连接机器人");
        return;
    }
    if (!m_cameraOpened) {
        APP_LOG_WARNING("calibration", "start_rejected reason=camera_closed");
        setStatus("请先打开相机");
        return;
    }

    APP_LOG_INFO("calibration", QString("start_requested type=%1 robot=%2:%3 camera=%4:%5 "
                                         "resolution=%6x%7 camera_to_tcp_end=%8")
        .arg(m_calibrationType, m_robotIp).arg(m_robotPort)
        .arg(m_cameraIp).arg(m_cameraPort)
        .arg(m_previewWidth).arg(m_previewHeight).arg(m_cameraToTcpEnd));

    // 标定期间暂停预览（标定线程会独占相机）
    stopPreview();

    m_running = true;
    m_cancelRequested = false;
    m_lastLoggedProgress = -1;
    m_progress = 0;
    setResult("");
    emit runningChanged();
    emit progressChanged();

    waitForCalibrationThread();

    m_calibThread = std::thread(
        &CalibrationController::calibrationThreadFunc,
        this,
        m_calibrationType,
        m_robotIp,
        m_robotPort,
        m_robotPasswd,
        m_cameraToTcpEnd,
        m_camera);
    APP_LOG_INFO("calibration", "worker_thread_created");
}

void CalibrationController::stopCalibration()
{
    if (!m_running) {
        APP_LOG_DEBUG("calibration", "cancel_ignored reason=not_running");
        return;
    }
    APP_LOG_WARNING("calibration", "cancel_requested");
    m_cancelRequested = true;
    setStatus("正在取消标定...");
}

void CalibrationController::waitForCalibrationThread()
{
    if (m_calibThread.joinable() &&
        m_calibThread.get_id() != std::this_thread::get_id()) {
        APP_LOG_DEBUG("calibration", "waiting_for_worker_thread");
        m_calibThread.join();
        APP_LOG_DEBUG("calibration", "worker_thread_joined");
    }
}

void CalibrationController::calibrationThreadFunc(
    QString calibrationType,
    QString robotIp,
    int robotPort,
    QString robotPassword,
    int cameraToTcpEnd,
    std::shared_ptr<AICamera> camera)
{
    QElapsedTimer timer;
    timer.start();
    APP_LOG_INFO("calibration", QString("worker_started type=%1 robot=%2:%3 camera_instance=%4 "
                                         "camera_to_tcp_end=%5 output=%6")
        .arg(calibrationType, robotIp).arg(robotPort)
        .arg(reinterpret_cast<quintptr>(camera.get()), 0, 16)
        .arg(cameraToTcpEnd)
        .arg(QString::fromStdString(m_calibrationPath)));
    try {
    // ================================================================
    //  眼在手内 (eye_in_hand)
    // ================================================================
    if (calibrationType == "eye_in_hand") {

        setStatus("眼在手内标定 — 准备中...");

        // 1. 加载已有相机内参
        cv::Mat camera_matrix, dist_coeffs;
        std::string cam_file = m_paramPath + "camera_params.yaml";
        cv::FileStorage fs(cam_file, cv::FileStorage::READ);
        if (fs.isOpened()) {
            fs["camera_matrix"] >> camera_matrix;
            fs["distortion_coefficients"] >> dist_coeffs;
            fs.release();
        }
        APP_LOG_INFO("calibration", QString("eye_in_hand_intrinsics path=%1 loaded=%2 matrix=%3x%4 dist=%5")
            .arg(QString::fromStdString(cam_file))
            .arg(!camera_matrix.empty())
            .arg(camera_matrix.cols).arg(camera_matrix.rows)
            .arg(dist_coeffs.total()));

        // 2. 确保机器人上电
        if (!checkAndRestoreRobotHand()) {
            APP_LOG_ERROR("calibration", "robot_prepare_failed");
            setStatus("机器人上电使能失败");
            goto done;
        }

        // 3. 构建标定参数
        CharucoBoardConfig board_cfg;

        // 4. 调用核心标定
        {
            setStatus("眼在手内标定 — 采集中...");
            CalibrationResult result;

            QByteArray ip = robotIp.toLocal8Bit();
            const std::string password = robotPassword.toStdString();
            bool ok = autoEyeInHandCharucoCalibration(
                m_robotHandle,
                ip.data(),
                camera,
                board_cfg,
                camera_matrix,
                dist_coeffs,
                cameraToTcpEnd,
                result,
                m_calibrationPath,
                m_paramPath + "hand_eye_result.yaml",
                calibrationCallback,
                robotPort,
                password);

            APP_LOG_INFO("calibration", QString("eye_in_hand_core_returned success=%1 cancelled=%2 "
                                                 "valid_frames=%3 error_mm=%4 message=%5")
                .arg(ok).arg(m_cancelRequested.load()).arg(result.valid_frames)
                .arg(result.reproj_error * 1000.0, 0, 'f', 6)
                .arg(QString::fromStdString(result.message)));

            if (m_cancelRequested) {
                APP_LOG_WARNING("calibration", "eye_in_hand_cancelled");
                setStatus("标定已取消");
                goto done;
            }

            if (ok) {
                // 保存相机内参（如果标定过程中更新了）
                if (!camera_matrix.empty()) {
                    const std::string cameraOutput = m_appPath + "camera_params.yaml";
                    cv::FileStorage outCam(cameraOutput,
                                           cv::FileStorage::WRITE);
                    outCam << "camera_matrix" << result.camera_matrix;
                    outCam << "distortion_coefficients" << result.dist_coeffs;
                    outCam.release();
                    APP_LOG_INFO("calibration", QString("eye_in_hand_intrinsics_saved path=%1")
                        .arg(QString::fromStdString(cameraOutput)));
                }

                std::stringstream ss_in;
                ss_in << result.T_flange_camera;
                QString resText = QString(
                    "T_flange_camera:\n%1\n"
                    "重投影误差: %2 mm\n"
                    "有效帧数: %3")
                    .arg(QString::fromStdString(ss_in.str()))
                    .arg(result.reproj_error * 1000.0, 0, 'f', 4)
                    .arg(result.valid_frames);

                setResult(resText);
                setStatus("眼在手内标定成功!");
                APP_LOG_INFO("calibration", QString("eye_in_hand_success result=%1")
                    .arg(QString::fromStdString(m_paramPath + "hand_eye_result.yaml")));
                emit calibrationFinished(true, "标定成功");
            } else {
                APP_LOG_ERROR("calibration", QString("eye_in_hand_failed message=%1")
                    .arg(QString::fromStdString(result.message)));
                setStatus(QString("标定失败: %1").arg(
                              QString::fromStdString(result.message)));
                emit calibrationFinished(false,
                    QString::fromStdString(result.message));
            }
        }

    // ================================================================
    //  眼在手外 (eye_to_hand)
    // ================================================================
    } else {

        setStatus("眼在手外标定 — 准备中...");

        CharucoBoardConfig board_cfg;
        CharucoDetector detector(board_cfg);  // 不加载内参

        // 加载配置
        PoseGenerationConfig pose_config;
        std::string cfg_path = m_paramPath + "eye_to_hand_config.yaml";
        cv::FileStorage cfg_fs(cfg_path, cv::FileStorage::READ);
        if (cfg_fs.isOpened()) {
            cv::FileNode fn;
            fn = cfg_fs["offset_x"]; if (!fn.empty()) fn >> pose_config.offset_x;
            fn = cfg_fs["offset_y"]; if (!fn.empty()) fn >> pose_config.offset_y;
            fn = cfg_fs["offset_z"]; if (!fn.empty()) fn >> pose_config.offset_z;
            fn = cfg_fs["offset_rx"]; if (!fn.empty()) fn >> pose_config.offset_rx;
            fn = cfg_fs["offset_ry"]; if (!fn.empty()) fn >> pose_config.offset_ry;
            fn = cfg_fs["offset_rz"]; if (!fn.empty()) fn >> pose_config.offset_rz;
            cfg_fs["min_corners"] >> pose_config.min_corners;
            cfg_fs["min_valid_frames"] >> pose_config.min_valid_frames;
            cfg_fs.release();
            APP_LOG_INFO("calibration", QString("eye_to_hand_config_loaded path=%1")
                .arg(QString::fromStdString(cfg_path)));
        } else {
            APP_LOG_WARNING("calibration", QString("eye_to_hand_config_missing path=%1 using_defaults=true")
                .arg(QString::fromStdString(cfg_path)));
        }

        APP_LOG_INFO("calibration", QString("eye_to_hand_config offsets={x:%1,y:%2,z:%3,rx:%4,ry:%5,rz:%6} "
                                             "min_corners=%7 min_valid_frames=%8")
            .arg(formatDoubleVector(pose_config.offset_x))
            .arg(formatDoubleVector(pose_config.offset_y))
            .arg(formatDoubleVector(pose_config.offset_z))
            .arg(formatDoubleVector(pose_config.offset_rx))
            .arg(formatDoubleVector(pose_config.offset_ry))
            .arg(formatDoubleVector(pose_config.offset_rz))
            .arg(pose_config.min_corners).arg(pose_config.min_valid_frames));

        std::string save_dir = m_calibrationPath + "eye_to_hand/";

        {
            setStatus("眼在手外标定 — 采集中...");
            EyeToHandResult result;

            QByteArray ip = robotIp.toLocal8Bit();
            bool ok = autoEyeToHandCharucoCalibration(
                m_robotHandle,
                ip.data(),
                camera,
                detector,
                pose_config,
                result,
                save_dir,
                calibrationCallback);

            APP_LOG_INFO("calibration", QString("eye_to_hand_core_returned success=%1 cancelled=%2 "
                                                 "valid_frames=%3 error_mm=%4 message=%5")
                .arg(ok).arg(m_cancelRequested.load()).arg(result.valid_frames)
                .arg(result.reproj_error * 1000.0, 0, 'f', 6)
                .arg(QString::fromStdString(result.message)));

            if (m_cancelRequested) {
                APP_LOG_WARNING("calibration", "eye_to_hand_cancelled");
                setStatus("标定已取消");
                goto done;
            }

            if (ok) {
                std::string out_file = m_paramPath + "eye_to_hand_result.yaml";
                cv::FileStorage out(out_file, cv::FileStorage::WRITE);
                out << "T_base_camera" << result.T_base_camera;
                out << "camera_matrix" << result.camera_matrix;
                out << "distortion_coefficients" << result.dist_coeffs;
                out << "reprojection_error_mm" << result.reproj_error * 1000.0;
                out << "valid_frames" << result.valid_frames;
                out.release();
                APP_LOG_INFO("calibration", QString("eye_to_hand_result_saved path=%1")
                    .arg(QString::fromStdString(out_file)));

                // 保存相机内参（如果标定过程中更新了）
                if (!result.camera_matrix.empty()) {
                    const std::string cameraOutput = m_appPath + "camera_params.yaml";
                    cv::FileStorage outCam(cameraOutput,
                                           cv::FileStorage::WRITE);
                    outCam << "camera_matrix" << result.camera_matrix;
                    outCam << "distortion_coefficients" << result.dist_coeffs;
                    outCam.release();
                    APP_LOG_INFO("calibration", QString("eye_to_hand_intrinsics_saved path=%1")
                        .arg(QString::fromStdString(cameraOutput)));
                }


                std::stringstream ss_et;
                ss_et << result.T_base_camera;
                QString resText = QString(
                    "T_base_camera:\n%1\n"
                    "重投影误差: %2 mm\n"
                    "有效帧数: %3")
                    .arg(QString::fromStdString(ss_et.str()))
                    .arg(result.reproj_error * 1000.0, 0, 'f', 4)
                    .arg(result.valid_frames);

                setResult(resText);
                setStatus("眼在手外标定成功!");
                APP_LOG_INFO("calibration", "eye_to_hand_success");
                emit calibrationFinished(true, "标定成功");
            } else {
                APP_LOG_ERROR("calibration", QString("eye_to_hand_failed message=%1")
                    .arg(QString::fromStdString(result.message)));
                setStatus(QString("标定失败: %1").arg(
                              QString::fromStdString(result.message)));
                emit calibrationFinished(false,
                    QString::fromStdString(result.message));
            }
        }
    }
    } catch (const cv::Exception& e) {
        APP_LOG_ERROR("calibration", QString("worker_exception type=opencv code=%1 file=%2 line=%3 message=%4")
            .arg(e.code).arg(QString::fromStdString(e.file)).arg(e.line)
            .arg(QString::fromUtf8(e.what())));
        const QString message = QString("标定异常 (OpenCV): %1")
                                    .arg(QString::fromUtf8(e.what()));
        setStatus(message);
        setResult(message);
        emit calibrationFinished(false, message);
    } catch (const std::exception& e) {
        APP_LOG_ERROR("calibration", QString("worker_exception type=std message=%1")
            .arg(QString::fromUtf8(e.what())));
        const QString message = QString("标定异常: %1")
                                    .arg(QString::fromUtf8(e.what()));
        setStatus(message);
        setResult(message);
        emit calibrationFinished(false, message);
    } catch (...) {
        APP_LOG_ERROR("calibration", "worker_exception type=unknown");
        const QString message = "标定发生未知异常";
        setStatus(message);
        setResult(message);
        emit calibrationFinished(false, message);
    }

done:
    const bool robotHandleLost = (m_robotHandle == 0);
    APP_LOG_INFO("calibration", QString("worker_finished elapsed_ms=%1 cancelled=%2 robot_handle_lost=%3")
        .arg(timer.elapsed()).arg(m_cancelRequested.load()).arg(robotHandleLost));
    DiagnosticLogger::instance().flush();
    QMetaObject::invokeMethod(this, [this, robotHandleLost]() {
        m_running = false;
        m_progress = 0;
        emit runningChanged();
        emit progressChanged();
        if (robotHandleLost && m_robotConnected) {
            m_robotConnected = false;
            emit robotConnectedChanged();
        }
    }, Qt::QueuedConnection);
}

// ===================================================================
//  标定回调（C 函数指针，从标定线程调用）
// ===================================================================

int CalibrationController::calibrationCallback(char* msg, int msg_len, int flag)
{
    if (!s_instance) {
        APP_LOG_WARNING("calibration", "callback_ignored reason=no_controller");
        return 0;
    }

    // 取消检查
    if (s_instance->m_cancelRequested) {
        APP_LOG_WARNING("calibration", "callback_cancel_acknowledged");
        return -1;
    }

    // flag 为进度百分比 (0-100) 或有图时为 1
    if (flag > 0 && flag <= 100) {
        s_instance->setProgress(flag);
        int previous = s_instance->m_lastLoggedProgress.load();
        const bool shouldLog = previous < 0 || flag == 100 || flag / 10 > previous / 10;
        if (shouldLog && s_instance->m_lastLoggedProgress.compare_exchange_strong(previous, flag))
            APP_LOG_INFO("calibration", QString("progress percent=%1").arg(flag));
    }

    // msg 非空时是已保存的图像路径，加载并发布到 QML 显示
    if (msg && msg_len > 0) {
        std::string img_path(msg, msg_len);
        // 检查是否是 jpg 图像路径
        if (img_path.find(".jpg") != std::string::npos ||
            img_path.find(".png") != std::string::npos) {
            cv::Mat frame = cv::imread(img_path);
            if (!frame.empty()) {
                APP_LOG_DEBUG("calibration", QString("callback_image_loaded path=%1 size=%2x%3")
                    .arg(QString::fromStdString(img_path)).arg(frame.cols).arg(frame.rows));
                s_instance->publishFrame(frame);
                // 通知 QML 刷新
                QMetaObject::invokeMethod(s_instance, []() {
                    if (s_instance)
                        emit s_instance->frameUpdated();
                }, Qt::QueuedConnection);
            } else {
                APP_LOG_WARNING("calibration", QString("callback_image_load_failed path=%1")
                    .arg(QString::fromStdString(img_path)));
            }
        }
    }

    return 0;
}

void CalibrationController::appendLog(const QString& line)
{
    if (QThread::currentThread() != thread()) {
        QMetaObject::invokeMethod(this, [this, line]() {
            appendLog(line);
        }, Qt::QueuedConnection);
        return;
    }

    // 仅保留最近 500 行，避免内存无限增长
    m_logText += line;
    if (m_logText.count('\n') > 500) {
        int idx = m_logText.indexOf('\n');
        m_logText = m_logText.mid(idx + 1);
    }
    emit logTextChanged();
}

// ===================================================================
//  内部辅助
// ===================================================================

void CalibrationController::setStatus(const QString& msg)
{
    if (QThread::currentThread() != thread()) {
        QMetaObject::invokeMethod(this, [this, msg]() {
            setStatus(msg);
        }, Qt::QueuedConnection);
        return;
    }

    if (m_statusMessage != msg) {
        APP_LOG_INFO("state", QString("status_changed from=%1 to=%2")
            .arg(m_statusMessage, msg));
        m_statusMessage = msg;
        emit statusMessageChanged();
    }
}

void CalibrationController::setProgress(int p)
{
    if (QThread::currentThread() != thread()) {
        QMetaObject::invokeMethod(this, [this, p]() {
            setProgress(p);
        }, Qt::QueuedConnection);
        return;
    }

    if (m_progress != p) {
        APP_LOG_DEBUG("state", QString("progress_changed from=%1 to=%2")
            .arg(m_progress).arg(p));
        m_progress = p;
        emit progressChanged();
    }
}

void CalibrationController::setResult(const QString& text)
{
    if (QThread::currentThread() != thread()) {
        QMetaObject::invokeMethod(this, [this, text]() {
            setResult(text);
        }, Qt::QueuedConnection);
        return;
    }

    m_resultText = text;
    APP_LOG_INFO("state", QString("result_updated characters=%1 empty=%2")
        .arg(text.size()).arg(text.isEmpty()));
    emit resultTextChanged();
}
