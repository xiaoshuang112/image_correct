#include "CalibrationController.hpp"
#include "CameraImageProvider.hpp"
#include "auto_eye_in_hand_charuco_calibration.hpp"
#include "eye_to_hand_calibration.hpp"
#include "utils/robot_util.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QMutexLocker>
#include <QDebug>

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

namespace fs = std::filesystem;

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
}

CalibrationController::~CalibrationController()
{
    stopCalibration();
    if (m_calibThread.joinable())
        m_calibThread.join();

    stopPreview();
    closeCamera();
    disconnectRobot();

    s_instance = nullptr;
}

// ===================================================================
//  初始化
// ===================================================================

void CalibrationController::init(CameraImageProvider* provider)
{
    m_imageProvider = provider;
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

    if (!fs::exists(m_paramPath))
        fs::create_directory(m_paramPath);
    if (!fs::exists(m_calibrationPath))
        fs::create_directory(m_calibrationPath);

    qDebug() << "[CalibrationController] appPath:" << m_appPath.c_str();
}

void CalibrationController::loadConfigYaml()
{
    std::string cfg_file = m_appPath + "config.yaml";
    cv::FileStorage fs(cfg_file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        qDebug() << "[CalibrationController] 未找到 config.yaml，使用默认配置";
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

    qDebug() << "[CalibrationController] 加载 config.yaml 完成"
             << "cam:" << m_cameraIp << m_cameraPort
             << "res:" << m_previewWidth << "x" << m_previewHeight;
}

// ===================================================================
//  属性 setter
// ===================================================================

void CalibrationController::setRobotIp(const QString& v) {
    if (m_robotIp != v) { m_robotIp = v; emit robotIpChanged(); }
}
void CalibrationController::setRobotPort(int v) {
    if (m_robotPort != v) { m_robotPort = v; emit robotPortChanged(); }
}
void CalibrationController::setRobotPasswd(const QString& v) {
    if (m_robotPasswd != v) { m_robotPasswd = v; emit robotPasswdChanged(); }
}
void CalibrationController::setCameraToTcpEnd(int v) {
    if (m_cameraToTcpEnd != v) { m_cameraToTcpEnd = v; emit cameraToTcpEndChanged(); }
}
void CalibrationController::setCalibrationType(const QString& v) {
    if (m_calibrationType != v) { m_calibrationType = v; emit calibrationTypeChanged(); }
}
void CalibrationController::setCameraIp(const QString& v) {
    if (m_cameraIp != v) { m_cameraIp = v; emit cameraIpChanged(); }
}
void CalibrationController::setCameraPort(int v) {
    if (m_cameraPort != v) { m_cameraPort = v; emit cameraPortChanged(); }
}
void CalibrationController::setPreviewWidth(int v) {
    if (m_previewWidth != v) { m_previewWidth = v; emit previewWidthChanged(); }
}
void CalibrationController::setPreviewHeight(int v) {
    if (m_previewHeight != v) { m_previewHeight = v; emit previewHeightChanged(); }
}

// ===================================================================
//  机器人连接
// ===================================================================

void CalibrationController::connectRobot()
{
    if (m_robotConnected) {
        setStatus("机器人已连接");
        return;
    }

    setStatus("正在连接机器人...");

    QByteArray ip   = m_robotIp.toLocal8Bit();
    QByteArray pwd  = m_robotPasswd.toLocal8Bit();

    CRresult ret = cr_create_robot(&m_robotHandle, ip.data(),
                                    m_robotPort, pwd.data());
    if (ret != sucess) {
        setStatus(QString("连接机器人失败 (err=%1)").arg(ret));
        emit robotConnectedChanged();
        return;
    }

    m_robotConnected = true;
    emit robotConnectedChanged();

    // 上电使能
    checkAndRestoreRobotHand();
    setStatus("机器人已连接");
}

void CalibrationController::disconnectRobot()
{
    stopPreview();
    stopCalibration();

    if (m_robotConnected) {
        cr_destroy_robot(m_robotHandle);
        m_robotHandle = 0;
        m_robotConnected = false;
        emit robotConnectedChanged();
    }
    setStatus("机器人已断开");
}

bool CalibrationController::checkAndRestoreRobotHand()
{
    if (!m_robotConnected) return false;

    enum RobotModes robotMode;
    CRresult res = cr_get_robotMode(m_robotHandle, &robotMode);
    if (res != sucess) {
        setStatus("获取机械臂模式失败");
        return false;
    }
    if (robotMode != JointPowerOff)
        return true;  // 已上电

    setStatus("机械臂未上电，正在上电使能...");

    res = cr_FaultReset(m_robotHandle);
    if (res == sucess) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    } else {
        setStatus("故障复位失败");
        return false;
    }

    res = cr_poweron(m_robotHandle);
    if (res != sucess) {
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
        setStatus("使能失败");
        return false;
    }

    // 等待 ProgramStop
    while (cr_get_robotMode(m_robotHandle, &robotMode) == sucess) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (robotMode == ProgramStop) break;
    }

    setStatus("机械臂已上电使能");
    return true;
}

// ===================================================================
//  相机操作
// ===================================================================

void CalibrationController::openCamera()
{
    if (m_cameraOpened) return;

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

    qDebug() << "[CalibrationController] 创建相机:" << cfg.cam_ip.c_str()
             << cfg.cam_port << cfg.cam_width << "x" << cfg.cam_height;

#ifdef useORB
    m_camera = std::make_shared<OrbbecCameraV2>(cfg);
#else
    m_camera = std::make_shared<MVSCamera>(cfg);
#endif

    setStatus("正在打开相机...");
    if (!m_camera->open()) {
        setStatus("打开相机失败");
        emit cameraOpenedChanged();
        return;
    }

    m_cameraOpened = true;
    emit cameraOpenedChanged();

    // 读取并保存相机内参
    auto intrinsics = m_camera->getCameraIntrinsics(m_previewWidth, m_previewHeight);
    if (!intrinsics.empty()) {
        cv::FileStorage out(m_paramPath + "camera_params.yaml",
                            cv::FileStorage::WRITE);
        cv::Mat camera_matrix = cv::Mat::zeros(3, 3, CV_64F);
        cv::Mat dist_coeffs   = cv::Mat::zeros(5, 1, CV_64F);

        camera_matrix.at<double>(0, 0) = intrinsics[0][0];
        camera_matrix.at<double>(1, 1) = intrinsics[1][1];
        camera_matrix.at<double>(2, 2) = 1.0;
        camera_matrix.at<double>(0, 2) = intrinsics[0][2];
        camera_matrix.at<double>(1, 2) = intrinsics[1][2];

        for (size_t i = 0; i < intrinsics[3].size(); i++)
            dist_coeffs.at<double>(i, 0) = intrinsics[3][i];

        out << "camera_matrix" << camera_matrix;
        out << "distortion_coefficients" << dist_coeffs;
        out.release();
        qDebug() << "[CalibrationController] 相机内参已保存";
    }

    setStatus("相机已打开");
}

void CalibrationController::closeCamera()
{
    stopPreview();

    if (m_camera) {
        m_camera->close();
    }
    m_cameraOpened = false;
    emit cameraOpenedChanged();
    setStatus("相机关闭");
}

// ===================================================================
//  预览
// ===================================================================

void CalibrationController::startPreview()
{
    if (m_previewActive) return;
    if (!m_camera || !m_cameraOpened) {
        setStatus("请先打开相机");
        return;
    }

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
    m_previewActive = true;
    emit previewActiveChanged();
    setStatus("预览中 — 调整位姿后点击开始标定");
}

void CalibrationController::stopPreview()
{
    m_previewTimer->stop();
    m_previewDetector.reset();
    m_previewActive = false;
    emit previewActiveChanged();
}

void CalibrationController::onPreviewTick()
{
    if (!m_camera || !m_cameraOpened || m_running) return;

    cv::Mat frame = m_camera->captureOneFrame(10);
    if (frame.empty()) return;

    // ChArUco 检测叠加
    cv::Mat display;
    if (m_previewDetector) {
        cv::Mat rvec, tvec;
        if (m_previewDetector->detectAndDraw(frame, display, rvec, tvec)) {
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

    publishFrame(display);
    emit frameUpdated();
}

// ===================================================================
//  cv::Mat → QImage 转换
// ===================================================================

void CalibrationController::publishFrame(const cv::Mat& bgr)
{
    if (bgr.empty() || !m_imageProvider) return;

    QImage qimg(bgr.data, bgr.cols, bgr.rows,
                static_cast<int>(bgr.step),
                QImage::Format_RGB888);
    // OpenCV 默认 BGR → QImage 期望 RGB
    cv::Mat rgb;
    if (bgr.channels() == 3) {
        cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
        qimg = QImage(rgb.data, rgb.cols, rgb.rows,
                      static_cast<int>(rgb.step),
                      QImage::Format_RGB888);
    }

    m_imageProvider->setImage(qimg.copy());  // 深拷贝，rgb 析构后仍然有效
}

// ===================================================================
//  标定线程
// ===================================================================

void CalibrationController::startCalibration()
{
    if (m_running) {
        setStatus("标定已在运行中");
        return;
    }
    if (!m_robotConnected) {
        setStatus("请先连接机器人");
        return;
    }
    if (!m_cameraOpened) {
        setStatus("请先打开相机");
        return;
    }

    // 标定期间暂停预览（标定线程会独占相机）
    stopPreview();

    m_running = true;
    m_cancelRequested = false;
    m_progress = 0;
    setResult("");
    emit runningChanged();
    emit progressChanged();

    if (m_calibThread.joinable())
        m_calibThread.join();

    m_calibThread = std::thread(&CalibrationController::calibrationThreadFunc, this);
}

void CalibrationController::stopCalibration()
{
    if (!m_running) return;
    m_cancelRequested = true;
    setStatus("正在取消标定...");
}

void CalibrationController::calibrationThreadFunc()
{
    // 重定向控制台输出到 QML 日志
    redirectConsole();

    // ================================================================
    //  眼在手内 (eye_in_hand)
    // ================================================================
    if (m_calibrationType == "eye_in_hand") {

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

        // 2. 确保机器人上电
        if (!checkAndRestoreRobotHand()) {
            setStatus("机器人上电使能失败");
            goto done;
        }

        // 3. 构建标定参数
        CharucoBoardConfig board_cfg;

        // 4. 调用核心标定
        {
            setStatus("眼在手内标定 — 采集中...");
            CalibrationResult result;

            QByteArray ip = m_robotIp.toLocal8Bit();
            bool ok = autoEyeInHandCharucoCalibration(
                m_robotHandle,
                ip.data(),
                m_camera,
                board_cfg,
                camera_matrix,
                dist_coeffs,
                m_cameraToTcpEnd,
                result,
                m_calibrationPath,
                m_paramPath + "hand_eye_result.yaml",
                calibrationCallback);

            if (m_cancelRequested) {
                setStatus("标定已取消");
                goto done;
            }

            if (ok) {
                // 保存相机内参（如果标定过程中更新了）
                if (!camera_matrix.empty()) {
                    cv::FileStorage outCam(m_appPath + "camera_params.yaml",
                                           cv::FileStorage::WRITE);
                    outCam << "camera_matrix" << result.camera_matrix;
                    outCam << "distortion_coefficients" << result.dist_coeffs;
                    outCam.release();
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
                emit calibrationFinished(true, "标定成功");
            } else {
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
        }

        std::string save_dir = m_calibrationPath + "eye_to_hand/";

        {
            setStatus("眼在手外标定 — 采集中...");
            EyeToHandResult result;

            QByteArray ip = m_robotIp.toLocal8Bit();
            bool ok = autoEyeToHandCharucoCalibration(
                m_robotHandle,
                ip.data(),
                m_camera,
                detector,
                pose_config,
                result,
                save_dir,
                calibrationCallback);

            if (m_cancelRequested) {
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

                // 保存相机内参（如果标定过程中更新了）
                if (!result.camera_matrix.empty()) {
                    cv::FileStorage outCam(m_appPath + "camera_params.yaml",
                                           cv::FileStorage::WRITE);
                    outCam << "camera_matrix" << result.camera_matrix;
                    outCam << "distortion_coefficients" << result.dist_coeffs;
                    outCam.release();
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
                emit calibrationFinished(true, "标定成功");
            } else {
                setStatus(QString("标定失败: %1").arg(
                              QString::fromStdString(result.message)));
                emit calibrationFinished(false,
                    QString::fromStdString(result.message));
            }
        }
    }

done:
    restoreConsole();
    m_running = false;
    m_progress = 0;
    // 通过 invokeMethod 确保主线程更新属性
    QMetaObject::invokeMethod(this, [this]() {
        emit runningChanged();
        emit progressChanged();
    }, Qt::QueuedConnection);
}

// ===================================================================
//  标定回调（C 函数指针，从标定线程调用）
// ===================================================================

int CalibrationController::calibrationCallback(char* msg, int msg_len, int flag)
{
    if (!s_instance) return 0;

    // 取消检查
    if (s_instance->m_cancelRequested)
        return -1;

    // flag 为进度百分比 (0-100) 或有图时为 1
    if (flag > 0 && flag <= 100) {
        s_instance->setProgress(flag);
    }

    // msg 非空时是已保存的图像路径，加载并发布到 QML 显示
    if (msg && msg_len > 0) {
        std::string img_path(msg, msg_len);
        // 检查是否是 jpg 图像路径
        if (img_path.find(".jpg") != std::string::npos ||
            img_path.find(".png") != std::string::npos) {
            cv::Mat frame = cv::imread(img_path);
            if (!frame.empty()) {
                s_instance->publishFrame(frame);
                // 通知 QML 刷新
                QMetaObject::invokeMethod(s_instance, []() {
                    if (s_instance)
                        emit s_instance->frameUpdated();
                }, Qt::QueuedConnection);
            }
        }
    }

    return 0;
}

// ===================================================================
//  控制台重定向
// ===================================================================

void CalibrationController::redirectConsole()
{
    // 清除之前的日志
    m_logText.clear();
    QMetaObject::invokeMethod(this, [this]() {
        emit logTextChanged();
    }, Qt::QueuedConnection);

    // 创建自定义 streambuf，每次输出一行就追加到日志
    auto cb = [this](const std::string& line) {
        appendLog(QString::fromStdString(line));
    };

    m_coutBuf = new LogStreambuf(cb);
    m_cerrBuf = new LogStreambuf(cb);

    m_oldCout = std::cout.rdbuf(m_coutBuf);
    m_oldCerr = std::cerr.rdbuf(m_cerrBuf);
}

void CalibrationController::restoreConsole()
{
    if (m_oldCout) {
        std::cout.rdbuf(m_oldCout);
        delete m_coutBuf;
        m_coutBuf = nullptr;
        m_oldCout = nullptr;
    }
    if (m_oldCerr) {
        std::cerr.rdbuf(m_oldCerr);
        delete m_cerrBuf;
        m_cerrBuf = nullptr;
        m_oldCerr = nullptr;
    }
}

void CalibrationController::appendLog(const QString& line)
{
    // 仅保留最近 500 行，避免内存无限增长
    m_logText += line;
    if (m_logText.count('\n') > 500) {
        int idx = m_logText.indexOf('\n');
        m_logText = m_logText.mid(idx + 1);
    }
    QMetaObject::invokeMethod(this, [this]() {
        emit logTextChanged();
    }, Qt::QueuedConnection);
}

// ===================================================================
//  内部辅助
// ===================================================================

void CalibrationController::setStatus(const QString& msg)
{
    QMutexLocker lock(&m_statusMutex);
    if (m_statusMessage != msg) {
        m_statusMessage = msg;
        QMetaObject::invokeMethod(this, [this]() {
            emit statusMessageChanged();
        }, Qt::QueuedConnection);
    }
}

void CalibrationController::setProgress(int p)
{
    if (m_progress != p) {
        m_progress = p;
        QMetaObject::invokeMethod(this, [this]() {
            emit progressChanged();
        }, Qt::QueuedConnection);
    }
}

void CalibrationController::setResult(const QString& text)
{
    m_resultText = text;
    QMetaObject::invokeMethod(this, [this]() {
        emit resultTextChanged();
    }, Qt::QueuedConnection);
}
