#include "CalibrationController.hpp"
#include "CameraImageProvider.hpp"
#include "calibration/auto_eye_in_hand_charuco_calibration.hpp"
#include "calibration/calibration_session.hpp"
#include "calibration/eye_to_hand_calibration.hpp"
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
#include <array>

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

    m_logFlushTimer = new QTimer(this);
    m_logFlushTimer->setSingleShot(true);
    m_logFlushTimer->setInterval(75);
    connect(m_logFlushTimer, &QTimer::timeout,
            this, &CalibrationController::flushPendingLogs);

    APP_LOG_EVENT_INFO("controller",
                       "ui_controller",
                       "lifecycle",
                       {{"action", "constructed"},
                        {"object", QString::number(reinterpret_cast<quintptr>(this), 16)},
                        {"preview_interval_ms", QString::number(m_previewTimer->interval())}});
}

CalibrationController::~CalibrationController()
{
    APP_LOG_EVENT_INFO("controller",
                       "lifecycle",
                       "destructor",
                       {{"state", "running"},
                        {"running", QString::number(m_running)},
                        {"preview", QString::number(m_previewActive)},
                        {"camera_opened", QString::number(m_cameraOpened)},
                        {"robot_connected", QString::number(m_robotConnected)}});
    stopCalibration();
    waitForCalibrationThread();
    flushPendingLogs();

    // 事件循环退出后，工作线程投递的完成事件可能还未执行。
    // 此时线程已经 join，可以安全地直接复位，保证硬件资源能够释放。
    m_running = false;

    stopPreview();
    closeCamera();
    disconnectRobot();

    s_instance = nullptr;
    DiagnosticLogger::instance().setUiSink({});
    APP_LOG_EVENT_INFO("controller",
                       "ui_controller",
                       "lifecycle",
                       {{"action", "destructor_complete"},
                        {"object", QString::number(reinterpret_cast<quintptr>(this), 16)}});
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
    APP_LOG_EVENT_INFO("controller",
                       "ui_controller",
                       "initialize",
                       {{"action", "init"},
                        {"image_provider", QString::number(reinterpret_cast<quintptr>(provider), 16)}});
    setupPaths();
    loadConfigYaml();
    std::vector<std::string> recoveredSessions;
    std::string recoveryError;
    const std::array<std::string, 2> auditRoots{{
        m_calibrationPath, m_paramPath + "semi_eye_in_hand/"}};
    for (const std::string& root : auditRoots) {
        std::vector<std::string> recovered;
        if (!CalibrationSession::recoverPendingSessions(root, recovered, recoveryError)) {
            APP_LOG_EVENT_ERROR("calibration.audit", "startup_recovery", "fail",
                                {{"root", QString::fromStdString(root)},
                                 {"error", QString::fromStdString(recoveryError)}});
            continue;
        }
        recoveredSessions.insert(recoveredSessions.end(), recovered.begin(), recovered.end());
    }
    for (const std::string& session : recoveredSessions) {
        APP_LOG_EVENT_WARNING("calibration.audit", "startup_recovery", "recovered",
                              {{"session", QString::fromStdString(session)}});
    }
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
        APP_LOG_EVENT_ERROR("filesystem",
                            "path_prepare",
                            "fail",
                            {{"path", QString::fromStdString(m_calibrationPath)},
                             {"error", QString::fromUtf8(e.what())}});
        throw;
    }

    APP_LOG_EVENT_INFO("filesystem",
                       "path_prepare",
                       "ready",
                       {{"app", QString::fromStdString(m_appPath)},
                        {"param", QString::fromStdString(m_paramPath)},
                        {"calibration", QString::fromStdString(m_calibrationPath)},
                        {"log", DiagnosticLogger::instance().logFilePath()}});
}

void CalibrationController::loadConfigYaml()
{
    std::string cfg_file = m_appPath + "config.yaml";
    if (!QFileInfo::exists(QString::fromStdString(cfg_file))) {
        APP_LOG_EVENT_WARNING("config",
                              "config_load",
                              "skip",
                              {{"reason", "config_missing"},
                               {"path", QString::fromStdString(cfg_file)},
                               {"using_defaults", "true"}});
        return;
    }
    cv::FileStorage fs(cfg_file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        APP_LOG_EVENT_ERROR("config",
                            "config_load",
                            "fail",
                            {{"reason", "config_open_failed"},
                             {"path", QString::fromStdString(cfg_file)},
                             {"using_defaults", "true"}});
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

    APP_LOG_EVENT_INFO("config",
                       "config_load",
                       "success",
                       {{"path", QString::fromStdString(cfg_file)},
                        {"camera", m_cameraIp + ":" + QString::number(m_cameraPort)},
                        {"resolution_w", QString::number(m_previewWidth)},
                        {"resolution_h", QString::number(m_previewHeight)}});
}

// ===================================================================
//  属性 setter
// ===================================================================

void CalibrationController::setRobotIp(const QString& v) {
    if (m_running) {
        APP_LOG_EVENT_WARNING("config",
                              "config_update",
                              "skip",
                              {{"field", "robot_ip"},
                               {"reason", "calibration_running"}});
        return;
    }
    if (m_robotIp != v) {
        APP_LOG_EVENT_INFO("config",
                           "config_update",
                           "success",
                           {{"field", "robot_ip"},
                            {"from", m_robotIp},
                            {"to", v}});
        m_robotIp = v; emit robotIpChanged();
    }
}
void CalibrationController::setRobotPort(int v) {
    if (m_running) {
        APP_LOG_EVENT_WARNING("config",
                              "config_update",
                              "skip",
                              {{"field", "robot_port"},
                               {"reason", "calibration_running"}});
        return;
    }
    if (m_robotPort != v) {
        APP_LOG_EVENT_INFO("config",
                           "config_update",
                           "success",
                           {{"field", "robot_port"},
                            {"from", QString::number(m_robotPort)},
                            {"to", QString::number(v)}});
        m_robotPort = v; emit robotPortChanged();
    }
}
void CalibrationController::setRobotPasswd(const QString& v) {
    if (m_running) {
        APP_LOG_EVENT_WARNING("config",
                              "config_update",
                              "skip",
                              {{"field", "robot_passwd"},
                               {"reason", "calibration_running"}});
        return;
    }
    if (m_robotPasswd != v) {
        APP_LOG_EVENT_INFO("config",
                           "config_update",
                           "success",
                           {{"field", "robot_passwd"},
                            {"value_redacted", "true"}});
        m_robotPasswd = v; emit robotPasswdChanged();
    }
}
void CalibrationController::setCameraToTcpEnd(int v) {
    if (m_running) {
        APP_LOG_EVENT_WARNING("config",
                              "config_update",
                              "skip",
                              {{"field", "camera_to_tcp_end"},
                               {"reason", "calibration_running"}});
        return;
    }
    if (m_cameraToTcpEnd != v) {
        APP_LOG_EVENT_INFO("config",
                           "config_update",
                           "success",
                           {{"field", "camera_to_tcp_end"},
                            {"from", QString::number(m_cameraToTcpEnd)},
                            {"to", QString::number(v)}});
        m_cameraToTcpEnd = v; emit cameraToTcpEndChanged();
    }
}
void CalibrationController::setCalibrationType(const QString& v) {
    if (m_running) {
        APP_LOG_EVENT_WARNING("config",
                              "config_update",
                              "skip",
                              {{"field", "calibration_type"},
                               {"reason", "calibration_running"}});
        return;
    }
    if (m_calibrationType != v) {
        APP_LOG_EVENT_INFO("config",
                           "config_update",
                           "success",
                           {{"field", "calibration_type"},
                            {"from", m_calibrationType},
                            {"to", v}});
        m_calibrationType = v; emit calibrationTypeChanged();
    }
}
void CalibrationController::setCameraIp(const QString& v) {
    if (m_running) {
        APP_LOG_EVENT_WARNING("config",
                              "config_update",
                              "skip",
                              {{"field", "camera_ip"},
                               {"reason", "calibration_running"}});
        return;
    }
    if (m_cameraIp != v) {
        APP_LOG_EVENT_INFO("config",
                           "config_update",
                           "success",
                           {{"field", "camera_ip"},
                            {"from", m_cameraIp},
                            {"to", v}});
        m_cameraIp = v; emit cameraIpChanged();
    }
}
void CalibrationController::setCameraPort(int v) {
    if (m_running) {
        APP_LOG_EVENT_WARNING("config",
                              "config_update",
                              "skip",
                              {{"field", "camera_port"},
                               {"reason", "calibration_running"}});
        return;
    }
    if (m_cameraPort != v) {
        APP_LOG_EVENT_INFO("config",
                           "config_update",
                           "success",
                           {{"field", "camera_port"},
                            {"from", QString::number(m_cameraPort)},
                            {"to", QString::number(v)}});
        m_cameraPort = v; emit cameraPortChanged();
    }
}
void CalibrationController::setPreviewWidth(int v) {
    if (m_running) {
        APP_LOG_EVENT_WARNING("config",
                              "config_update",
                              "skip",
                              {{"field", "preview_width"},
                               {"reason", "calibration_running"}});
        return;
    }
    if (m_previewWidth != v) {
        APP_LOG_EVENT_INFO("config",
                           "config_update",
                           "success",
                           {{"field", "preview_width"},
                            {"from", QString::number(m_previewWidth)},
                            {"to", QString::number(v)}});
        m_previewWidth = v; emit previewWidthChanged();
    }
}
void CalibrationController::setPreviewHeight(int v) {
    if (m_running) {
        APP_LOG_EVENT_WARNING("config",
                              "config_update",
                              "skip",
                              {{"field", "preview_height"},
                               {"reason", "calibration_running"}});
        return;
    }
    if (m_previewHeight != v) {
        APP_LOG_EVENT_INFO("config",
                           "config_update",
                           "success",
                           {{"field", "preview_height"},
                            {"from", QString::number(m_previewHeight)},
                            {"to", QString::number(v)}});
        m_previewHeight = v; emit previewHeightChanged();
    }
}

QString CalibrationController::diagnosticLogPath() const
{
    return DiagnosticLogger::instance().logFilePath();
}

QString CalibrationController::calibrationStageText() const
{
    switch (m_runtimeStatus.stage) {
    case CalibrationRuntimeStage::Idle: return QStringLiteral("空闲");
    case CalibrationRuntimeStage::Preparing: return QStringLiteral("准备");
    case CalibrationRuntimeStage::PoseSearch: return QStringLiteral("搜索观察位");
    case CalibrationRuntimeStage::CandidateGeneration: return QStringLiteral("生成姿态");
    case CalibrationRuntimeStage::Collecting: return QStringLiteral("采集数据");
    case CalibrationRuntimeStage::PnP: return QStringLiteral("PnP");
    case CalibrationRuntimeStage::HandEyeInitial: return QStringLiteral("初次手眼求解");
    case CalibrationRuntimeStage::MadFiltering: return QStringLiteral("MAD过滤");
    case CalibrationRuntimeStage::HandEyeRefit: return QStringLiteral("重新拟合");
    case CalibrationRuntimeStage::QualityValidation: return QStringLiteral("质量检查");
    case CalibrationRuntimeStage::Publishing: return QStringLiteral("发布");
    case CalibrationRuntimeStage::Completed: return QStringLiteral("完成");
    case CalibrationRuntimeStage::Failed: return QStringLiteral("失败");
    case CalibrationRuntimeStage::Cancelled: return QStringLiteral("已取消");
    }
    return QStringLiteral("未知");
}

QString CalibrationController::calibrationOutcome() const
{
    switch (m_runtimeStatus.stage) {
    case CalibrationRuntimeStage::Completed:
        return QStringLiteral("PASS");
    case CalibrationRuntimeStage::Failed:
        return QStringLiteral("FAIL");
    case CalibrationRuntimeStage::Cancelled:
        return QStringLiteral("CANCELLED");
    case CalibrationRuntimeStage::Idle:
        return QStringLiteral("IDLE");
    default:
        return QStringLiteral("RUNNING");
    }
}

void CalibrationController::resetRuntimeStatus()
{
    if (QThread::currentThread() != thread()) {
        QMetaObject::invokeMethod(this, [this]() {
            resetRuntimeStatus();
        }, Qt::QueuedConnection);
        return;
    }
    m_runtimeStatus = makeEmptyCalibrationRuntimeStatus();
    m_handEyeMatrixText.clear();
    emit calibrationRuntimeChanged();
    emit resultTextChanged();
}

void CalibrationController::applyRuntimeStatus(const CalibrationRuntimeStatus& status)
{
    if (QThread::currentThread() != thread()) {
        CalibrationRuntimeStatus copy = status;
        QMetaObject::invokeMethod(this, [this, copy]() {
            applyRuntimeStatus(copy);
        }, Qt::QueuedConnection);
        return;
    }
    m_runtimeStatus = status;
    updateProgressFromRuntime();
    emit calibrationRuntimeChanged();
}

void CalibrationController::setRuntimeStage(CalibrationRuntimeStage stage,
                                            const std::string& detail)
{
    if (QThread::currentThread() != thread()) {
        QMetaObject::invokeMethod(this, [this, stage, detail]() {
            setRuntimeStage(stage, detail);
        }, Qt::QueuedConnection);
        return;
    }
    m_runtimeStatus.stage = stage;
    if (!detail.empty())
        m_runtimeStatus.detail = detail;
    emit calibrationRuntimeChanged();
}

void CalibrationController::updateProgressFromRuntime()
{
    if (!m_useRuntimeProgress.load())
        return;
    if (m_runtimeStatus.candidate_reachable <= 0 ||
        m_runtimeStatus.candidate_processed < 0)
        return;
    int p = m_runtimeStatus.candidate_processed * 100 /
            m_runtimeStatus.candidate_reachable;
    if (p < 0)
        p = 0;
    if (p > 100)
        p = 100;
    if (m_progress == p)
        return;
    m_progress = p;
    emit progressChanged();
}

// ===================================================================
//  机器人连接
// ===================================================================

void CalibrationController::connectRobot()
{
    if (m_running) {
        setStatus("标定运行中，不能更改机器人连接");
        return;
    }
    if (m_robotConnected) {
        APP_LOG_EVENT_INFO("robot",
                           "robot_connect",
                           "skip",
                           {{"reason", "already_connected"}});
        setStatus("机器人已连接");
        return;
    }

    QElapsedTimer timer;
    timer.start();
    APP_LOG_EVENT_INFO("robot",
                       "robot_connect",
                       "start",
                       {{"endpoint", QString("%1:%2").arg(m_robotIp).arg(m_robotPort)}});
    setStatus("正在连接机器人...");

    QByteArray ip   = m_robotIp.toLocal8Bit();
    QByteArray pwd  = m_robotPasswd.toLocal8Bit();

    CRresult ret = cr_create_robot(&m_robotHandle, ip.data(),
                                    m_robotPort, pwd.data());
    if (ret != sucess) {
        APP_LOG_EVENT_ERROR("robot",
                            "robot_connect",
                            "fail",
                            {{"endpoint", QString("%1:%2").arg(m_robotIp).arg(m_robotPort)},
                             {"error", QString::number(ret)},
                             {"elapsed_ms", QString::number(timer.elapsed())}});
        setStatus(QString("连接机器人失败 (err=%1)").arg(ret));
        emit robotConnectedChanged();
        return;
    }

    m_robotConnected = true;
    emit robotConnectedChanged();

    // 上电使能
    const bool ready = checkAndRestoreRobotHand();
    APP_LOG_EVENT_INFO("robot",
                       "robot_connect",
                       "success",
                       {{"handle", QString::number(static_cast<qulonglong>(m_robotHandle))},
                        {"ready", ready ? "true" : "false"},
                        {"elapsed_ms", QString::number(timer.elapsed())}});
    setStatus("机器人已连接");
}

void CalibrationController::disconnectRobot()
{
    APP_LOG_EVENT_INFO("robot",
                       "robot_disconnect",
                       "start",
                       {{"connected", QString::number(m_robotConnected)},
                        {"handle", QString::number(static_cast<qulonglong>(m_robotHandle))}});
    if (m_running) {
        APP_LOG_EVENT_WARNING("robot",
                              "robot_disconnect",
                              "skip",
                              {{"reason", "calibration_running"}});
        setStatus("请先取消标定并等待结束，再断开机器人");
        return;
    }
    stopPreview();

    if (m_robotConnected) {
        if (m_robotHandle != 0) {
            const CRresult result = cr_destroy_robot(m_robotHandle);
            if (result != sucess)
                APP_LOG_EVENT_WARNING("robot",
                                     "robot_destroy",
                                     "fail",
                                     {{"error", QString::number(result)}});
        }
        m_robotHandle = 0;
        m_robotConnected = false;
        emit robotConnectedChanged();
    }
    setStatus("机器人已断开");
    APP_LOG_EVENT_INFO("robot",
                       "robot_disconnect",
                       "complete",
                       {{"connected", QString::number(false)}});
}

bool CalibrationController::checkAndRestoreRobotHand()
{
    if (!m_robotConnected || m_robotHandle == 0) {
        APP_LOG_EVENT_WARNING("robot",
                              "robot_prepare",
                              "skip",
                              {{"reason", "not_connected"},
                               {"connected", QString::number(m_robotConnected)},
                               {"handle", QString::number(static_cast<qulonglong>(m_robotHandle))}});
        return false;
    }

    enum RobotModes robotMode;
    CRresult res = cr_get_robotMode(m_robotHandle, &robotMode);
    if (res != sucess) {
        APP_LOG_EVENT_ERROR("robot",
                           "robot_mode",
                           "fail",
                           {{"error", QString::number(res)}});
        setStatus("获取机械臂模式失败");
        return false;
    }
    APP_LOG_EVENT_DEBUG("robot",
                        "robot_mode",
                        "ready",
                        {{"mode", QString::number(static_cast<int>(robotMode))}});
    if (robotMode != JointPowerOff) {
        APP_LOG_EVENT_INFO("robot",
                           "robot_prepare",
                           "success",
                           {{"action", "already_powered"},
                            {"mode", QString::number(static_cast<int>(robotMode))}});
        return true;  // 已上电
    }

    setStatus("机械臂未上电，正在上电使能...");

    res = cr_FaultReset(m_robotHandle);
    if (res == sucess) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    } else {
        APP_LOG_EVENT_ERROR("robot",
                            "robot_prepare",
                            "fail",
                            {{"step", "fault_reset"},
                             {"error", QString::number(res)}});
        setStatus("故障复位失败");
        return false;
    }

    res = cr_poweron(m_robotHandle);
    if (res != sucess) {
        APP_LOG_EVENT_ERROR("robot",
                            "robot_prepare",
                            "fail",
                            {{"step", "power_on"},
                             {"error", QString::number(res)}});
        setStatus("上电失败");
        return false;
    }

    const auto waitForMode = [this, &robotMode](enum RobotModes target,
                                                const char* phase) {
        constexpr int kPollIntervalMs = 100;
        constexpr int kMaxPolls = 300;  // 最多等待 30 秒
        QElapsedTimer timer;
        timer.start();
        for (int poll = 1; poll <= kMaxPolls; ++poll) {
            const CRresult modeRet = cr_get_robotMode(m_robotHandle, &robotMode);
            if (modeRet != sucess) {
                APP_LOG_EVENT_ERROR("robot",
                                    "robot_prepare",
                                    "retry_fail",
                                    {{"phase", QString::fromLatin1(phase)},
                                     {"error", QString::number(modeRet)},
                                     {"poll", QString::number(poll)},
                                     {"elapsed_ms", QString::number(timer.elapsed())}});
                return false;
            }
            if (robotMode == target) {
                APP_LOG_EVENT_INFO("robot",
                                  "robot_prepare",
                                  "success",
                                  {{"phase", QString::fromLatin1(phase)},
                                   {"mode", QString::number(static_cast<int>(robotMode))},
                                   {"poll", QString::number(poll)},
                                   {"elapsed_ms", QString::number(timer.elapsed())}});
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(kPollIntervalMs));
        }
        APP_LOG_EVENT_ERROR("robot",
                            "robot_prepare",
                            "timeout",
                            {{"phase", QString::fromLatin1(phase)},
                             {"target", QString::number(static_cast<int>(target))},
                             {"last_mode", QString::number(static_cast<int>(robotMode))},
                             {"elapsed_ms", QString::number(timer.elapsed())},
                             {"poll_limit", QString::number(300)}});
        return false;
    };

    if (!waitForMode(JointIdle, "power_on")) {
        setStatus("等待机械臂上电就绪超时或通信失败");
        return false;
    }

    res = cr_enable(m_robotHandle);
    if (res != sucess) {
        APP_LOG_EVENT_ERROR("robot",
                            "robot_prepare",
                            "fail",
                            {{"step", "enable"},
                             {"error", QString::number(res)}});
        setStatus("使能失败");
        return false;
    }

    if (!waitForMode(ProgramStop, "enable")) {
        setStatus("等待机械臂使能就绪超时或通信失败");
        return false;
    }

    setStatus("机械臂已上电使能");
    APP_LOG_EVENT_INFO("robot",
                       "robot_prepare",
                       "success",
                       {{"mode", QString::number(static_cast<int>(robotMode))},
                        {"result", "ready"}});
    return true;
}

// ===================================================================
//  相机操作
// ===================================================================

void CalibrationController::openCamera()
{
    if (m_running) {
        APP_LOG_EVENT_WARNING("camera",
                              "camera_open",
                              "skip",
                              {{"reason", "calibration_running"}});
        setStatus("标定运行中，不能更改相机连接");
        return;
    }
    if (m_cameraOpened) {
        APP_LOG_EVENT_INFO("camera",
                           "camera_open",
                           "skip",
                           {{"reason", "already_open"}});
        return;
    }

    QElapsedTimer timer;
    timer.start();
    APP_LOG_EVENT_INFO("camera",
                       "camera_open",
                       "start",
                       {{"endpoint", QString("%1:%2").arg(m_cameraIp).arg(m_cameraPort)},
                        {"requested", QString("%1x%2").arg(m_previewWidth).arg(m_previewHeight)},
#ifdef useORB
                        {"backend", "orbbec"}
#else
                        {"backend", "mvs"}
#endif
                       ,
                        {"status", "opening"}});
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

    APP_LOG_EVENT_INFO("camera",
                       "camera_open",
                       "instance_create",
                       {{"endpoint", QString("%1:%2").arg(QString::fromStdString(cfg.cam_ip)).arg(cfg.cam_port)},
                        {"resolution", QString("%1x%2").arg(cfg.cam_width).arg(cfg.cam_height)}});

#ifdef useORB
    m_camera = std::make_shared<OrbbecCameraV2>(cfg);
#else
    m_camera = std::make_shared<MVSCamera>(cfg);
#endif

    setStatus("正在打开相机...");
    if (!m_camera->open()) {
        APP_LOG_EVENT_ERROR("camera",
                            "camera_open",
                            "fail",
                            {{"reason", "open_failed"},
                             {"elapsed_ms", QString::number(timer.elapsed())}});
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
        APP_LOG_EVENT_INFO("camera",
                           "camera_intrinsics",
                           "saved",
                           {{"path", QString::fromStdString(m_paramPath + "camera_params.yaml")},
                            {"fx", QString::number(camera_matrix.at<double>(0, 0), 'f', 4)},
                            {"fy", QString::number(camera_matrix.at<double>(1, 1), 'f', 4)},
                            {"cx", QString::number(camera_matrix.at<double>(0, 2), 'f', 4)},
                            {"cy", QString::number(camera_matrix.at<double>(1, 2), 'f', 4)},
                            {"distortion_count", QString::number(static_cast<int>(distortionCount))}});
    } else {
        APP_LOG_EVENT_WARNING("camera",
                              "camera_intrinsics",
                              "skip",
                              {{"reason", "invalid_intrinsics"},
                               {"outer_size", QString::number(intrinsics.size())}});
    }

    setStatus("相机已打开");
    APP_LOG_EVENT_INFO("camera",
                       "camera_open",
                       "success",
                       {{"elapsed_ms", QString::number(timer.elapsed())},
                        {"opened", "true"}});
    } catch (const std::exception& e) {
        APP_LOG_EVENT_ERROR("camera",
                            "camera_open",
                            "fail",
                            {{"type", "std"},
                             {"message", QString::fromUtf8(e.what())},
                             {"elapsed_ms", QString::number(timer.elapsed())}});
        if (m_camera)
            m_camera->close();
        m_cameraOpened = false;
        emit cameraOpenedChanged();
        setStatus(QString("打开相机异常: %1").arg(QString::fromUtf8(e.what())));
    } catch (...) {
        APP_LOG_EVENT_ERROR("camera",
                            "camera_open",
                            "fail",
                            {{"type", "unknown"},
                             {"elapsed_ms", QString::number(timer.elapsed())}});
        if (m_camera)
            m_camera->close();
        m_cameraOpened = false;
        emit cameraOpenedChanged();
        setStatus("打开相机发生未知异常");
    }
}

void CalibrationController::closeCamera()
{
    APP_LOG_EVENT_INFO("camera",
                       "camera_close",
                       "start",
                       {{"opened", QString::number(m_cameraOpened)},
                        {"instance", QString::number(reinterpret_cast<quintptr>(m_camera.get()), 16)}});
    if (m_running) {
        APP_LOG_EVENT_WARNING("camera",
                              "camera_close",
                              "skip",
                              {{"reason", "calibration_running"}});
        setStatus("请先取消标定并等待结束，再关闭相机");
        return;
    }
    stopPreview();

    if (m_camera) {
        m_camera->close();
    }
    m_cameraOpened = false;
    emit cameraOpenedChanged();
    setStatus("相机关闭");
    APP_LOG_EVENT_INFO("camera",
                       "camera_close",
                       "complete",
                       {{"opened", "false"}});
}

// ===================================================================
//  预览
// ===================================================================

void CalibrationController::startPreview()
{
    if (m_previewActive) {
        APP_LOG_EVENT_INFO("preview",
                           "preview_start",
                           "skip",
                           {{"reason", "already_active"}});
        return;
    }
    if (!m_camera || !m_cameraOpened) {
        APP_LOG_EVENT_WARNING("preview",
                              "preview_start",
                              "skip",
                              {{"reason", "camera_not_ready"},
                               {"camera_instance", QString::number(static_cast<bool>(m_camera))},
                               {"camera_opened", QString::number(m_cameraOpened)}});
        setStatus("请先打开相机");
        return;
    }

    try {
        APP_LOG_EVENT_INFO("preview",
                           "preview_start",
                           "start",
                           {{"calibration_type", m_calibrationType},
                            {"interval_ms", QString::number(m_previewTimer->interval())}});
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
        APP_LOG_EVENT_INFO("preview",
                           "preview_start",
                           "success",
                           {{"calibration_type", m_calibrationType}});
    } catch (const std::exception& e) {
        APP_LOG_EVENT_ERROR("preview",
                            "preview_start",
                            "fail",
                            {{"type", "std"},
                             {"message", QString::fromUtf8(e.what())}});
        m_previewDetector.reset();
        setStatus(QString("启动预览异常: %1").arg(QString::fromUtf8(e.what())));
    } catch (...) {
        APP_LOG_EVENT_ERROR("preview",
                            "preview_start",
                            "fail",
                            {{"type", "unknown"}});
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
        APP_LOG_EVENT_INFO("preview",
                           "preview_stop",
                           "success",
                           {{"frames", QString::number(m_previewFrameCount)},
                            {"empty_frames", QString::number(m_previewEmptyCount)}});
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
            APP_LOG_EVENT_WARNING("preview",
                                  "preview_frame",
                                  "empty",
                                  {{"empty_count", QString::number(m_previewEmptyCount)},
                                   {"successful_frames", QString::number(m_previewFrameCount)}});
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
        APP_LOG_EVENT_INFO("preview",
                           "preview_board_detect",
                           "changed",
                           {{"detected", boardDetected ? "true" : "false"},
                            {"frame", QString::number(m_previewFrameCount)}});
        m_haveBoardDetectionState = true;
        m_lastBoardDetected = boardDetected;
    }
    if (m_previewFrameCount == 1 || m_previewFrameCount % 150 == 0) {
        APP_LOG_EVENT_DEBUG("preview",
                           "preview_frame",
                           "heartbeat",
                           {{"frame", QString::number(m_previewFrameCount)},
                            {"empty_frames", QString::number(m_previewEmptyCount)},
                            {"size", QString("%1x%2").arg(frame.cols).arg(frame.rows)},
                            {"channels", QString::number(frame.channels())},
                            {"board", boardDetected ? "true" : "false"}});
    }

    publishFrame(display);
    emit frameUpdated();
    } catch (const std::exception& e) {
        APP_LOG_EVENT_ERROR("preview",
                            "preview_tick",
                            "exception",
                            {{"type", "std"},
                             {"message", QString::fromUtf8(e.what())},
                             {"frame", QString::number(m_previewFrameCount)}});
        stopPreview();
        setStatus(QString("预览异常: %1").arg(QString::fromUtf8(e.what())));
    } catch (...) {
        APP_LOG_EVENT_ERROR("preview",
                            "preview_tick",
                            "exception",
                            {{"type", "unknown"},
                             {"frame", QString::number(m_previewFrameCount)}});
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
        APP_LOG_EVENT_WARNING("image",
                              "publish_frame",
                              "skip",
                              {{"empty", bgr.empty() ? "true" : "false"},
                               {"provider", QString::number(reinterpret_cast<quintptr>(m_imageProvider), 16)}});
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
        APP_LOG_EVENT_WARNING("image",
                              "publish_frame",
                              "fail",
                              {{"channels", QString::number(bgr.channels())},
                               {"size", QString("%1x%2").arg(bgr.cols).arg(bgr.rows)}});
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
        APP_LOG_EVENT_WARNING("calibration",
                              "calibration_start",
                              "skip",
                              {{"reason", "already_running"}});
        setStatus("标定已在运行中");
        return;
    }
    if (!m_robotConnected) {
        APP_LOG_EVENT_WARNING("calibration",
                              "calibration_start",
                              "skip",
                              {{"reason", "robot_disconnected"}});
        setStatus("请先连接机器人");
        return;
    }
    if (!m_cameraOpened) {
        APP_LOG_EVENT_WARNING("calibration",
                              "calibration_start",
                              "skip",
                              {{"reason", "camera_closed"}});
        setStatus("请先打开相机");
        return;
    }

    APP_LOG_EVENT_INFO("calibration",
                       "calibration_start",
                       "start",
                       {{"type", m_calibrationType},
                        {"robot", QString("%1:%2").arg(m_robotIp).arg(m_robotPort)},
                        {"camera", QString("%1:%2").arg(m_cameraIp).arg(m_cameraPort)},
                        {"resolution", QString("%1x%2").arg(m_previewWidth).arg(m_previewHeight)},
                        {"camera_to_tcp_end", QString::number(m_cameraToTcpEnd)}});

    // 标定期间暂停预览（标定线程会独占相机）
    stopPreview();

    m_running = true;
    m_cancelRequested = false;
    setRobotMotionCancelCheck(&CalibrationController::robotMotionCancellationRequested);
    m_lastLoggedProgress = -1;
    m_progress = 0;
    m_useRuntimeProgress = (m_calibrationType == "eye_in_hand");
    setResult("");
    resetRuntimeStatus();
    setRuntimeStage(CalibrationRuntimeStage::Preparing, "controller_prepare");
    emit runningChanged();
    emit progressChanged();

    waitForCalibrationThread();

    try {
        m_calibThread = std::thread(
            &CalibrationController::calibrationThreadFunc,
            this,
            m_calibrationType,
            m_robotIp,
            m_robotPort,
            m_cameraToTcpEnd,
            m_camera);
        APP_LOG_EVENT_INFO("calibration",
                           "calibration_thread",
                           "start",
                           {{"type", m_calibrationType},
                            {"robot", QString("%1:%2").arg(m_robotIp).arg(m_robotPort)},
                            {"camera", QString::number(reinterpret_cast<quintptr>(m_camera.get()), 16)}});
    } catch (const std::exception& e) {
        m_running = false;
        setRobotMotionCancelCheck(nullptr);
        const QString message = QString("无法启动标定线程: %1").arg(
            QString::fromLocal8Bit(e.what()));
        APP_LOG_EVENT_ERROR("calibration",
                            "calibration_thread",
                            "fail",
                            {{"type", "std"},
                             {"message", QString::fromLocal8Bit(e.what())}});
        setStatus(message);
        setRuntimeStage(CalibrationRuntimeStage::Failed, "thread_start_failed");
        emit runningChanged();
        emit calibrationFinished(false, message);
    }
}

void CalibrationController::stopCalibration()
{
    if (!m_running) {
        APP_LOG_EVENT_DEBUG("calibration",
                           "calibration_cancel",
                           "skip",
                           {{"reason", "not_running"}});
        return;
    }
    APP_LOG_EVENT_WARNING("calibration",
                          "calibration_cancel",
                          "requested",
                          {{"requested", "true"}});
    m_cancelRequested = true;
    setStatus("正在取消标定...");
}

void CalibrationController::waitForCalibrationThread()
{
    if (m_calibThread.joinable() &&
        m_calibThread.get_id() != std::this_thread::get_id()) {
        APP_LOG_EVENT_DEBUG("calibration",
                           "calibration_thread",
                           "waiting",
                           {{"action", "join"}});
        m_calibThread.join();
        APP_LOG_EVENT_DEBUG("calibration",
                           "calibration_thread",
                           "joined",
                           {{"action", "join_completed"}});
    }
}

void CalibrationController::calibrationThreadFunc(
    QString calibrationType,
    QString robotIp,
    int robotPort,
    int cameraToTcpEnd,
    std::shared_ptr<AICamera> camera)
{
    QElapsedTimer timer;
    timer.start();
    APP_LOG_EVENT_INFO("calibration",
                       "calibration_pipeline",
                       "worker_started",
                       {{"type", calibrationType},
                        {"robot", QString("%1:%2").arg(robotIp).arg(robotPort)},
                        {"camera_instance", QString::number(reinterpret_cast<quintptr>(camera.get()), 16)},
                        {"camera_to_tcp_end", QString::number(cameraToTcpEnd)},
                        {"output", QString::fromStdString(m_calibrationPath)}});
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
        APP_LOG_EVENT_INFO("calibration",
                           "calibration_input",
                           "matrix_load",
                           {{"type", "eye_in_hand"},
                            {"path", QString::fromStdString(cam_file)},
                            {"loaded", !camera_matrix.empty() ? "true" : "false"},
                            {"matrix_rows", QString::number(camera_matrix.rows)},
                            {"matrix_cols", QString::number(camera_matrix.cols)},
                            {"dist_count", QString::number(static_cast<int>(dist_coeffs.total()))}});

        // 2. 确保机器人上电
        if (!checkAndRestoreRobotHand()) {
            APP_LOG_EVENT_ERROR("calibration",
                                "calibration_pipeline",
                                "fail",
                                {{"type", "eye_in_hand"},
                                 {"reason", "robot_prepare_failed"}});
            setStatus("机器人上电使能失败");
            setRuntimeStage(CalibrationRuntimeStage::Failed, "robot_prepare_failed");
            emit calibrationFinished(false, "机器人上电使能失败");
            goto done;
        }

        // 3. 构建标定参数
        CharucoBoardConfig board_cfg;

        // 4. 调用核心标定
        {
            setStatus("眼在手内标定 — 采集中...");
            CalibrationResult result;

            QByteArray ip = robotIp.toLocal8Bit();
            QPointer<CalibrationController> guard(this);
            auto runtime_cb = [guard](const CalibrationRuntimeStatus& status) {
                if (!guard)
                    return;
                guard->applyRuntimeStatus(status);
            };
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
                cam_file,
                runtime_cb);

            APP_LOG_EVENT_INFO("calibration",
                               "calibration_pipeline",
                               "eye_in_hand_result",
                               {{"success", ok ? "true" : "false"},
                                {"cancelled", m_cancelRequested.load() ? "true" : "false"},
                                {"valid_frames", QString::number(result.valid_frames)},
                                {"error_mm", QString::number(result.reproj_error * 1000.0, 'f', 6)},
                                {"session_dir", QString::fromStdString(result.session_dir)},
                                {"message", QString::fromStdString(result.message)}});

            if (m_cancelRequested) {
                APP_LOG_EVENT_WARNING("calibration",
                                      "calibration_pipeline",
                                      "cancelled",
                                      {{"type", "eye_in_hand"}});
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
                    APP_LOG_EVENT_INFO("calibration",
                                       "calibration_output",
                                       "intrinsics_saved",
                                       {{"type", "eye_in_hand"},
                                        {"path", QString::fromStdString(cameraOutput)}});
                }

                std::stringstream ss_in;
                ss_in << result.T_flange_camera;
                const QString matrixText = QString::fromStdString(ss_in.str());
                QString resText = QString(
                    "T_flange_camera:\n%1\n"
                    "平移一致性 RMS: %2 mm\n"
                    "有效帧数: %3\n"
                    "审计会话: %4")
                    .arg(matrixText)
                    .arg(result.reproj_error * 1000.0, 0, 'f', 4)
                    .arg(result.valid_frames)
                    .arg(QString::fromStdString(result.session_dir));

                QMetaObject::invokeMethod(this, [this, matrixText]() {
                    m_handEyeMatrixText = matrixText;
                    emit resultTextChanged();
                }, Qt::QueuedConnection);
                setResult(resText);
                setStatus("眼在手内标定成功!");
                APP_LOG_EVENT_INFO("calibration",
                                   "calibration_pipeline",
                                   "success",
                                   {{"type", "eye_in_hand"},
                                    {"result", QString::fromStdString(m_paramPath + "hand_eye_result.yaml" )}});
                emit calibrationFinished(true, "标定成功");
            } else {
                APP_LOG_EVENT_ERROR("calibration",
                                    "calibration_pipeline",
                                    "fail",
                                    {{"type", "eye_in_hand"},
                                     {"message", QString::fromStdString(result.message)}});
                if (!result.T_flange_camera.empty()) {
                    std::stringstream ss_in;
                    ss_in << result.T_flange_camera;
                    const QString matrixText = QString::fromStdString(ss_in.str());
                    QMetaObject::invokeMethod(this, [this, matrixText]() {
                        m_handEyeMatrixText = matrixText;
                        emit resultTextChanged();
                    }, Qt::QueuedConnection);
                }
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
            APP_LOG_EVENT_INFO("calibration",
                               "calibration_input",
                               "config_load",
                               {{"type", "eye_to_hand"},
                                {"status", "loaded"},
                                {"path", QString::fromStdString(cfg_path)}});
        } else {
            APP_LOG_EVENT_WARNING("calibration",
                                  "calibration_input",
                                  "config_load",
                                  {{"type", "eye_to_hand"},
                                   {"status", "missing"},
                                   {"path", QString::fromStdString(cfg_path)},
                                   {"using_defaults", "true"}});
        }

        APP_LOG_EVENT_INFO("calibration",
                           "calibration_input",
                           "config_detail",
                           {{"type", "eye_to_hand"},
                            {"offset_x", formatDoubleVector(pose_config.offset_x)},
                            {"offset_y", formatDoubleVector(pose_config.offset_y)},
                            {"offset_z", formatDoubleVector(pose_config.offset_z)},
                            {"offset_rx", formatDoubleVector(pose_config.offset_rx)},
                            {"offset_ry", formatDoubleVector(pose_config.offset_ry)},
                            {"offset_rz", formatDoubleVector(pose_config.offset_rz)},
                            {"min_corners", QString::number(pose_config.min_corners)},
                            {"min_valid_frames", QString::number(pose_config.min_valid_frames)}});

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

            APP_LOG_EVENT_INFO("calibration",
                               "calibration_pipeline",
                               "eye_to_hand_result",
                               {{"success", ok ? "true" : "false"},
                                {"cancelled", m_cancelRequested.load() ? "true" : "false"},
                                {"valid_frames", QString::number(result.valid_frames)},
                                {"error_mm", QString::number(result.reproj_error * 1000.0, 'f', 6)},
                                {"message", QString::fromStdString(result.message)}});

            if (m_cancelRequested) {
                APP_LOG_EVENT_WARNING("calibration",
                                      "calibration_pipeline",
                                      "cancelled",
                                      {{"type", "eye_to_hand"}});
                setStatus("标定已取消");
                setRuntimeStage(CalibrationRuntimeStage::Cancelled,
                                "eye_to_hand_cancelled");
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
                APP_LOG_EVENT_INFO("calibration",
                                   "calibration_output",
                                   "result_saved",
                                   {{"type", "eye_to_hand"},
                                    {"path", QString::fromStdString(out_file)}});

                // 保存相机内参（如果标定过程中更新了）
                if (!result.camera_matrix.empty()) {
                    const std::string cameraOutput = m_appPath + "camera_params.yaml";
                    cv::FileStorage outCam(cameraOutput,
                                           cv::FileStorage::WRITE);
                    outCam << "camera_matrix" << result.camera_matrix;
                    outCam << "distortion_coefficients" << result.dist_coeffs;
                    outCam.release();
                    APP_LOG_EVENT_INFO("calibration",
                                       "calibration_output",
                                       "intrinsics_saved",
                                       {{"type", "eye_to_hand"},
                                        {"path", QString::fromStdString(cameraOutput)}});
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
                setRuntimeStage(CalibrationRuntimeStage::Completed,
                                "eye_to_hand_complete");
                APP_LOG_EVENT_INFO("calibration",
                                   "calibration_pipeline",
                                   "success",
                                   {{"type", "eye_to_hand"}});
                emit calibrationFinished(true, "标定成功");
            } else {
                APP_LOG_EVENT_ERROR("calibration",
                                    "calibration_pipeline",
                                    "fail",
                                    {{"type", "eye_to_hand"},
                                     {"message", QString::fromStdString(result.message)}});
                setStatus(QString("标定失败: %1").arg(
                              QString::fromStdString(result.message)));
                setRuntimeStage(CalibrationRuntimeStage::Failed,
                                "eye_to_hand_failed");
                emit calibrationFinished(false,
                    QString::fromStdString(result.message));
            }
        }
        }
    } catch (const cv::Exception& e) {
        APP_LOG_EVENT_ERROR("calibration",
                            "calibration_pipeline",
                            "exception",
                            {{"type", "opencv"},
                             {"code", QString::number(e.code)},
                             {"file", QString::fromStdString(e.file)},
                             {"line", QString::number(e.line)},
                             {"message", QString::fromUtf8(e.what())}});
        const QString message = QString("标定异常 (OpenCV): %1")
                                    .arg(QString::fromUtf8(e.what()));
        setStatus(message);
        setResult(message);
        emit calibrationFinished(false, message);
    } catch (const std::exception& e) {
        APP_LOG_EVENT_ERROR("calibration",
                            "calibration_pipeline",
                            "exception",
                            {{"type", "std"},
                             {"message", QString::fromUtf8(e.what())}});
        const QString message = QString("标定异常: %1")
                                    .arg(QString::fromUtf8(e.what()));
        setStatus(message);
        setResult(message);
        emit calibrationFinished(false, message);
    } catch (...) {
        APP_LOG_EVENT_ERROR("calibration",
                            "calibration_pipeline",
                            "exception",
                            {{"type", "unknown"}});
        const QString message = "标定发生未知异常";
        setStatus(message);
        setResult(message);
        emit calibrationFinished(false, message);
    }

done:
    setRobotMotionCancelCheck(nullptr);
    const bool robotHandleLost = (m_robotHandle == 0);
    APP_LOG_EVENT_INFO("calibration",
                       "calibration_pipeline",
                       "worker_finished",
                       {{"elapsed_ms", QString::number(timer.elapsed())},
                        {"cancelled", m_cancelRequested.load() ? "true" : "false"},
                        {"robot_handle_lost", robotHandleLost ? "true" : "false"}});
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
        APP_LOG_EVENT_WARNING("calibration",
                              "calibration_callback",
                              "ignore",
                              {{"reason", "no_controller"}});
        return 0;
    }

    // 取消检查
    if (s_instance->m_cancelRequested) {
        APP_LOG_EVENT_WARNING("calibration",
                              "calibration_callback",
                              "acknowledged",
                              {{"reason", "cancel_requested"}});
        return -1;
    }

    // flag 为进度百分比 (0-100) 或有图时为 1。
    // Eye-in-Hand 主进度由 Runtime Status 的 processed/reachable 驱动；
    // 本 callback 仅继续负责图片通知，以及 Eye-to-Hand 兼容进度。
    if (flag > 0 && flag <= 100 && !s_instance->m_useRuntimeProgress.load()) {
        s_instance->setProgress(flag);
        int previous = s_instance->m_lastLoggedProgress.load();
        const bool shouldLog = previous < 0 || flag == 100 || flag / 10 > previous / 10;
        if (shouldLog && s_instance->m_lastLoggedProgress.compare_exchange_strong(previous, flag))
            APP_LOG_EVENT_INFO("calibration",
                               "calibration_progress",
                               "update",
                               {{"percent", QString::number(flag)}});
    }

    // msg 非空时是已保存的图像路径，加载并发布到 QML 显示
    if (msg && msg_len > 0) {
        std::string img_path(msg, msg_len);
        // 检查是否是 jpg 图像路径
        if (img_path.find(".jpg") != std::string::npos ||
            img_path.find(".png") != std::string::npos) {
            cv::Mat frame = cv::imread(img_path);
            if (!frame.empty()) {
                APP_LOG_EVENT_DEBUG("calibration",
                                   "calibration_callback",
                                   "image_loaded",
                                   {{"path", QString::fromStdString(img_path)},
                                    {"size", QString("%1x%2").arg(frame.cols).arg(frame.rows)}});
                s_instance->publishFrame(frame);
                // 通知 QML 刷新
                QMetaObject::invokeMethod(s_instance, []() {
                    if (s_instance)
                        emit s_instance->frameUpdated();
                }, Qt::QueuedConnection);
            } else {
                APP_LOG_EVENT_WARNING("calibration",
                                      "calibration_callback",
                                      "image_load_failed",
                                      {{"path", QString::fromStdString(img_path)}});
            }
        }
    }

    return 0;
}

bool CalibrationController::robotMotionCancellationRequested()
{
    return s_instance && s_instance->m_cancelRequested.load();
}

void CalibrationController::setLogAutoScroll(bool enabled)
{
    if (m_logAutoScroll == enabled)
        return;
    m_logAutoScroll = enabled;
    emit logAutoScrollChanged();
}

void CalibrationController::appendLog(const QString& line)
{
    if (QThread::currentThread() != thread()) {
        QMetaObject::invokeMethod(this, [this, line]() {
            appendLog(line);
        }, Qt::QueuedConnection);
        return;
    }

    m_pendingLogLines.append(line);
    if (m_logFlushTimer && !m_logFlushTimer->isActive())
        m_logFlushTimer->start();
}

void CalibrationController::flushPendingLogs()
{
    if (QThread::currentThread() != thread()) {
        QMetaObject::invokeMethod(this, [this]() {
            flushPendingLogs();
        }, Qt::QueuedConnection);
        return;
    }
    if (m_pendingLogLines.isEmpty())
        return;

    m_logText += m_pendingLogLines.join(QString());
    m_pendingLogLines.clear();

    // 仅保留最近约 800 行，避免内存无限增长
    constexpr int kMaxUiLogLines = 800;
    int newlineCount = m_logText.count('\n');
    while (newlineCount > kMaxUiLogLines) {
        const int idx = m_logText.indexOf('\n');
        if (idx < 0)
            break;
        m_logText = m_logText.mid(idx + 1);
        --newlineCount;
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
        APP_LOG_EVENT_INFO("state",
                          "status_update",
                          "changed",
                          {{"from", m_statusMessage},
                           {"to", msg}});
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
        APP_LOG_EVENT_DEBUG("state",
                           "progress_update",
                           "changed",
                           {{"from", QString::number(m_progress)},
                            {"to", QString::number(p)}});
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
    APP_LOG_EVENT_INFO("state",
                       "result_update",
                       "changed",
                       {{"characters", QString::number(text.size())},
                        {"empty", text.isEmpty() ? "true" : "false"}});
    emit resultTextChanged();
}
