#include "CollectionController.hpp"
#include "CameraImageProvider.hpp"
#include "utils/robot_util.hpp"
#include "utils/collection_undistortion.hpp"
#include "DiagnosticLogger.hpp"
#include "config_resolver.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QThread>
#include <QPointer>
#include <QElapsedTimer>
#include <QStringList>

#include <opencv2/opencv.hpp>

#include "cameras/MVSCamera.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {

constexpr int kMinimumConfiguredExposureUs = 1;
constexpr int kMaximumConfiguredExposureUs = 10000000;

} // namespace

// ---- 静态成员 ----
CollectionController* CollectionController::s_instance = nullptr;

// ===================================================================
//  构造 / 析构
// ===================================================================

CollectionController::CollectionController(QObject* parent)
    : QObject(parent)
{
    s_instance = this;

    m_previewTimer = new QTimer(this);
    m_previewTimer->setInterval(66);  // ~15 fps
    connect(m_previewTimer, &QTimer::timeout,
            this, &CollectionController::onPreviewTick);

    m_logFlushTimer = new QTimer(this);
    m_logFlushTimer->setSingleShot(true);
    m_logFlushTimer->setInterval(75);
    connect(m_logFlushTimer, &QTimer::timeout,
            this, &CollectionController::flushPendingLogs);

    APP_LOG_EVENT_INFO("controller",
                       "ui_controller",
                       "lifecycle",
                       {{"action", "constructed"},
                        {"object", QString::number(reinterpret_cast<quintptr>(this), 16)},
                        {"preview_interval_ms", QString::number(m_previewTimer->interval())}});
}

CollectionController::~CollectionController()
{
    APP_LOG_EVENT_INFO("controller",
                       "lifecycle",
                       "destructor",
                       {{"state", "running"},
                        {"preview", QString::number(m_previewActive)},
                        {"camera_opened", QString::number(m_cameraOpened)},
                        {"robot_connected", QString::number(m_robotConnected)}});
    m_cancelRequested = true;
    if (m_collectionThread.joinable())
        m_collectionThread.join();
    m_collectionBusy = false;
    flushPendingLogs();

    // 事件循环退出后，工作线程投递的完成事件可能还未执行。
    // 此时线程已经 join，可以安全地直接复位，保证硬件资源能够释放。

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

void CollectionController::init(CameraImageProvider* provider)
{
    m_imageProvider = provider;
    QPointer<CollectionController> guard(this);
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
    initializeCollection();
    setStatus("就绪 — 连接相机后可预览与拍照；" + m_undistortionStatus);
}

void CollectionController::setupPaths()
{
    // 获取可执行文件所在目录
    QString exePath = QCoreApplication::applicationDirPath();
    m_appPath = exePath.toStdString() + "/";

    m_paramPath      = m_appPath + "param/";
    APP_LOG_EVENT_INFO("filesystem", "path_prepare", "ready",
                       {{"app", QString::fromStdString(m_appPath)},
                        {"param", QString::fromStdString(m_paramPath)},
                        {"log", DiagnosticLogger::instance().logFilePath()}});
}

CameraConfig CollectionController::currentCameraConfig() const
{
    CameraConfig cfg;
    cfg.cam_ip = m_cameraIp.toStdString();
    cfg.cam_width = m_previewWidth;
    cfg.cam_height = m_previewHeight;
    cfg.exposure_us = m_cameraExposureUs;
    cfg.gain = m_cameraGain;
    return cfg;
}

void CollectionController::loadConfigYaml()
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
    try {
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

    // 配置兼容顺序（逐键回退）：平铺键 > camera_profile 嵌套节 > 编译期默认。
    // 解析逻辑在纯 C++ 助手 config_resolver 中，可无硬件单测。
    const auto resolved =
        config_resolver::resolveCameraConfig(fs, currentCameraConfig());
    config_resolver::CollectionConfig collectionDefaults;
    collectionDefaults.robot_ip = m_robotIp.toStdString();
    const auto collectionConfig = config_resolver::resolveCollectionConfig(fs, collectionDefaults);
    fs.release();

    m_robotIp = QString::fromStdString(collectionConfig.robot_ip);
    QStringList exposureValues;
    for (int exposure : collectionConfig.fusion_exposures_us) exposureValues << QString::number(exposure);
    m_exposureList = exposureValues.join(",");
    for (const auto& warning : collectionConfig.warnings)
        APP_LOG_EVENT_WARNING("config", "config_load", "invalid_value", {{"message", QString::fromStdString(warning)}});
    m_cameraIp = QString::fromStdString(resolved.cam_ip);
    m_previewWidth = resolved.cam_width;
    m_previewHeight = resolved.cam_height;
    m_cameraExposureUs = resolved.exposure_us;
    m_cameraGain = resolved.gain;
    m_cameraProfile = QString::fromStdString(resolved.profile);
    if (!resolved.profile.empty() && !resolved.profile_found) {
        APP_LOG_EVENT_WARNING("config",
                              "config_load",
                              "profile_missing",
                              {{"profile", m_cameraProfile},
                               {"path", QString::fromStdString(cfg_file)}});
    }

    QStringList sources;
    for (const std::string& s : resolved.sources)
        sources << QString::fromStdString(s);

    APP_LOG_EVENT_INFO("config",
                       "config_load",
                       "success",
                       {{"path", QString::fromStdString(cfg_file)},
                        {"camera", m_cameraIp},
                        {"camera_profile", m_cameraProfile},
                        {"resolution_w", QString::number(m_previewWidth)},
                        {"resolution_h", QString::number(m_previewHeight)},
                        {"camera_exposure_us", QString::number(m_cameraExposureUs)},
                        {"robot_ip", m_robotIp},
                        {"fusion_exposures_us", m_exposureList},
                        {"camera_gain", QString::number(m_cameraGain)},
                        {"resolved", sources.join(", ")}});
    } catch (const std::exception& error) {
        APP_LOG_EVENT_ERROR("config", "config_load", "fail",
            {{"path", QString::fromStdString(cfg_file)}, {"error", QString::fromUtf8(error.what())},
             {"using_defaults", "true"}});
    }
}

// ===================================================================
//  属性 setter
// ===================================================================

void CollectionController::setRobotIp(const QString& v) {
    if (m_collectionBusy) {
        APP_LOG_EVENT_WARNING("config",
                              "config_update",
                              "skip",
                              {{"field", "robot_ip"},
                               {"reason", "collection_busy"}});
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
        persistCollectionString("robot_ip", v);
    }
}
void CollectionController::setCameraIp(const QString& v) {
    if (m_collectionBusy) {
        APP_LOG_EVENT_WARNING("config",
                              "config_update",
                              "skip",
                              {{"field", "camera_ip"},
                               {"reason", "collection_busy"}});
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
        persistCollectionString("cam_ip", v);
    }
}
void CollectionController::setPreviewWidth(int v) {
    if (m_collectionBusy) {
        APP_LOG_EVENT_WARNING("config",
                              "config_update",
                              "skip",
                              {{"field", "preview_width"},
                               {"reason", "collection_busy"}});
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
        persistCollectionInteger("cam_width", v);
    }
}
void CollectionController::setPreviewHeight(int v) {
    if (m_collectionBusy) {
        APP_LOG_EVENT_WARNING("config",
                              "config_update",
                              "skip",
                              {{"field", "preview_height"},
                               {"reason", "collection_busy"}});
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
        persistCollectionInteger("cam_height", v);
    }
}

QString CollectionController::diagnosticLogPath() const
{
    return DiagnosticLogger::instance().logFilePath();
}

QString CollectionController::cameraBackend() const
{
    return QStringLiteral("mvs");
}

void CollectionController::connectRobot()
{
    if (m_collectionBusy) {
        setStatus("采集操作中，不能更改机器人连接");
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

void CollectionController::disconnectRobot()
{
    APP_LOG_EVENT_INFO("robot",
                       "robot_disconnect",
                       "start",
                       {{"connected", QString::number(m_robotConnected)},
                        {"handle", QString::number(static_cast<qulonglong>(m_robotHandle))}});
    if (m_collectionBusy) {
        APP_LOG_EVENT_WARNING("robot",
                              "robot_disconnect",
                              "skip",
                              {{"reason", "collection_busy"}});
        setStatus("请先停止采集操作并等待结束，再断开机器人");
        return;
    }

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

bool CollectionController::checkAndRestoreRobotHand()
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

void CollectionController::openCamera()
{
    if (m_collectionBusy) {
        APP_LOG_EVENT_WARNING("camera",
                              "camera_open",
                              "skip",
                              {{"reason", "collection_busy"}});
        setStatus("采集操作中，不能更改相机连接");
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
                       {{"endpoint", m_cameraIp},
                        {"requested", QString("%1x%2").arg(m_previewWidth).arg(m_previewHeight)},
                        {"backend", "mvs"},
                        {"status", "opening"}});
    try {

    // 关闭旧相机（如果存在且配置可能已变更）
    if (m_camera) {
        m_camera->close();
        m_camera.reset();
    }

    // 根据当前 UI 配置创建相机
    // Collection always selects by IP; the optional SDK serial selector stays empty.
    const auto cfg = currentCameraConfig();

    APP_LOG_EVENT_INFO("camera",
                       "camera_open",
                       "instance_create",
                       {{"endpoint", QString::fromStdString(cfg.cam_ip)},
                        {"resolution", QString("%1x%2").arg(cfg.cam_width).arg(cfg.cam_height)},
                        {"exposure_us", QString::number(cfg.exposure_us)},
                        {"gain", QString::number(cfg.gain)}});

    m_camera = std::make_shared<MVSCamera>(cfg);

    setStatus("正在打开相机...");
    if (!m_camera->open()) {
        // 打开失败日志必须包含：选择条件、枚举设备数、候选设备、
        // 失败阶段和 SDK 错误码（由相机实现通过 lastError 提供）
        const CameraErrorDetail camErr = m_camera->lastError();
        APP_LOG_EVENT_ERROR("camera",
                            "camera_open",
                            "fail",
                            {{"reason", "open_failed"},
                             {"stage", QString::fromStdString(camErr.stage)},
                             {"sdk_code", QString("0x%1").arg(camErr.sdk_code, 0, 16)},
                             {"detail", QString::fromStdString(camErr.message)},
                             {"configured_ip", m_cameraIp},
                             {"elapsed_ms", QString::number(timer.elapsed())}});
        setStatus("打开相机失败");
        emit cameraOpenedChanged();
        return;
    }

    m_cameraOpened = true;
    emit cameraOpenedChanged();

    // 刷新只读设备审计信息（实际值，非请求值）
    {
        const CameraDeviceAuditInfo audit = m_camera->getDeviceAuditInfo();
        m_camDeviceModel = QString::fromStdString(audit.model);
        m_camDeviceSerial = QString::fromStdString(audit.serial);
        m_camDeviceFirmware = QString::fromStdString(audit.firmware);
        m_camDeviceTransport = QString::fromStdString(audit.transport);
        m_camActualWidth = audit.actual_width;
        m_camActualHeight = audit.actual_height;
        const int actualExposure = m_camera->getColorExposureTime();
        m_cameraActualExposureUs = actualExposure > 0
            ? actualExposure : audit.exposure_us;
        emit cameraDeviceInfoChanged();
        emit cameraActualExposureUsChanged();
        APP_LOG_EVENT_INFO("camera",
                           "camera_open",
                           "device_identity",
                           {{"backend", cameraBackend()},
                            {"model", m_camDeviceModel},
                            {"serial", m_camDeviceSerial},
                            {"firmware", m_camDeviceFirmware},
                            {"transport", m_camDeviceTransport},
                            {"current_ip", QString::fromStdString(audit.current_ip)},
                            {"actual_resolution", QString("%1x%2").arg(audit.actual_width).arg(audit.actual_height)},
                            {"pixel_format", QString::fromStdString(audit.pixel_format)},
                            {"exposure_us", QString::number(audit.exposure_us)},
                            {"gain", QString::number(audit.gain, 'f', 2)},
                            {"trigger_mode", QString::fromStdString(audit.trigger_mode)},
                            {"sdk_version", QString::fromStdString(audit.sdk_version)}});
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

void CollectionController::closeCamera()
{
    APP_LOG_EVENT_INFO("camera",
                       "camera_close",
                       "start",
                       {{"opened", QString::number(m_cameraOpened)},
                        {"instance", QString::number(reinterpret_cast<quintptr>(m_camera.get()), 16)}});
    if (m_collectionBusy) {
        APP_LOG_EVENT_WARNING("camera",
                              "camera_close",
                              "skip",
                              {{"reason", "collection_busy"}});
        setStatus("请先停止采集操作并等待结束，再关闭相机");
        return;
    }
    stopPreview();

    if (m_camera) {
        m_camera->close();
    }
    m_cameraOpened = false;
    if (m_undistortionMaps) {
        m_undistortionMaps.reset();
        emit collectionChanged();
    }
    if (m_cameraActualExposureUs != -1) {
        m_cameraActualExposureUs = -1;
        emit cameraActualExposureUsChanged();
    }
    emit cameraOpenedChanged();
    setStatus("相机关闭");
    APP_LOG_EVENT_INFO("camera",
                       "camera_close",
                       "complete",
                       {{"opened", "false"}});
}

void CollectionController::adjustCameraExposure(int exposureUs)
{
    if (m_collectionBusy) {
        APP_LOG_EVENT_WARNING("camera", "exposure_update", "skip",
                              {{"reason", "collection_busy"},
                               {"requested_us", QString::number(exposureUs)}});
        setStatus("采集操作中，不能调整曝光");
        return;
    }
    if (exposureUs < kMinimumConfiguredExposureUs ||
        exposureUs > kMaximumConfiguredExposureUs) {
        APP_LOG_EVENT_WARNING(
            "camera", "exposure_update", "invalid_value",
            {{"requested_us", QString::number(exposureUs)},
             {"valid_range", QString("%1..%2")
                  .arg(kMinimumConfiguredExposureUs)
                  .arg(kMaximumConfiguredExposureUs)}});
        setStatus(QString("曝光值无效，有效范围为 %1–%2 μs")
                      .arg(kMinimumConfiguredExposureUs)
                      .arg(kMaximumConfiguredExposureUs));
        return;
    }

    const bool cameraReady = m_cameraOpened && static_cast<bool>(m_camera);
    const int previousConfigured = m_cameraExposureUs;
    int previousDevice = -1;
    int actualExposure = exposureUs;
    bool deviceChanged = false;

    if (cameraReady) {
        int minimumUs = 0;
        int maximumUs = 0;
        if (m_camera->getExposureRange(minimumUs, maximumUs) &&
            (exposureUs < minimumUs || exposureUs > maximumUs)) {
            APP_LOG_EVENT_WARNING(
                "camera", "exposure_update", "out_of_device_range",
                {{"requested_us", QString::number(exposureUs)},
                 {"device_min_us", QString::number(minimumUs)},
                 {"device_max_us", QString::number(maximumUs)}});
            setStatus(QString("曝光超出相机范围：%1–%2 μs")
                          .arg(minimumUs).arg(maximumUs));
            return;
        }

        previousDevice = m_camera->getColorExposureTime();
        if (m_camera->getAutoExposure() && !m_camera->setAutoExposure(false)) {
            const CameraErrorDetail error = m_camera->lastError();
            APP_LOG_EVENT_ERROR("camera", "exposure_update", "fail",
                                {{"step", "disable_auto_exposure"},
                                 {"detail", QString::fromStdString(error.message)}});
            setStatus("关闭自动曝光失败，曝光未更新");
            return;
        }
        if (!m_camera->setColorExposureTime(exposureUs)) {
            const CameraErrorDetail error = m_camera->lastError();
            APP_LOG_EVENT_ERROR("camera", "exposure_update", "fail",
                                {{"step", "apply_device"},
                                 {"requested_us", QString::number(exposureUs)},
                                 {"detail", QString::fromStdString(error.message)}});
            setStatus("相机曝光更新失败");
            return;
        }
        deviceChanged = true;
        actualExposure = m_camera->getColorExposureTime();
        if (actualExposure <= 0) {
            if (previousDevice > 0)
                m_camera->setColorExposureTime(previousDevice);
            APP_LOG_EVENT_ERROR("camera", "exposure_update", "fail",
                                {{"step", "device_readback"},
                                 {"requested_us", QString::number(exposureUs)}});
            setStatus("相机曝光回读失败，已尝试恢复原值");
            return;
        }
    }

    const std::string configPath = m_appPath + "config.yaml";
    std::string persistError;
    if (!config_resolver::updateFlatIntegerConfigFile(
            configPath, "camera_exposure_us", actualExposure, persistError)) {
        if (deviceChanged && previousDevice > 0)
            m_camera->setColorExposureTime(previousDevice);
        const int restoredActual = cameraReady
            ? m_camera->getColorExposureTime() : -1;
        if (m_cameraActualExposureUs != restoredActual) {
            m_cameraActualExposureUs = restoredActual;
            emit cameraActualExposureUsChanged();
        }
        APP_LOG_EVENT_ERROR(
            "config", "exposure_persist", "fail",
            {{"path", QString::fromStdString(configPath)},
             {"requested_us", QString::number(exposureUs)},
             {"actual_us", QString::number(actualExposure)},
             {"rollback_us", QString::number(previousDevice)},
             {"error", QString::fromStdString(persistError)}});
        setStatus("曝光配置保存失败，相机已尝试恢复原值");
        return;
    }

    if (m_cameraExposureUs != actualExposure) {
        m_cameraExposureUs = actualExposure;
        emit cameraExposureUsChanged();
    }
    const int newActual = cameraReady ? actualExposure : -1;
    if (m_cameraActualExposureUs != newActual) {
        m_cameraActualExposureUs = newActual;
        emit cameraActualExposureUsChanged();
    }

    APP_LOG_EVENT_INFO(
        "camera", "exposure_update", "success",
        {{"requested_us", QString::number(exposureUs)},
         {"actual_us", QString::number(actualExposure)},
         {"previous_configured_us", QString::number(previousConfigured)},
         {"camera_opened", cameraReady ? "true" : "false"},
         {"preview_active", m_previewActive ? "true" : "false"},
         {"config_path", QString::fromStdString(configPath)}});
    if (cameraReady) {
        setStatus(QString("曝光已调整为 %1 μs，并已保存")
                      .arg(actualExposure));
    } else {
        setStatus(QString("曝光已保存为 %1 μs，下次打开相机生效")
                      .arg(actualExposure));
    }
}

// ===================================================================
//  预览
// ===================================================================

void CollectionController::startPreview()
{
    if (m_collectionBusy) return;
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
                           {{"interval_ms", QString::number(m_previewTimer->interval())}});
        m_previewTimer->start();
        m_previewFrameCount = 0;
        m_previewEmptyCount = 0;
        m_previewActive = true;
        emit previewActiveChanged();
        setStatus("预览中 — 空格或 Ctrl+S 拍照，自动融合多曝光");
        APP_LOG_EVENT_INFO("preview",
                           "preview_start",
                           "success",
                           {});
    } catch (const std::exception& e) {
        APP_LOG_EVENT_ERROR("preview",
                            "preview_start",
                            "fail",
                            {{"type", "std"},
                             {"message", QString::fromUtf8(e.what())}});
        setStatus(QString("启动预览异常: %1").arg(QString::fromUtf8(e.what())));
    } catch (...) {
        APP_LOG_EVENT_ERROR("preview",
                            "preview_start",
                            "fail",
                            {{"type", "unknown"}});
        setStatus("启动预览发生未知异常");
    }
}

void CollectionController::stopPreview()
{
    const bool wasActive = m_previewActive;
    m_previewTimer->stop();
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

void CollectionController::onPreviewTick()
{
    if (!m_camera || !m_cameraOpened || m_collectionBusy) return;

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

    if (m_undistortionMaps)
        frame = m_undistortionMaps->apply(frame);

    if (m_previewFrameCount == 1 || m_previewFrameCount % 150 == 0) {
        APP_LOG_EVENT_DEBUG("preview",
                           "preview_frame",
                           "heartbeat",
                           {{"frame", QString::number(m_previewFrameCount)},
                            {"empty_frames", QString::number(m_previewEmptyCount)},
                            {"size", QString("%1x%2").arg(frame.cols).arg(frame.rows)},
                            {"channels", QString::number(frame.channels())}});
    }

    publishFrame(frame);
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

void CollectionController::publishFrame(const cv::Mat& bgr)
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
// ===================================================================

bool CollectionController::robotMotionCancellationRequested()
{
    return s_instance && s_instance->m_cancelRequested.load();
}

void CollectionController::appendLog(const QString& line)
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

void CollectionController::flushPendingLogs()
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

void CollectionController::setStatus(const QString& msg)
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
