#include "CollectionController.hpp"
#include "CameraImageProvider.hpp"
#include "FivePointCapture.hpp"
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

// ===================================================================
//  构造 / 析构
// ===================================================================

CollectionController::CollectionController(QObject* parent)
    : QObject(parent)
{
    m_robotDelivery = std::shared_ptr<QObject>(new QObject, [](QObject* value) { value->deleteLater(); });
    m_tcpPoseTimer = new QTimer(this);
    m_tcpPoseTimer->setInterval(200);
    connect(m_tcpPoseTimer, &QTimer::timeout, this, &CollectionController::pollTcpPose);
    m_tcpPoseTimer->start();

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
    m_tcpPoseTimer->stop();
    ++m_tcpPoseGeneration;
    if (m_robotSession) m_robotSession->closing = true;
    m_cancelRequested = true;
    if (m_collectionThread.joinable())
        m_collectionThread.join();
    // A connect worker may finish after the event loop stopped, before its UI
    // delivery. Adopt the pending handle so shutdown still releases it safely.
    if (const auto pending = m_pendingRobotHandle.exchange(0)) {
        m_robotHandle = pending;
        m_robotConnected = true;
        m_robotSession = std::make_shared<RobotSession>(pending);
        m_robotSession->closing = true;
    }
    if (m_robotConnected && m_robotStopUnconfirmed) {
        try {
            const auto result = cr_stop(m_robotHandle);
            APP_LOG_EVENT_WARNING("robot", "shutdown_stop", "requested", {{"result", QString::number(result)}});
            confirmMotionStopped();
        } catch (const std::exception& error) {
            APP_LOG_EVENT_ERROR("robot", "shutdown_stop", "unconfirmed",
                {{"error", QString::fromUtf8(error.what())}, {"robot_handle_preserved", "true"}});
        }
    }
    m_collectionBusy = false;
    flushPendingLogs();

    // 事件循环退出后，工作线程投递的完成事件可能还未执行。
    // 相机可直接释放；机器人停止仍未确认时，disconnectRobot 保留句柄。

    stopPreview();
    closeCamera();
    disconnectRobot();

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

QString CollectionController::diagnosticLogPath() const
{
    return DiagnosticLogger::instance().logFilePath();
}

QString CollectionController::cameraBackend() const
{
    return QStringLiteral("mvs");
}

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

    // 与界面一致，仅保留最近 1000 行，避免内存无限增长。
    constexpr int kMaxUiLogLines = 1000;
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
