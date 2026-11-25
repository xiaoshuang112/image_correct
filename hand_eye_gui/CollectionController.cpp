#include "CollectionController.hpp"
#include "CameraImageProvider.hpp"
#include "NinePointCapture.hpp"
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

    m_ninePointYawDeg = collection::defaultNinePointYawOffsets;
    m_previewTimer = new QTimer(this);
    m_previewTimer->setInterval(66);  // ~15 fps
    connect(m_previewTimer, &QTimer::timeout,
            this, &CollectionController::onPreviewTick);

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
    delete m_coordinateDialog.data();
    ++m_tcpPoseGeneration;
    if (m_robotSession) m_robotSession->closing = true;
    m_cancelRequested = true;
    {
        std::lock_guard<std::mutex> lock(m_exposurePreviewMutex);
        m_exposurePreviewClosing = true;
    }
    m_exposurePreviewChanged.notify_one();
    if (m_jogLease) m_jogLease->release();
    finishTeachPreferences();
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
            const auto result = cr_moveControl(m_robotHandle, DecStop);
            APP_LOG_EVENT_WARNING("robot", "shutdown_stop", "requested", {{"result", QString::number(result)}});
            confirmMotionStopped();
        } catch (const std::exception& error) {
            APP_LOG_EVENT_ERROR("robot", "shutdown_stop", "unconfirmed",
                {{"error", QString::fromUtf8(error.what())}, {"robot_handle_preserved", "true"}});
        }
    }
    m_collectionBusy = false;

    // 事件循环退出后，工作线程投递的完成事件可能还未执行。
    // 相机可直接释放；机器人停止仍未确认时，disconnectRobot 保留句柄。

    stopPreview();
    closeCamera();
    disconnectRobot();

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
    APP_LOG_EVENT_INFO("controller",
                       "ui_controller",
                       "initialize",
                       {{"action", "init"},
                        {"image_provider", QString::number(reinterpret_cast<quintptr>(provider), 16)}});
    setupPaths();
    loadConfigYaml();
    loadTeachPreferences();
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

CameraConfig CollectionController::currentCamera2Config() const
{
    CameraConfig cfg;
    cfg.cam_ip = m_camera2Ip.toStdString();
    cfg.cam_sn = m_camera2Sn.toStdString();
    cfg.cam_width = m_preview2Width;
    cfg.cam_height = m_preview2Height;
    cfg.exposure_us = m_camera2ExposureUs;
    cfg.gain = m_camera2Gain;
    return cfg;
}

bool CollectionController::camera2IdentityConfigured() const
{
    return !m_camera2Ip.trimmed().isEmpty() || !m_camera2Sn.trimmed().isEmpty();
}

void CollectionController::setCamera2Status(const QString& msg)
{
    if (m_camera2StatusMessage == msg) return;
    m_camera2StatusMessage = msg;
    emit camera2StatusMessageChanged();
}

void CollectionController::setCameraHealth(int cameraIndex, bool healthy)
{
    bool changed = false;
    if (cameraIndex == 1 && m_camera1Healthy != healthy) {
        m_camera1Healthy = healthy;
        changed = true;
    } else if (cameraIndex == 2 && m_camera2Healthy != healthy) {
        m_camera2Healthy = healthy;
        changed = true;
    }
    if (changed) emit cameraHealthChanged();
    updateDualCameraReady();
}

void CollectionController::updateDualCameraReady()
{
    const bool ready = m_dualCameraMode && m_cameraOpened && m_camera2Opened
        && m_previewActive && m_camera1Healthy && m_camera2Healthy;
    if (m_dualCameraReady == ready) return;
    m_dualCameraReady = ready;
    emit dualCameraReadyChanged();
}

void CollectionController::setDualCameraMode(bool enabled)
{
    if (m_dualCameraMode == enabled) return;
    if (m_collectionBusy || m_settingsSaving) {
        setStatus("操作进行中，无法切换采集模式");
        return;
    }
    if (m_cameraOpened || m_camera2Opened) {
        setStatus("请先断开相机，再切换双相机模式");
        return;
    }
    m_dualCameraMode = enabled;
    if (!enabled && m_activePreviewCamera != 1) {
        m_activePreviewCamera = 1;
        emit activePreviewCameraChanged();
    }
    updateDualCameraReady();
    emit dualCameraModeChanged();
    setStatus(enabled ? "已切换到双相机模式，请配置并连接两台相机" : "已切换到单相机模式");
}

void CollectionController::setActivePreviewCamera(int channel)
{
    const int normalized = (channel == 2) ? 2 : 1;
    if (!m_dualCameraMode) return;
    if (m_activePreviewCamera == normalized) return;
    m_activePreviewCamera = normalized;
    ++m_previewGeneration;
    m_previewEmptyCount = 0;
    m_healthProbeFailCount = 0;
    m_switchDiscardPending = true;
    m_lastHealthProbeTimer.invalidate();
    if (m_imageProvider) m_imageProvider->clearImage();
    emit activePreviewCameraChanged();
    emit frameCleared();
    if (previewFrameGrabAllowed() && !m_previewFuture.valid())
        onPreviewTick();
}

void CollectionController::togglePreviewCamera()
{
    if (!m_dualCameraMode) return;
    setActivePreviewCamera(m_activePreviewCamera == 1 ? 2 : 1);
}

void CollectionController::releaseCameraPair()
{
    if (m_previewFuture.valid()) m_previewFuture.wait();
    if (m_healthProbeFuture.valid()) m_healthProbeFuture.wait();
    if (m_camera) {
        m_camera->close();
        m_camera.reset();
    }
    if (m_camera2) {
        m_camera2->close();
        m_camera2.reset();
    }
    if (m_cameraOpened) {
        m_cameraOpened = false;
        emit cameraOpenedChanged();
    }
    if (m_camera2Opened) {
        m_camera2Opened = false;
        emit camera2OpenedChanged();
    }
    if (m_imageProvider) m_imageProvider->clearImage();
    emit frameCleared();
    m_previewFrameCount = 0;
    m_previewEmptyCount = 0;
    m_healthProbeFailCount = 0;
    m_switchDiscardPending = false;
    m_lastHealthProbeTimer.invalidate();
    m_camera1FaultReported = false;
    m_camera2FaultReported = false;
    setCameraHealth(1, false);
    setCameraHealth(2, false);
    if (m_cameraActualExposureUs != -1) {
        m_cameraActualExposureUs = -1;
        emit cameraActualExposureUsChanged();
    }
    if (m_camera2ActualExposureUs != -1) {
        m_camera2ActualExposureUs = -1;
        emit camera2ActualExposureUsChanged();
    }
    setCamera2Status("相机 2 未连接");
    updateDualCameraReady();
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
    config_resolver::ResolvedCamera2Config camera2Defaults;
    camera2Defaults.cam_width = m_preview2Width;
    camera2Defaults.cam_height = m_preview2Height;
    camera2Defaults.exposure_us = m_camera2ExposureUs;
    camera2Defaults.gain = m_camera2Gain;
    const auto camera2 = config_resolver::resolveCamera2Config(fs, camera2Defaults);
    fs.release();

    m_robotIp = QString::fromStdString(collectionConfig.robot_ip);
    m_robotSpeedMmS = collectionConfig.robot_speed_mm_s;
    m_ninePointOffsetXMm = collectionConfig.nine_point_offset_x_mm;
    m_ninePointOffsetYMm = collectionConfig.nine_point_offset_y_mm;
    for (size_t point = 0; point < m_ninePointYawDeg.size() && point < collectionConfig.nine_point_yaw_deg.size(); ++point)
        m_ninePointYawDeg[point] = collectionConfig.nine_point_yaw_deg[point];
    QStringList yawValues;
    for (double angle : m_ninePointYawDeg) yawValues << QString::number(angle);
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

    m_camera2Ip = QString::fromStdString(camera2.cam_ip);
    m_camera2Sn = QString::fromStdString(camera2.cam_sn);
    m_preview2Width = camera2.cam_width;
    m_preview2Height = camera2.cam_height;
    m_camera2ExposureUs = camera2.exposure_us;
    m_camera2Gain = camera2.gain;
    QStringList exposure2Values;
    for (int exposure : camera2.fusion_exposures_us) exposure2Values << QString::number(exposure);
    m_exposure2List = exposure2Values.join(",");
    for (const auto& warning : camera2.warnings)
        APP_LOG_EVENT_WARNING("config", "config_load", "invalid_value",
                              {{"message", QString::fromStdString(warning)}});

    QStringList sources;
    for (const std::string& s : resolved.sources)
        sources << QString::fromStdString(s);
    QStringList sources2;
    for (const std::string& s : camera2.sources)
        sources2 << QString::fromStdString(s);

    APP_LOG_EVENT_INFO("config",
                       "config_load",
                       "success",
                       {{"path", QString::fromStdString(cfg_file)},
                        {"camera", m_cameraIp},
                        {"camera_profile", m_cameraProfile},
                        {"resolution_w", QString::number(m_previewWidth)},
                        {"resolution_h", QString::number(m_previewHeight)},
                        {"camera_exposure_us", QString::number(m_cameraExposureUs)},
                        {"cam2_ip", m_camera2Ip},
                        {"cam2_sn", m_camera2Sn},
                        {"cam2_resolution", QString("%1x%2").arg(m_preview2Width).arg(m_preview2Height)},
                        {"camera2_exposure_us", QString::number(m_camera2ExposureUs)},
                        {"camera2_fusion_exposures_us", m_exposure2List},
                        {"robot_ip", m_robotIp},
                        {"robot_speed_mm_s", QString::number(m_robotSpeedMmS)},
                        {"fusion_exposures_us", m_exposureList},
                        {"nine_point_offset_x_mm", QString::number(m_ninePointOffsetXMm)},
                        {"nine_point_offset_y_mm", QString::number(m_ninePointOffsetYMm)},
                        {"nine_point_yaw_deg", yawValues.join(",")},
                        {"camera_gain", QString::number(m_cameraGain)},
                        {"camera2_gain", QString::number(m_camera2Gain)},
                        {"resolved", sources.join(", ")},
                        {"resolved_cam2", sources2.join(", ")}});
    emit ninePointPlanChanged();
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
        APP_LOG_EVENT_WARNING("camera", "camera_open", "skip", {{"reason", "collection_busy"}});
        setStatus("采集操作中，不能更改相机连接");
        return;
    }
    if (m_cameraOpened) {
        APP_LOG_EVENT_INFO("camera", "camera_open", "skip", {{"reason", "already_open"}});
        return;
    }

    QElapsedTimer timer;
    timer.start();
    const auto applyAudit = [this](AICamera& device, bool second) {
        const CameraDeviceAuditInfo audit = device.getDeviceAuditInfo();
        const int actualExposure = device.getColorExposureTime();
        const int resolvedExposure = actualExposure > 0 ? actualExposure : audit.exposure_us;
        if (second) {
            m_cam2DeviceModel = QString::fromStdString(audit.model);
            m_cam2DeviceSerial = QString::fromStdString(audit.serial);
            m_cam2DeviceFirmware = QString::fromStdString(audit.firmware);
            m_cam2DeviceTransport = QString::fromStdString(audit.transport);
            m_cam2ActualWidth = audit.actual_width;
            m_cam2ActualHeight = audit.actual_height;
            m_camera2ActualExposureUs = resolvedExposure;
            emit camera2DeviceInfoChanged();
            emit camera2ActualExposureUsChanged();
        } else {
            m_camDeviceModel = QString::fromStdString(audit.model);
            m_camDeviceSerial = QString::fromStdString(audit.serial);
            m_camDeviceFirmware = QString::fromStdString(audit.firmware);
            m_camDeviceTransport = QString::fromStdString(audit.transport);
            m_camActualWidth = audit.actual_width;
            m_camActualHeight = audit.actual_height;
            m_cameraActualExposureUs = resolvedExposure;
            emit cameraDeviceInfoChanged();
            emit cameraActualExposureUsChanged();
        }
        APP_LOG_EVENT_INFO("camera", "camera_open", "device_identity",
            {{"camera", second ? "2" : "1"},
             {"backend", cameraBackend()},
             {"model", QString::fromStdString(audit.model)},
             {"serial", QString::fromStdString(audit.serial)},
             {"firmware", QString::fromStdString(audit.firmware)},
             {"transport", QString::fromStdString(audit.transport)},
             {"current_ip", QString::fromStdString(audit.current_ip)},
             {"actual_resolution", QString("%1x%2").arg(audit.actual_width).arg(audit.actual_height)},
             {"pixel_format", QString::fromStdString(audit.pixel_format)},
             {"exposure_us", QString::number(resolvedExposure)},
             {"gain", QString::number(audit.gain, 'f', 2)},
             {"trigger_mode", QString::fromStdString(audit.trigger_mode)},
             {"sdk_version", QString::fromStdString(audit.sdk_version)}});
        return audit;
    };
    const auto logOpenFailure = [&timer](int cameraIndex, const AICamera& device, const QString& endpoint) {
        const CameraErrorDetail camErr = device.lastError();
        APP_LOG_EVENT_ERROR("camera", "camera_open", "fail",
            {{"camera", QString::number(cameraIndex)},
             {"reason", "open_failed"},
             {"stage", QString::fromStdString(camErr.stage)},
             {"sdk_code", QString("0x%1").arg(camErr.sdk_code, 0, 16)},
             {"detail", QString::fromStdString(camErr.message)},
             {"endpoint", endpoint},
             {"elapsed_ms", QString::number(timer.elapsed())}});
    };

    if (!m_dualCameraMode) {
        APP_LOG_EVENT_INFO("camera", "camera_open", "start",
            {{"endpoint", m_cameraIp},
             {"requested", QString("%1x%2").arg(m_previewWidth).arg(m_previewHeight)},
             {"backend", "mvs"}, {"dual", "false"}});
        try {
            if (m_camera) {
                m_camera->close();
                m_camera.reset();
            }
            const auto cfg = currentCameraConfig();
            m_camera = std::make_shared<MVSCamera>(cfg);
            setStatus("正在打开相机...");
            if (!m_camera->open()) {
                logOpenFailure(1, *m_camera, m_cameraIp);
                m_camera.reset();
                setStatus("打开相机失败");
                emit cameraOpenedChanged();
                return;
            }
            m_cameraOpened = true;
            emit cameraOpenedChanged();
            applyAudit(*m_camera, false);
            setStatus("相机已打开");
            APP_LOG_EVENT_INFO("camera", "camera_open", "success",
                {{"elapsed_ms", QString::number(timer.elapsed())}, {"opened", "true"}});
        } catch (const std::exception& e) {
            APP_LOG_EVENT_ERROR("camera", "camera_open", "fail",
                {{"type", "std"}, {"message", QString::fromUtf8(e.what())},
                 {"elapsed_ms", QString::number(timer.elapsed())}});
            if (m_camera) {
                m_camera->close();
                m_camera.reset();
            }
            m_cameraOpened = false;
            emit cameraOpenedChanged();
            setStatus(QString("打开相机异常: %1").arg(QString::fromUtf8(e.what())));
        } catch (...) {
            APP_LOG_EVENT_ERROR("camera", "camera_open", "fail",
                {{"type", "unknown"}, {"elapsed_ms", QString::number(timer.elapsed())}});
            if (m_camera) {
                m_camera->close();
                m_camera.reset();
            }
            m_cameraOpened = false;
            emit cameraOpenedChanged();
            setStatus("打开相机发生未知异常");
        }
        return;
    }

    if (!camera2IdentityConfigured()) {
        setStatus("请先在设置中配置相机 2 的 IP 或序列号");
        return;
    }
    const QString ip2 = m_camera2Ip.trimmed();
    if (!ip2.isEmpty() && !m_cameraIp.trimmed().isEmpty() && ip2 == m_cameraIp.trimmed()) {
        setStatus("相机 1 与相机 2 的 IP 不能相同");
        return;
    }
    const QString cam2Endpoint = m_camera2Sn.trimmed().isEmpty() ? ip2 : m_camera2Sn.trimmed();
    APP_LOG_EVENT_INFO("camera", "camera_open", "start",
        {{"dual", "true"}, {"cam1", m_cameraIp}, {"cam2", cam2Endpoint}});
    releaseCameraPair();
    setStatus("正在打开双相机...");
    try {
        m_camera = std::make_shared<MVSCamera>(currentCameraConfig());
        if (!m_camera->open()) {
            logOpenFailure(1, *m_camera, m_cameraIp);
            releaseCameraPair();
            setStatus("打开相机 1 失败");
            return;
        }
        m_cameraOpened = true;
        emit cameraOpenedChanged();

        m_camera2 = std::make_shared<MVSCamera>(currentCamera2Config());
        if (!m_camera2->open()) {
            logOpenFailure(2, *m_camera2, cam2Endpoint);
            releaseCameraPair();
            setStatus("打开相机 2 失败");
            return;
        }
        m_camera2Opened = true;
        emit camera2OpenedChanged();

        const auto audit1 = applyAudit(*m_camera, false);
        const auto audit2 = applyAudit(*m_camera2, true);
        if (!audit1.serial.empty() && audit1.serial == audit2.serial) {
            releaseCameraPair();
            setStatus("相机 1 与相机 2 指向同一台设备（序列号相同）");
            return;
        }
        if (m_activePreviewCamera != 1) {
            m_activePreviewCamera = 1;
            emit activePreviewCameraChanged();
        }
        setCamera2Status("相机 2 已连接，等待首帧探活");
        setStatus("双相机已打开");
        updateDualCameraReady();
        APP_LOG_EVENT_INFO("camera", "camera_open", "dual_success",
            {{"elapsed_ms", QString::number(timer.elapsed())},
             {"serial1", QString::fromStdString(audit1.serial)},
             {"serial2", QString::fromStdString(audit2.serial)}});
    } catch (const std::exception& e) {
        APP_LOG_EVENT_ERROR("camera", "camera_open", "fail",
            {{"dual", "true"}, {"type", "std"}, {"message", QString::fromUtf8(e.what())},
             {"elapsed_ms", QString::number(timer.elapsed())}});
        releaseCameraPair();
        setStatus(QString("打开双相机异常: %1").arg(QString::fromUtf8(e.what())));
    } catch (...) {
        APP_LOG_EVENT_ERROR("camera", "camera_open", "fail",
            {{"dual", "true"}, {"type", "unknown"}, {"elapsed_ms", QString::number(timer.elapsed())}});
        releaseCameraPair();
        setStatus("打开双相机发生未知异常");
    }
}

void CollectionController::closeCamera()
{
    APP_LOG_EVENT_INFO("camera", "camera_close", "start",
        {{"opened", QString::number(m_cameraOpened)}, {"camera2", QString::number(m_camera2Opened)}});
    if (m_collectionBusy) {
        APP_LOG_EVENT_WARNING("camera", "camera_close", "skip", {{"reason", "collection_busy"}});
        setStatus("请先停止采集操作并等待结束，再关闭相机");
        return;
    }
    stopPreview();
    releaseCameraPair();
    if (m_undistortionMaps || m_undistortionMaps2) {
        m_undistortionMaps.reset();
        m_undistortionMaps2.reset();
        emit collectionChanged();
    }
    setStatus(m_dualCameraMode ? "双相机已断开" : "相机关闭");
    APP_LOG_EVENT_INFO("camera", "camera_close", "complete", {{"opened", "false"}});
}

void CollectionController::startPreview()
{
    if (m_collectionBusy) return;
    if (m_previewActive) {
        APP_LOG_EVENT_INFO("preview", "preview_start", "skip", {{"reason", "already_active"}});
        return;
    }
    if (!m_camera || !m_cameraOpened) {
        APP_LOG_EVENT_WARNING("preview", "preview_start", "skip",
            {{"reason", "camera_not_ready"},
             {"camera_instance", QString::number(static_cast<bool>(m_camera))},
             {"camera_opened", QString::number(m_cameraOpened)}});
        setStatus("请先打开相机");
        return;
    }
    if (m_dualCameraMode && (!m_camera2 || !m_camera2Opened)) {
        APP_LOG_EVENT_WARNING("preview", "preview_start", "skip",
            {{"reason", "camera2_not_ready"}});
        setStatus("请先打开相机 2");
        return;
    }

    try {
        APP_LOG_EVENT_INFO("preview", "preview_start", "start",
            {{"interval_ms", QString::number(m_previewTimer->interval())},
             {"dual", m_dualCameraMode ? "true" : "false"}});
        m_previewFrameCount = 0;
        m_previewEmptyCount = 0;
        m_healthProbeFailCount = 0;
        m_switchDiscardPending = false;
        m_lastHealthProbeTimer.invalidate();
        m_camera1FaultReported = false;
        m_camera2FaultReported = false;
        setCameraHealth(1, false);
        setCameraHealth(2, false);
        ++m_previewGeneration;
        m_previewActive = true;
        updateDualCameraReady();
        updatePreviewTimer();
        emit previewActiveChanged();
        if (m_dualCameraMode) setCamera2Status("相机 2 探活中…");
        setStatus(m_dualCameraMode
            ? "双路预览中（Tab 一键切换画面）— 空格或 Ctrl+S 双路拍照"
            : "预览中 — 空格或 Ctrl+S 拍照，自动融合多曝光");
        APP_LOG_EVENT_INFO("preview", "preview_start", "success", {});
    } catch (const std::exception& e) {
        APP_LOG_EVENT_ERROR("preview", "preview_start", "fail",
            {{"type", "std"}, {"message", QString::fromUtf8(e.what())}});
        setStatus(QString("启动预览异常: %1").arg(QString::fromUtf8(e.what())));
    } catch (...) {
        APP_LOG_EVENT_ERROR("preview", "preview_start", "fail", {{"type", "unknown"}});
        setStatus("启动预览发生未知异常");
    }
}

void CollectionController::stopPreview()
{
    const bool wasActive = m_previewActive;
    ++m_previewGeneration;
    m_previewActive = false;
    updateDualCameraReady();
    updatePreviewTimer();
    emit previewActiveChanged();
    if (wasActive) {
        APP_LOG_EVENT_INFO("preview", "preview_stop", "success",
            {{"frames", QString::number(m_previewFrameCount)},
             {"empty_frames", QString::number(m_previewEmptyCount)}});
    }
}

void CollectionController::setPreviewVisible(bool visible)
{
    if (m_previewVisible == visible) return;
    ++m_previewGeneration;
    m_previewVisible = visible;
    updatePreviewTimer();
    emit previewVisibleChanged();
}

void CollectionController::setPreviewDisplaySize(const QSize& size)
{
    if (size.isEmpty() || size == m_previewDisplaySize) return;
    m_previewDisplaySize = size;
    ++m_previewGeneration;
    emit previewDisplaySizeChanged();
}

bool CollectionController::previewFrameGrabAllowed() const
{
    // Hidden pages and a user pause stay off. Exposure grabs are the only
    // capture window that takes the camera; motion, settling, fusion and
    // saving keep the live image moving.
    if (!m_previewVisible || !m_previewActive || !m_cameraOpened || m_exposureCaptureActive)
        return false;
    if (m_dualCameraMode && !m_camera2Opened)
        return false;
    return !m_previewHeld || m_manualMoveActive;
}

void CollectionController::updatePreviewTimer()
{
    if (previewFrameGrabAllowed()) {
        if (!m_previewTimer->isActive()) m_previewTimer->start();
    } else {
        m_previewTimer->stop();
    }
}

void CollectionController::suspendPreviewForExposure()
{
    std::unique_lock<std::mutex> lock(m_exposurePreviewMutex);
    if (m_exposurePreviewClosing) throw std::runtime_error("拍照已取消");
    m_exposurePreviewReady = false;
    const bool posted = QMetaObject::invokeMethod(this, [this] {
        std::lock_guard<std::mutex> lock(m_exposurePreviewMutex);
        m_exposureCaptureActive = true;
        ++m_previewGeneration;
        updatePreviewTimer();
        m_exposurePreviewReady = true;
        m_exposurePreviewChanged.notify_one();
    }, Qt::QueuedConnection);
    if (!posted) throw std::runtime_error("无法暂停预览");
    // Destruction can wake the worker after the GUI event loop has stopped.
    m_exposurePreviewChanged.wait(lock, [this] {
        return m_exposurePreviewReady || m_exposurePreviewClosing;
    });
    if (m_exposurePreviewClosing) throw std::runtime_error("拍照已取消");
    const auto pendingPreview = m_previewFuture;
    const auto pendingProbe = m_healthProbeFuture;
    lock.unlock();
    if (pendingPreview.valid()) pendingPreview.wait();
    if (pendingProbe.valid()) pendingProbe.wait();
}

void CollectionController::resumePreviewAfterExposure()
{
    // Resuming the UI is independent of the worker's remaining work and shutdown.
    QMetaObject::invokeMethod(this, [this] {
        m_exposureCaptureActive = false;
        updatePreviewTimer();
        if (m_previewTimer->isActive()) onPreviewTick();
    }, Qt::QueuedConnection);
}

void CollectionController::onPreviewTick()
{
    if (!previewFrameGrabAllowed() || !m_camera) return;
    if (m_dualCameraMode && !m_camera2) return;
    const int activeCam = m_dualCameraMode ? m_activePreviewCamera : 1;
    const int inactiveCam = (activeCam == 1) ? 2 : 1;
    auto reportFault = [this](int camIdx, bool clearPreview) {
        bool& reported = (camIdx == 1) ? m_camera1FaultReported : m_camera2FaultReported;
        if (!reported) {
            reported = true;
            if (clearPreview) {
                if (m_imageProvider) m_imageProvider->clearImage();
                emit frameCleared();
            }
            if (camIdx == 2) setCamera2Status("相机 2 取帧持续失败");
            setStatus(QString("相机 %1 取帧持续失败，已禁用双路拍照，请重连").arg(camIdx));
        }
    };
    try {
        if (m_previewFuture.valid() &&
            m_previewFuture.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            auto completed = std::move(m_previewFuture);
            // Navigation, settings, channel switches and capture invalidate stale frames.
            if (m_pendingPreviewGeneration == m_previewGeneration) {
                const auto& frame = completed.get();
                if (frame.image.isNull()) {
                    ++m_previewEmptyCount;
                    if (m_previewEmptyCount == 1 || m_previewEmptyCount % 150 == 0)
                        APP_LOG_EVENT_WARNING("preview", "preview_frame", "empty",
                            {{"camera", QString::number(activeCam)},
                             {"empty_count", QString::number(m_previewEmptyCount)},
                             {"successful_frames", QString::number(m_previewFrameCount)}});
                    if (m_dualCameraMode && m_previewEmptyCount >= 30) {
                        setCameraHealth(activeCam, false);
                        reportFault(activeCam, true);
                    }
                } else {
                    m_previewEmptyCount = 0;
                    ++m_previewFrameCount;
                    if (m_previewFrameCount == 1 || m_previewFrameCount % 150 == 0)
                        APP_LOG_EVENT_DEBUG("preview", "preview_frame", "heartbeat",
                            {{"camera", QString::number(activeCam)},
                             {"frame", QString::number(m_previewFrameCount)},
                             {"empty_frames", QString::number(m_previewEmptyCount)},
                             {"size", QString("%1x%2").arg(frame.originalSize.width()).arg(frame.originalSize.height())},
                             {"display_size", QString("%1x%2").arg(frame.image.width()).arg(frame.image.height())},
                             {"channels", QString::number(frame.channels)}});
                    if (activeCam == 1) m_camera1FaultReported = false;
                    else {
                        m_camera2FaultReported = false;
                        setCamera2Status("相机 2 预览中");
                    }
                    if (m_imageProvider) m_imageProvider->setImage(frame.image);
                    emit frameUpdated();
                    setCameraHealth(activeCam, true);
                }
            } else {
                try { completed.get(); }
                catch (const std::exception& error) {
                    APP_LOG_EVENT_WARNING("preview", "preview_frame", "discarded_error", {{"message", QString::fromUtf8(error.what())}});
                } catch (...) {
                    APP_LOG_EVENT_WARNING("preview", "preview_frame", "discarded_error", {{"type", "unknown"}});
                }
            }
        }

        if (m_dualCameraMode && m_healthProbeFuture.valid() &&
            m_healthProbeFuture.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            auto completedProbe = std::move(m_healthProbeFuture);
            if (m_pendingHealthProbeGeneration == m_previewGeneration) {
                bool ok = false;
                try { ok = completedProbe.get(); }
                catch (...) { ok = false; }
                const int probedCam = m_pendingHealthProbeCamera;
                if (ok) {
                    m_healthProbeFailCount = 0;
                    if (probedCam == 1) m_camera1FaultReported = false;
                    else {
                        m_camera2FaultReported = false;
                        setCamera2Status("相机 2 就绪");
                    }
                    setCameraHealth(probedCam, true);
                } else {
                    ++m_healthProbeFailCount;
                    const bool currentlyHealthy = (probedCam == 1) ? m_camera1Healthy : m_camera2Healthy;
                    const int threshold = currentlyHealthy ? 3 : 30;
                    if (m_healthProbeFailCount >= threshold) {
                        setCameraHealth(probedCam, false);
                        reportFault(probedCam, false);
                    }
                }
                m_lastHealthProbeTimer.restart();
            } else {
                try { completedProbe.get(); } catch (...) {}
            }
        }

        if (!previewFrameGrabAllowed()) return;

        if (!m_previewFuture.valid() &&
            (!m_healthProbeFuture.valid() || m_pendingHealthProbeCamera != activeCam)) {
            const auto camera = (activeCam == 2) ? m_camera2 : m_camera;
            const auto maps = (activeCam == 2) ? m_undistortionMaps2 : m_undistortionMaps;
            const auto displaySize = m_previewDisplaySize;
            const bool flushStale = m_switchDiscardPending;
            m_switchDiscardPending = false;
            m_pendingPreviewGeneration = m_previewGeneration;
            // One frame at a time: no worker queue and no device/pixel work on the GUI thread.
            m_previewFuture = std::async(std::launch::async, [camera, maps, displaySize, flushStale] {
                cv::Mat bgr = camera->captureOneFrame(10);
                if (flushStale) {
                    cv::Mat fresh = camera->captureOneFrame(10);
                    if (!fresh.empty()) bgr = std::move(fresh);
                }
                if (bgr.empty()) return PreviewFrame{};
                if (maps) bgr = maps->apply(bgr);
                return PreviewFrame{makePreviewImage(bgr, displaySize), QSize(bgr.cols, bgr.rows), bgr.channels()};
            }).share();
        }

        if (m_dualCameraMode && !m_healthProbeFuture.valid()) {
            const bool inactiveHealthy = (inactiveCam == 1) ? m_camera1Healthy : m_camera2Healthy;
            const bool due = !inactiveHealthy || !m_lastHealthProbeTimer.isValid() ||
                             m_lastHealthProbeTimer.elapsed() >= 2500;
            if (due) {
                const auto probeCamera = (inactiveCam == 2) ? m_camera2 : m_camera;
                m_pendingHealthProbeGeneration = m_previewGeneration;
                m_pendingHealthProbeCamera = inactiveCam;
                m_healthProbeFuture = std::async(std::launch::async, [probeCamera] {
                    return !probeCamera->captureOneFrame(10).empty();
                }).share();
            }
        }
    } catch (const std::exception& error) {
        APP_LOG_EVENT_ERROR("preview", "preview_tick", "exception",
            {{"message", QString::fromUtf8(error.what())}, {"frame", QString::number(m_previewFrameCount)}});
        stopPreview();
        setStatus(QString("预览异常: %1").arg(QString::fromUtf8(error.what())));
    } catch (...) {
        APP_LOG_EVENT_ERROR("preview", "preview_tick", "exception", {{"type", "unknown"}});
        stopPreview();
        setStatus("预览发生未知异常");
    }
}

QImage CollectionController::makePreviewImage(const cv::Mat& bgr, const QSize& displaySize)
{
    cv::Mat pixels = bgr;
    const QSize fitted = QSize(bgr.cols, bgr.rows).scaled(displaySize, Qt::KeepAspectRatio);
    if (fitted.width() < bgr.cols && fitted.height() < bgr.rows)
        cv::resize(bgr, pixels, cv::Size(std::max(1, fitted.width()), std::max(1, fitted.height())), 0, 0, cv::INTER_AREA);
    cv::Mat converted;
    QImage::Format format;
    if (pixels.channels() == 3) {
        cv::cvtColor(pixels, converted, cv::COLOR_BGR2RGB);
        pixels = converted;
        format = QImage::Format_RGB888;
    } else if (pixels.channels() == 1) {
        format = QImage::Format_Grayscale8;
    } else if (pixels.channels() == 4) {
        cv::cvtColor(pixels, converted, cv::COLOR_BGRA2RGBA);
        pixels = converted;
        format = QImage::Format_RGBA8888;
    } else {
        throw std::runtime_error("预览图像通道数无效");
    }
    // Own the pixels before the worker's cv::Mat is released. Saved images keep full resolution.
    return QImage(pixels.data, pixels.cols, pixels.rows, static_cast<int>(pixels.step), format).copy();
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
