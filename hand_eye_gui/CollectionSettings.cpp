#include "CollectionController.hpp"
#include "CollectionCapture.hpp"
#include "CollectionPoseRecord.hpp"
#include "DiagnosticLogger.hpp"
#include "ExposureList.hpp"
#include "config_resolver.hpp"

#include <QDesktopServices>
#include <QFileDialog>
#include <QApplication>
#include <QWindow>
#include <QFileInfo>
#include <QRegularExpression>
#include <QUrl>

namespace {
bool validIp(const QString& value) {
    const auto parts = value.split('.');
    if (parts.size() != 4) return false;
    static const QRegularExpression digits(QStringLiteral("^[0-9]{1,3}$"));
    for (const auto& part : parts)
        if (!digits.match(part).hasMatch() || part.toInt() > 255) return false;
    return true;
}
}

void CollectionController::applySettings(const QVariantMap& values) {
    if (m_settingsSaving) return;
    const auto fail = [this](const QString& message) {
        APP_LOG_EVENT_WARNING("config", "settings_apply", "failed", {{"reason", message}});
        setStatus(message);
        emit settingsFinished(message);
    };
    if (m_collectionBusy || m_robotStopUnconfirmed)
        return fail("操作进行中，设置已锁定");
    if (m_appPath.empty()) return fail("配置目录尚未初始化，无法保存设置");
    const auto text = [&values](const QString& key, const QString& fallback) {
        return values.value(key, fallback).toString().trimmed();
    };
    const auto integer = [&text](const QString& key, int fallback, int maximum) {
        bool ok = false;
        const int value = text(key, QString::number(fallback)).toInt(&ok);
        return ok && value > 0 && value <= maximum ? value : -1;
    };
    const auto nonNegative = [&text](const QString& key, int fallback) {
        bool ok = false;
        const int value = text(key, QString::number(fallback)).toInt(&ok);
        return ok && value >= 0 ? value : -1;
    };
    const QString cameraIp = text("cameraIp", m_cameraIp);
    const QString camera2Ip = text("camera2Ip", m_camera2Ip);
    const QString camera2Sn = text("camera2Sn", m_camera2Sn);
    const QString robotIp = text("robotIp", m_robotIp);
    const int robotSpeed = integer("robotSpeedMmS", m_robotSpeedMmS, collection::maximumRobotSpeedMmS);
    const int width = integer("previewWidth", m_previewWidth, 32768);
    const int height = integer("previewHeight", m_previewHeight, 32768);
    const int width2 = integer("preview2Width", m_preview2Width, 32768);
    const int height2 = integer("preview2Height", m_preview2Height, 32768);
    const int requestedExposure = integer("cameraExposureUs", m_cameraExposureUs, 10000000);
    const int requestedExposure2 = integer("camera2ExposureUs", m_camera2ExposureUs, 10000000);
    const int requestedGain2 = nonNegative("camera2Gain", m_camera2Gain);
    if (!validIp(cameraIp)) return fail("相机 1 IP 格式无效，请输入完整 IPv4 地址");
    if (!camera2Ip.isEmpty() && !validIp(camera2Ip))
        return fail("相机 2 IP 格式无效，请输入完整 IPv4 地址或留空");
    if (!camera2Ip.isEmpty() && !cameraIp.isEmpty() && camera2Ip == cameraIp)
        return fail("相机 1 与相机 2 的 IP 不能相同");
    if (!validIp(robotIp)) return fail("机器人 IP 格式无效，请输入完整 IPv4 地址");
    if (!collection::validRobotSpeed(robotSpeed)) return fail("移动速度须为 1～1000 mm/s 范围内的整数");
    if (width < 0 || height < 0) return fail("相机 1 采集宽、高须为 1～32768 范围内的整数");
    if (width2 < 0 || height2 < 0) return fail("相机 2 采集宽、高须为 1～32768 范围内的整数");
    if (requestedExposure < 0) return fail("相机 1 预览曝光须为 1～10000000 μs 范围内的整数");
    if (requestedExposure2 < 0) return fail("相机 2 预览曝光须为 1～10000000 μs 范围内的整数");
    if (requestedGain2 < 0) return fail("相机 2 增益须为非负整数");
    std::vector<int> exposures, exposures2;
    const auto exposureError = collection::parseExposureList(text("exposureList", m_exposureList), exposures);
    if (exposureError == collection::ExposureListError::InvalidValue)
        return fail("相机 1 融合曝光应为不同的正整数（1～1000000 μs），未写入配置");
    if (exposureError == collection::ExposureListError::InvalidCount)
        return fail("相机 1 融合曝光应包含 2～5 个数值，未写入配置");
    const auto exposure2Error = collection::parseExposureList(text("exposure2List", m_exposure2List), exposures2);
    if (exposure2Error == collection::ExposureListError::InvalidValue)
        return fail("相机 2 融合曝光应为不同的正整数（1～1000000 μs），未写入配置");
    if (exposure2Error == collection::ExposureListError::InvalidCount)
        return fail("相机 2 融合曝光应包含 2～5 个数值，未写入配置");
    if (m_cameraOpened && (cameraIp != m_cameraIp || width != m_previewWidth || height != m_previewHeight))
        return fail("请先断开相机 1，再修改 IP 或分辨率");
    if (m_camera2Opened &&
        (camera2Ip != m_camera2Ip || camera2Sn != m_camera2Sn || width2 != m_preview2Width ||
         height2 != m_preview2Height || requestedGain2 != m_camera2Gain))
        return fail("请先断开相机 2，再修改 IP、序列号、分辨率或增益");
    if (m_robotConnected && robotIp != m_robotIp)
        return fail("请先断开机器人，再修改 IP");

    QStringList exposureStrings, exposure2Strings;
    for (int exposure : exposures) exposureStrings << QString::number(exposure);
    for (int exposure : exposures2) exposure2Strings << QString::number(exposure);
    const QString exposureList = exposureStrings.join(',');
    const QString exposure2List = exposure2Strings.join(',');
    config_resolver::ConfigUpdates updates;
    if (cameraIp != m_cameraIp) updates["cam_ip"] = cameraIp.toStdString();
    if (camera2Ip != m_camera2Ip) updates["cam2_ip"] = camera2Ip.toStdString();
    if (camera2Sn != m_camera2Sn) updates["cam2_sn"] = camera2Sn.toStdString();
    if (robotIp != m_robotIp) updates["robot_ip"] = robotIp.toStdString();
    if (robotSpeed != m_robotSpeedMmS) updates["robot_speed_mm_s"] = robotSpeed;
    if (width != m_previewWidth) updates["cam_width"] = width;
    if (height != m_previewHeight) updates["cam_height"] = height;
    if (width2 != m_preview2Width) updates["cam2_width"] = width2;
    if (height2 != m_preview2Height) updates["cam2_height"] = height2;
    if (exposureList != m_exposureList) updates["fusion_exposures_us"] = exposures;
    if (exposure2List != m_exposure2List) updates["camera2_fusion_exposures_us"] = exposures2;
    const bool exposureChanged = requestedExposure != m_cameraExposureUs;
    const bool exposure2Changed = requestedExposure2 != m_camera2ExposureUs;
    if (exposureChanged) updates["camera_exposure_us"] = requestedExposure;
    if (exposure2Changed) updates["camera2_exposure_us"] = requestedExposure2;
    if (requestedGain2 != m_camera2Gain) updates["camera2_gain"] = requestedGain2;
    if (updates.empty()) { emit settingsFinished({}); return; }

    const auto camera = m_cameraOpened ? m_camera : nullptr;
    const auto camera2 = m_camera2Opened ? m_camera2 : nullptr;
    const auto pendingPreview = m_previewFuture;
    const auto pendingHealthProbe = m_healthProbeFuture;
    const auto path = m_appPath + "config.yaml";
    m_settingsSaving = true;
    beginCollectionOperation();
    setStatus("正在保存配置…");
    try {
        m_collectionThread = std::thread([this, camera, camera2, pendingPreview, pendingHealthProbe, path, updates,
            exposures, exposures2, exposureChanged, exposure2Changed, requestedExposure, requestedExposure2,
            requestedGain2, cameraIp, camera2Ip, camera2Sn, robotIp, robotSpeed, width, height, width2, height2,
            exposureList, exposure2List]() mutable {
            // Serialize device access behind the last preview/probe, without blocking the GUI.
            if (pendingPreview.valid()) pendingPreview.wait();
            if (pendingHealthProbe.valid()) pendingHealthProbe.wait();
            QString error;
            int actualExposure = requestedExposure, deviceExposure = -1, previousDevice = -1;
            int actualExposure2 = requestedExposure2, deviceExposure2 = -1, previousDevice2 = -1;
            bool previousAuto = false, previousAuto2 = false, deviceTouched = false, device2Touched = false;
            auto applyDeviceExposure = [](const std::shared_ptr<AICamera>& device, int label, int requested,
                                          int& previousOut, bool& autoOut, bool& touchedOut,
                                          int& actualOut, int& readbackOut) {
                collection::checkCaptureExposures(*device, {requested});
                previousOut = device->getColorExposureTime();
                if (previousOut <= 0)
                    throw std::runtime_error(QString("相机 %1 曝光回读失败，未修改设置").arg(label).toStdString());
                autoOut = device->getAutoExposure();
                touchedOut = true;
                if (autoOut && !device->setAutoExposure(false))
                    throw std::runtime_error(QString("关闭相机 %1 自动曝光失败").arg(label).toStdString());
                if (!device->setColorExposureTime(requested))
                    throw std::runtime_error(QString("相机 %1 曝光更新失败").arg(label).toStdString());
                actualOut = readbackOut = device->getColorExposureTime();
                if (actualOut <= 0 || actualOut > 10000000)
                    throw std::runtime_error(QString("相机 %1 曝光回读失败").arg(label).toStdString());
            };
            auto restoreDevice = [](const std::shared_ptr<AICamera>& device, int label, int previous,
                                    bool autoMode, int& readback) {
                readback = -1;
                try {
                    bool restored = device->setColorExposureTime(previous);
                    if (autoMode) restored = device->setAutoExposure(true) && restored;
                    readback = device->getColorExposureTime();
                    restored = restored && readback > 0 && (autoMode || readback == previous);
                    return restored
                        ? QStringLiteral("；相机 %1 已恢复原曝光").arg(label)
                        : QStringLiteral("；恢复相机 %1 曝光失败，请检查设备后重试").arg(label);
                } catch (...) {
                    return QStringLiteral("；恢复相机 %1 曝光异常，请检查设备").arg(label);
                }
            };
            try {
                if (camera && updates.count("fusion_exposures_us"))
                    collection::checkCaptureExposures(*camera, exposures);
                if (camera2 && updates.count("camera2_fusion_exposures_us"))
                    collection::checkCaptureExposures(*camera2, exposures2);
                if (camera && exposureChanged) {
                    applyDeviceExposure(camera, 1, requestedExposure, previousDevice, previousAuto,
                                        deviceTouched, actualExposure, deviceExposure);
                    updates["camera_exposure_us"] = actualExposure;
                }
                if (camera2 && exposure2Changed) {
                    applyDeviceExposure(camera2, 2, requestedExposure2, previousDevice2, previousAuto2,
                                        device2Touched, actualExposure2, deviceExposure2);
                    updates["camera2_exposure_us"] = actualExposure2;
                }
                std::string detail;
                if (!config_resolver::updateFlatConfigFile(path, updates, detail))
                    throw std::runtime_error(detail);
            } catch (const std::exception& failure) {
                error = "设置保存失败：" + QString::fromUtf8(failure.what());
            } catch (...) {
                error = "设置保存发生未知异常";
            }
            if (!error.isEmpty()) {
                if (deviceTouched && camera)
                    error += restoreDevice(camera, 1, previousDevice, previousAuto, deviceExposure);
                if (device2Touched && camera2)
                    error += restoreDevice(camera2, 2, previousDevice2, previousAuto2, deviceExposure2);
            }
            QMetaObject::invokeMethod(this, [this, error, actualExposure, actualExposure2,
                deviceExposure, deviceExposure2, deviceTouched, device2Touched,
                cameraIp, camera2Ip, camera2Sn, robotIp, robotSpeed, width, height, width2, height2,
                requestedGain2, exposureList, exposure2List, path, fields = updates.size()] {
                if (deviceTouched) {
                    m_cameraActualExposureUs = deviceExposure > 0 ? deviceExposure : -1;
                    emit cameraActualExposureUsChanged();
                }
                if (device2Touched) {
                    m_camera2ActualExposureUs = deviceExposure2 > 0 ? deviceExposure2 : -1;
                    emit camera2ActualExposureUsChanged();
                }
                if (error.isEmpty()) {
                    if (m_cameraIp != cameraIp) { m_cameraIp = cameraIp; emit cameraIpChanged(); }
                    if (m_camera2Ip != camera2Ip) { m_camera2Ip = camera2Ip; emit camera2IpChanged(); }
                    if (m_camera2Sn != camera2Sn) { m_camera2Sn = camera2Sn; emit camera2SnChanged(); }
                    if (m_robotIp != robotIp) { m_robotIp = robotIp; emit robotIpChanged(); }
                    m_robotSpeedMmS = robotSpeed;
                    if (m_previewWidth != width) { m_previewWidth = width; emit previewWidthChanged(); }
                    if (m_previewHeight != height) { m_previewHeight = height; emit previewHeightChanged(); }
                    if (m_preview2Width != width2) { m_preview2Width = width2; emit preview2WidthChanged(); }
                    if (m_preview2Height != height2) { m_preview2Height = height2; emit preview2HeightChanged(); }
                    if (m_cameraExposureUs != actualExposure) {
                        m_cameraExposureUs = actualExposure;
                        emit cameraExposureUsChanged();
                    }
                    if (m_camera2ExposureUs != actualExposure2) {
                        m_camera2ExposureUs = actualExposure2;
                        emit camera2ExposureUsChanged();
                    }
                    if (m_camera2Gain != requestedGain2) {
                        m_camera2Gain = requestedGain2;
                        emit camera2GainChanged();
                    }
                    m_exposureList = exposureList;
                    m_exposure2List = exposure2List;
                    emit collectionChanged();
                    APP_LOG_EVENT_INFO("config", "settings_apply", "saved",
                        {{"fields", QString::number(fields)}, {"path", QString::fromStdString(path)}});
                } else APP_LOG_EVENT_WARNING("config", "settings_apply", "failed", {{"reason", error}});
                m_settingsSaving = false;
                QString okDetail;
                if (deviceTouched)
                    okDetail += QString("，相机 1 实际曝光 %1 μs").arg(actualExposure);
                if (device2Touched)
                    okDetail += QString("，相机 2 实际曝光 %1 μs").arg(actualExposure2);
                finishCollectionOperation(error.isEmpty() ? "设置已保存" + okDetail : error);
                emit settingsFinished(error);
            }, Qt::QueuedConnection);
        });
    } catch (const std::exception& error) {
        m_settingsSaving = false;
        const QString message = "无法启动配置保存：" + QString::fromUtf8(error.what());
        finishCollectionOperation(message);
        fail(message);
    }
}

void CollectionController::openDiagnosticFolder() {
    const QString directory = QFileInfo(diagnosticLogPath()).absolutePath();
    if (!QDesktopServices::openUrl(QUrl::fromLocalFile(directory)))
        setStatus("无法打开诊断日志目录：" + directory);
}

void CollectionController::chooseTcpCoordinateFile() {
    if (m_collectionBusy) return;
    if (auto* current = qobject_cast<QFileDialog*>(m_coordinateDialog.data())) {
        current->raise(); current->activateWindow(); return;
    }
    auto* dialog = new QFileDialog(nullptr, "导入坐标 JSON", m_outputDirectory, "JSON 文件 (*.json)");
    m_coordinateDialog = dialog;
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    dialog->setFileMode(QFileDialog::ExistingFile);
    dialog->setWindowModality(Qt::ApplicationModal);
    auto* parentWindow = QGuiApplication::focusWindow();
    dialog->winId();
    if (parentWindow) dialog->windowHandle()->setTransientParent(parentWindow);
    connect(dialog, &QFileDialog::fileSelected, this, [this](const QString& path) {
        importTcpCoordinates(QUrl::fromLocalFile(path));
    });
    dialog->open(); // Nonblocking: no nested dialog event loop.
}

void CollectionController::importTcpCoordinates(const QUrl& url) {
    if (m_coordinatesImporting) return;
    if (m_collectionBusy || !url.isLocalFile()) {
        emit coordinatesImported({}, {}, m_collectionBusy ? "操作进行中，无法导入坐标" : "请选择本地 JSON 文件");
        return;
    }
    m_coordinatesImporting = true;
    beginCollectionOperation();
    setStatus("正在导入坐标…");
    try {
        m_collectionThread = std::thread([this, path = url.toLocalFile()] {
            QString coordinates, tool, error;
            try { error = collection::loadPoseCoordinates(path, coordinates, tool); }
            catch (const std::exception& failure) { error = "读取坐标失败：" + QString::fromUtf8(failure.what()); }
            catch (...) { error = "读取坐标发生未知异常"; }
            QMetaObject::invokeMethod(this, [this, coordinates, tool, error] {
                m_coordinatesImporting = false;
                finishCollectionOperation(error.isEmpty() ? "坐标已导入，请确认目标后手动执行移动" : error);
                emit coordinatesImported(coordinates, tool, error);
            }, Qt::QueuedConnection);
        });
    } catch (const std::exception& failure) {
        m_coordinatesImporting = false;
        const QString error = "无法启动坐标导入：" + QString::fromUtf8(failure.what());
        finishCollectionOperation(error);
        emit coordinatesImported({}, {}, error);
    }
}
