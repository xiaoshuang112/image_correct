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
    const QString cameraIp = text("cameraIp", m_cameraIp);
    const QString robotIp = text("robotIp", m_robotIp);
    const int robotSpeed = integer("robotSpeedMmS", m_robotSpeedMmS, collection::maximumRobotSpeedMmS);
    const int width = integer("previewWidth", m_previewWidth, 32768);
    const int height = integer("previewHeight", m_previewHeight, 32768);
    const int requestedExposure = integer("cameraExposureUs", m_cameraExposureUs, 10000000);
    if (!validIp(cameraIp)) return fail("相机 IP 格式无效，请输入完整 IPv4 地址");
    if (!validIp(robotIp)) return fail("机器人 IP 格式无效，请输入完整 IPv4 地址");
    if (!collection::validRobotSpeed(robotSpeed)) return fail("移动速度须为 1～1000 mm/s 范围内的整数");
    if (width < 0 || height < 0) return fail("采集宽、高须为 1～32768 范围内的整数");
    if (requestedExposure < 0) return fail("预览曝光须为 1～10000000 μs 范围内的整数");
    std::vector<int> exposures;
    const auto exposureError = collection::parseExposureList(text("exposureList", m_exposureList), exposures);
    if (exposureError == collection::ExposureListError::InvalidValue)
        return fail("融合曝光应为不同的正整数（1～1000000 μs），未写入配置");
    if (exposureError == collection::ExposureListError::InvalidCount)
        return fail("融合曝光应包含 2～5 个数值，未写入配置");
    if (m_cameraOpened && (cameraIp != m_cameraIp || width != m_previewWidth || height != m_previewHeight))
        return fail("请先断开相机，再修改 IP 或分辨率");
    if (m_robotConnected && robotIp != m_robotIp)
        return fail("请先断开机器人，再修改 IP");

    QStringList exposureStrings;
    for (int exposure : exposures) exposureStrings << QString::number(exposure);
    const QString exposureList = exposureStrings.join(',');
    config_resolver::ConfigUpdates updates;
    if (cameraIp != m_cameraIp) updates["cam_ip"] = cameraIp.toStdString();
    if (robotIp != m_robotIp) updates["robot_ip"] = robotIp.toStdString();
    if (robotSpeed != m_robotSpeedMmS) updates["robot_speed_mm_s"] = robotSpeed;
    if (width != m_previewWidth) updates["cam_width"] = width;
    if (height != m_previewHeight) updates["cam_height"] = height;
    if (exposureList != m_exposureList) updates["fusion_exposures_us"] = exposures;
    const bool exposureChanged = requestedExposure != m_cameraExposureUs;
    if (exposureChanged) updates["camera_exposure_us"] = requestedExposure;
    if (updates.empty()) { emit settingsFinished({}); return; }

    const auto camera = m_cameraOpened ? m_camera : nullptr;
    const auto pendingPreview = m_previewFuture;
    const auto path = m_appPath + "config.yaml";
    m_settingsSaving = true;
    beginCollectionOperation();
    setStatus("正在保存配置…");
    try {
        m_collectionThread = std::thread([this, camera, pendingPreview, path, updates, exposures,
            exposureChanged, requestedExposure, cameraIp, robotIp, robotSpeed, width, height, exposureList]() mutable {
            // Serialize device access behind the last preview, without blocking the GUI.
            if (pendingPreview.valid()) pendingPreview.wait();
            QString error;
            int actualExposure = requestedExposure, deviceExposure = -1, previousDevice = -1;
            bool previousAuto = false, deviceTouched = false;
            try {
                if (camera && updates.count("fusion_exposures_us"))
                    collection::checkCaptureExposures(*camera, exposures);
                if (camera && exposureChanged) {
                    collection::checkCaptureExposures(*camera, {requestedExposure});
                    previousDevice = camera->getColorExposureTime();
                    if (previousDevice <= 0) throw std::runtime_error("相机曝光回读失败，未修改设置");
                    previousAuto = camera->getAutoExposure();
                    deviceTouched = true;
                    if (previousAuto && !camera->setAutoExposure(false)) throw std::runtime_error("关闭自动曝光失败");
                    if (!camera->setColorExposureTime(requestedExposure)) throw std::runtime_error("相机曝光更新失败");
                    actualExposure = deviceExposure = camera->getColorExposureTime();
                    if (actualExposure <= 0 || actualExposure > 10000000) throw std::runtime_error("相机曝光回读失败");
                    updates["camera_exposure_us"] = actualExposure;
                }
                std::string detail;
                if (!config_resolver::updateFlatConfigFile(path, updates, detail))
                    throw std::runtime_error(detail);
            } catch (const std::exception& failure) {
                error = "设置保存失败：" + QString::fromUtf8(failure.what());
            } catch (...) {
                error = "设置保存发生未知异常";
            }
            if (!error.isEmpty() && deviceTouched) {
                deviceExposure = -1;
                try {
                    bool restored = camera->setColorExposureTime(previousDevice);
                    if (previousAuto) restored = camera->setAutoExposure(true) && restored;
                    deviceExposure = camera->getColorExposureTime();
                    restored = restored && deviceExposure > 0 && (previousAuto || deviceExposure == previousDevice);
                    error += restored ? "；相机已恢复原曝光" : "；恢复相机曝光失败，请检查设备后重试";
                } catch (...) { error += "；恢复相机曝光异常，请检查设备"; }
            }
            QMetaObject::invokeMethod(this, [this, error, actualExposure, deviceExposure, deviceTouched,
                cameraIp, robotIp, robotSpeed, width, height, exposureList, path, fields = updates.size()] {
                if (deviceTouched) {
                    m_cameraActualExposureUs = deviceExposure > 0 ? deviceExposure : -1;
                    emit cameraActualExposureUsChanged();
                }
                if (error.isEmpty()) {
                    if (m_cameraIp != cameraIp) { m_cameraIp = cameraIp; emit cameraIpChanged(); }
                    if (m_robotIp != robotIp) { m_robotIp = robotIp; emit robotIpChanged(); }
                    m_robotSpeedMmS = robotSpeed;
                    if (m_previewWidth != width) { m_previewWidth = width; emit previewWidthChanged(); }
                    if (m_previewHeight != height) { m_previewHeight = height; emit previewHeightChanged(); }
                    if (m_cameraExposureUs != actualExposure) {
                        m_cameraExposureUs = actualExposure;
                        emit cameraExposureUsChanged();
                    }
                    m_exposureList = exposureList;
                    APP_LOG_EVENT_INFO("config", "settings_apply", "saved",
                        {{"fields", QString::number(fields)}, {"path", QString::fromStdString(path)}});
                } else APP_LOG_EVENT_WARNING("config", "settings_apply", "failed", {{"reason", error}});
                m_settingsSaving = false;
                finishCollectionOperation(error.isEmpty() ? "设置已保存" + (deviceTouched
                    ? QString("，相机实际曝光 %1 μs").arg(actualExposure) : QString{}) : error);
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
