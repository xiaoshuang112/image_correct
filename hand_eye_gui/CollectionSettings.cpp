#include "CollectionController.hpp"
#include "CollectionCapture.hpp"
#include "DiagnosticLogger.hpp"
#include "ExposureList.hpp"
#include "config_resolver.hpp"

#include <QDesktopServices>
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

QString CollectionController::applySettings(const QVariantMap& values) {
    const auto fail = [this](const QString& message) {
        APP_LOG_EVENT_WARNING("config", "settings_apply", "failed", {{"reason", message}});
        setStatus(message);
        return message;
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
    if (updates.empty()) return {};

    int actualExposure = requestedExposure, previousDevice = -1;
    bool previousAuto = false, deviceTouched = false;
    const bool cameraReady = m_cameraOpened && bool(m_camera);
    const auto restoreDevice = [&] {
        if (!deviceTouched) return QString{};
        bool restored = m_camera->setColorExposureTime(previousDevice);
        if (previousAuto) restored = m_camera->setAutoExposure(true) && restored;
        const int actual = m_camera->getColorExposureTime();
        restored = restored && actual > 0 && (previousAuto || actual == previousDevice);
        m_cameraActualExposureUs = actual > 0 ? actual : -1;
        emit cameraActualExposureUsChanged();
        return restored ? QString("；相机已恢复原曝光") : QString("；恢复相机曝光失败，请检查设备后重试");
    };
    try {
        if (cameraReady && updates.count("fusion_exposures_us"))
            collection::checkCaptureExposures(*m_camera, exposures);
        if (cameraReady && exposureChanged) {
            collection::checkCaptureExposures(*m_camera, {requestedExposure});
            previousDevice = m_camera->getColorExposureTime();
            if (previousDevice <= 0) return fail("相机曝光回读失败，未修改设置");
            previousAuto = m_camera->getAutoExposure();
            deviceTouched = true;
            if (previousAuto && !m_camera->setAutoExposure(false))
                return fail("关闭自动曝光失败" + restoreDevice());
            if (!m_camera->setColorExposureTime(requestedExposure))
                return fail("相机曝光更新失败" + restoreDevice());
            actualExposure = m_camera->getColorExposureTime();
            if (actualExposure <= 0 || actualExposure > 10000000)
                return fail("相机曝光回读失败" + restoreDevice());
            updates["camera_exposure_us"] = actualExposure;
        }
        std::string error;
        if (!config_resolver::updateFlatConfigFile(m_appPath + "config.yaml", updates, error))
            return fail("设置保存失败：" + QString::fromStdString(error) + restoreDevice());
    } catch (const std::exception& error) {
        QString restore;
        try { restore = restoreDevice(); }
        catch (...) { restore = "；恢复相机曝光异常，请检查设备"; }
        return fail("设置保存失败：" + QString::fromUtf8(error.what()) + restore);
    }

    if (m_cameraIp != cameraIp) { m_cameraIp = cameraIp; emit cameraIpChanged(); }
    if (m_robotIp != robotIp) { m_robotIp = robotIp; emit robotIpChanged(); }
    if (m_robotSpeedMmS != robotSpeed) { m_robotSpeedMmS = robotSpeed; emit collectionChanged(); }
    if (m_previewWidth != width) { m_previewWidth = width; emit previewWidthChanged(); }
    if (m_previewHeight != height) { m_previewHeight = height; emit previewHeightChanged(); }
    if (m_cameraExposureUs != actualExposure) {
        m_cameraExposureUs = actualExposure;
        emit cameraExposureUsChanged();
    }
    if (exposureChanged && cameraReady) {
        m_cameraActualExposureUs = actualExposure;
        emit cameraActualExposureUsChanged();
    }
    if (m_exposureList != exposureList) { m_exposureList = exposureList; emit collectionChanged(); }
    APP_LOG_EVENT_INFO("config", "settings_apply", "saved",
        {{"fields", QString::number(updates.size())}, {"path", QString::fromStdString(m_appPath) + "config.yaml"}});
    setStatus("设置已保存" + (exposureChanged && cameraReady
        ? QString("，相机实际曝光 %1 μs").arg(actualExposure) : QString{}));
    return {};
}

void CollectionController::openDiagnosticFolder() {
    const QString directory = QFileInfo(diagnosticLogPath()).absolutePath();
    if (!QDesktopServices::openUrl(QUrl::fromLocalFile(directory)))
        setStatus("无法打开诊断日志目录：" + directory);
}
