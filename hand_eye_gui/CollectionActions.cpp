#include "CollectionController.hpp"
#include "DiagnosticLogger.hpp"
#include "config_resolver.hpp"
#include "CollectionPoseRecord.hpp"
#include "CollectionCapture.hpp"
#include "NinePointCapture.hpp"
#include "RobotTcp.hpp"
#include "ExposureList.hpp"
#include "utils/collection_undistortion.hpp"
#include <QDateTime>
#include <QDesktopServices>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QUrl>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace {
struct ConfiguredPoint {
    int number;
    collection::FlangePose offset;
};

std::vector<ConfiguredPoint> readConfiguredOffsets(const std::string& paramPath) {
    QFile file(QDir(QString::fromStdString(paramPath)).filePath("move_tcp_points.json"));
    if (!file.exists()) throw std::runtime_error("未找到 param/move_tcp_points.json");
    if (!file.open(QIODevice::ReadOnly)) throw std::runtime_error("无法打开 move_tcp_points.json");
    constexpr qint64 maximumBytes = 64 * 1024;
    const auto bytes = file.read(maximumBytes + 1);
    if (file.error() != QFileDevice::NoError) throw std::runtime_error("无法读取 move_tcp_points.json");
    if (bytes.size() > maximumBytes) throw std::runtime_error("点位配置超过 64 KB");
    QJsonParseError error;
    const auto document = QJsonDocument::fromJson(bytes, &error);
    if (error.error != QJsonParseError::NoError || !document.isObject())
        throw std::runtime_error("点位配置须为 JSON 对象");
    const auto points = document.object();
    if (points.isEmpty()) throw std::runtime_error("点位配置为空");

    std::vector<ConfiguredPoint> offsets;
    for (auto point = points.begin(); point != points.end(); ++point) {
        const auto key = point.key();
        bool validNumber = false;
        const int number = key.mid(1).toInt(&validNumber);
        if (!validNumber || number < 1 || key != QString("p%1").arg(number))
            throw std::runtime_error(QString("无效点位名称 %1，应为 p1、p2 等").arg(key).toStdString());
        if (!point.value().isObject())
            throw std::runtime_error(QString("%1 必须是坐标对象").arg(key).toStdString());
        const auto values = point.value().toObject();
        const char* axes[] = {"x", "y", "z", "rx", "ry", "rz"};
        collection::FlangePose offset{};
        for (size_t axis = 0; axis < offset.size(); ++axis) {
            const auto value = values.value(QLatin1String(axes[axis]));
            if (!value.isDouble() || !std::isfinite(value.toDouble()))
                throw std::runtime_error(QString("%1 缺少有效的 %2 数值").arg(key, QLatin1String(axes[axis])).toStdString());
            offset[axis] = value.toDouble();
        }
        if (std::hypot(offset[0], offset[1], offset[2]) > collection::maximumNinePointOffsetMm)
            throw std::runtime_error(QString("%1 的 XYZ 偏移距基准超过 50 mm").arg(key).toStdString());
        if (std::any_of(offset.begin() + 3, offset.end(), [](double angle) {
            return std::abs(angle) > collection::maximumNinePointYawDegrees;
        })) throw std::runtime_error(QString("%1 的欧拉角偏移超出 ±20°").arg(key).toStdString());
        offsets.push_back({number, offset});
    }
    std::sort(offsets.begin(), offsets.end(),
        [](const auto& a, const auto& b) { return a.number < b.number; });
    return offsets;
}

collection::FlangePose configuredTarget(const collection::FlangePose& origin, const collection::FlangePose& offset) {
    auto target = origin;
    if (offset[3] == 0 && offset[4] == 0) {
        target = collection::withEulerYawOffset(origin, offset[5]);
    } else {
        auto euler = collection::tcpPoseForDisplay(origin, collection::TeachOrientation::EulerZYX);
        for (int axis = 3; axis < 6; ++axis) euler[axis] += offset[axis];
        target = collection::tcpPoseFromDisplay(euler, collection::TeachOrientation::EulerZYX);
    }
    for (int axis = 0; axis < 3; ++axis) target[axis] = origin[axis] + offset[axis];
    if (collection::flangeRotationDistanceDeg(origin, target) > collection::maximumNinePointYawDegrees + 1e-9)
        throw std::runtime_error("组合旋转相对基准超过 20°");
    return target;
}

std::array<double, 6> readFlangePose(RobotHandle handle) {
    std::array<double, 6> pose{}, zero{};
    const auto result = cr_get_AssignTCP_Pose(handle, zero.data(), pose.data());
    if (result != sucess)
        throw std::runtime_error(QString("法兰位姿读取失败（错误码 %1）").arg(result).toStdString());
    return pose;
}

}

void CollectionController::initializeCollection() {
    m_outputDirectory = QDir::homePath() + "/Pictures/caitu-collect";
    m_handEyePath = QString::fromStdString(m_paramPath) + "hand_eye_result.yaml";
    loadCollectionUndistortion();
}

void CollectionController::loadCollectionUndistortion() {
    m_undistortionCalibration.reset();
    m_undistortionMaps.reset();
    try {
        if (!QFileInfo::exists(m_handEyePath))
            throw std::runtime_error("未找到手眼文件，去畸变不可用");
        auto calibration = std::make_shared<collection::UndistortionCalibration>(
            collection::UndistortionCalibration::load(m_handEyePath.toStdString()));
        m_undistortionStatus = calibration->calibratedSize.empty()
            ? "去畸变参数已加载；文件未记录标定分辨率，请确认与当前相机一致"
            : "去畸变参数已加载，标定分辨率 " + QString::number(calibration->calibratedSize.width) +
                "×" + QString::number(calibration->calibratedSize.height);
        m_undistortionCalibration = calibration;
        APP_LOG_EVENT_INFO("collection", "undistortion_load", "success",
            {{"path", m_handEyePath}, {"message", m_undistortionStatus}});
    } catch (const std::exception& error) {
        m_undistortionStatus = QString::fromUtf8(error.what());
        APP_LOG_EVENT_WARNING("collection", "undistortion_load", "unavailable",
            {{"path", m_handEyePath}, {"message", m_undistortionStatus}});
    }
    emit collectionChanged();
}

void CollectionController::toggleUndistortion() {
    if (m_collectionBusy) return;
    ++m_previewGeneration;
    if (m_undistortionMaps) {
        m_undistortionMaps.reset();
        emit collectionChanged();
        setStatus("去畸变已关闭，预览和拍照恢复原始图像");
        return;
    }
    if (!m_undistortionCalibration) { setStatus(m_undistortionStatus); return; }
    if (!m_cameraOpened) { setStatus("请先连接相机再启用去畸变"); return; }
    try {
        m_undistortionMaps = std::make_shared<collection::UndistortionMaps>(
            *m_undistortionCalibration, cv::Size(m_camActualWidth, m_camActualHeight),
            m_camDeviceSerial.toStdString());
        const auto& geometry = m_undistortionMaps->geometry();
        const cv::Mat& output = geometry.outputCameraMatrix;
        APP_LOG_EVENT_INFO("collection", "undistortion_map", "ready",
            {{"pixel_frame", QString::fromLatin1(geometry.pixelFrame)},
             {"alpha", QString::number(geometry.alpha, 'f', 1)},
             {"input", QString("%1x%2").arg(geometry.inputSize.width).arg(geometry.inputSize.height)},
             {"output", QString("%1x%2").arg(geometry.outputSize.width).arg(geometry.outputSize.height)},
             {"cropped", geometry.cropped ? "true" : "false"},
             {"flip_horizontal", geometry.flipHorizontal ? "true" : "false"},
             {"flip_vertical", geometry.flipVertical ? "true" : "false"},
             {"valid_roi", QString("%1,%2,%3,%4").arg(geometry.validPixROI.x).arg(geometry.validPixROI.y)
                  .arg(geometry.validPixROI.width).arg(geometry.validPixROI.height)},
             {"fx", QString::number(output.at<double>(0, 0), 'g', 17)},
             {"fy", QString::number(output.at<double>(1, 1), 'g', 17)},
             {"cx", QString::number(output.at<double>(0, 2), 'g', 17)},
             {"cy", QString::number(output.at<double>(1, 2), 'g', 17)}});
        emit collectionChanged();
        setStatus("去畸变已启用，后续预览和拍照使用校正图像，并保留原始融合图");
    } catch (const std::exception& error) {
        setStatus("启用去畸变失败：" + QString::fromUtf8(error.what()));
    }
}

void CollectionController::openCollectionFolder() {
    if (!QDir().mkpath(m_outputDirectory)) {
        setStatus("无法创建默认图片文件夹");
        return;
    }
    if (!QDesktopServices::openUrl(QUrl::fromLocalFile(m_outputDirectory))) {
        setStatus("无法打开图片文件夹");
        return;
    }
    setStatus("已打开图片文件夹");
}

void CollectionController::beginCollectionOperation(bool holdPreview) {
    if (m_collectionThread.joinable()) m_collectionThread.join();
    ++m_tcpPoseGeneration;
    m_cancelRequested = false;
    m_collectionBusy = true;
    m_ninePointProgress.clear();
    m_manualMoveProgress.clear();
    m_exposureCaptureActive = false;
    m_previewHeld = holdPreview;
    if (holdPreview) ++m_previewGeneration;
    updatePreviewTimer();
    emit collectionChanged();
}

bool CollectionController::persistNinePointPlan(double xMm, double yMm,
    const std::array<double, 9>& yaw, QString& error) {
    if (m_appPath.empty()) return true;
    config_resolver::ConfigUpdates updates;
    updates["nine_point_offset_x_mm"] = xMm;
    updates["nine_point_offset_y_mm"] = yMm;
    updates["nine_point_yaw_deg"] = std::vector<double>(yaw.begin(), yaw.end());
    std::string detail;
    if (!config_resolver::updateFlatConfigFile(m_appPath + "config.yaml", updates, detail)) {
        error = "九点参数写入配置失败：" + QString::fromStdString(detail);
        return false;
    }
    m_ninePointOffsetXMm = xMm;
    m_ninePointOffsetYMm = yMm;
    m_ninePointYawDeg = yaw;
    return true;
}

void CollectionController::finishCollectionOperation(const QString& message, const QString& savedPath) {
    if (!m_manualMoveProgress.isEmpty()) m_manualMoveProgress = message;
    m_manualMoveActive = false;
    if (m_jogLease) m_jogLease->release();
    m_jogLease.reset();
    m_motionFault = m_robotStopUnconfirmed.load();
    m_collectionBusy = m_motionFault;
    m_stopRetrying = false;
    m_previewHeld = false;
    m_exposureCaptureActive = false;
    if (!savedPath.isEmpty()) recordCapture(savedPath);
    updatePreviewTimer();
    QString status = savedPath.isEmpty() ? message : QString("已保存第 %1 张融合图片").arg(m_savedImageCount);
    if (!savedPath.isEmpty() && message != "多曝光融合完成，图片已保存")
        status += "；" + message;
    if (m_motionFault) {
        status += "；未确认机器人停止，操作已锁定，请点击“重试停止”";
        m_ninePointProgress = status;
        if (!m_manualMoveProgress.isEmpty()) m_manualMoveProgress = status;
    }
    emit collectionChanged();
    setStatus(status);
    APP_LOG_EVENT_INFO("collection", "operation", m_motionFault ? "motion_locked" : "finished",
        {{"message", status}, {"image", savedPath}});
}

void CollectionController::confirmMotionStopped() {
    collection::confirmRobotStopped(m_robotHandle);
    m_robotStopUnconfirmed = false;
}

void CollectionController::retryRobotStop() {
    if (m_stopRetrying) return;
    if (m_collectionThread.joinable()) m_collectionThread.join();
    m_stopRetrying = true;
    m_cancelRequested = false;
    m_ninePointProgress = "正在重试停止并确认机器人状态…";
    setStatus(m_ninePointProgress);
    emit collectionChanged();
    const auto session = robotSession();
    try {
        m_collectionThread = std::thread([this, session] {
            auto robotLock = session->acquire(m_cancelRequested);
            if (!robotLock.owns_lock()) {
                QMetaObject::invokeMethod(this, [this] { finishCollectionOperation("操作已取消"); }, Qt::QueuedConnection);
                return;
            }
            QString message;
            try {
                const auto result = cr_moveControl(m_robotHandle, DecStop);
                confirmMotionStopped();
                message = result == sucess ? "已确认机器人停止，操作已解锁"
                    : QString("停止指令返回错误码 %1，但已独立确认机器人停止，操作已解锁").arg(result);
            } catch (const std::exception& error) {
                message = QString::fromUtf8(error.what());
            }
            QMetaObject::invokeMethod(this, [this, message] {
                m_ninePointProgress = message;
                finishCollectionOperation(message);
            }, Qt::QueuedConnection);
        });
    } catch (const std::exception& error) { finishCollectionOperation(QString::fromUtf8(error.what())); }
}

void CollectionController::cancelCollectionOperation() {
    if (m_jogLease) m_jogLease->release();
    if (m_settingsSaving) { setStatus("正在保存配置，请等待完成"); return; }
    if (m_coordinatesImporting) { setStatus("正在导入坐标，请等待完成"); return; }
    if (m_motionFault) { retryRobotStop(); return; }
    if (m_collectionBusy) {
        m_cancelRequested = true;
        if (m_manualMoveActive) {
            m_manualMoveProgress = "正在停止 TCP 移动…";
            emit collectionChanged();
        }
        if (!m_ninePointProgress.isEmpty()) {
            m_ninePointProgress = "正在停止自动采图…";
            emit collectionChanged();
        }
        setStatus("正在停止当前操作…");
    }
}

void CollectionController::recordCapture(const QString& path) {
    m_lastCapturePath = path;
    m_lastCaptureTime = QDateTime::currentDateTime().toString("HH:mm:ss");
    ++m_savedImageCount;
    emit collectionChanged();
}

void CollectionController::capturePhoto() { startCapture(CaptureMode::Single); }
void CollectionController::captureNinePoints(double xMm, double yMm, const QVariantList& yawValues) {
    startCapture(CaptureMode::NinePoints, xMm, yMm, yawValues);
}
QVariantMap CollectionController::configuredPointSummary() const {
    try {
        const int count = static_cast<int>(readConfiguredOffsets(m_paramPath).size());
        return {{"count", count}, {"error", QString{}}};
    } catch (const std::exception& error) {
        return {{"count", 0}, {"error", QString::fromUtf8(error.what())}};
    }
}
void CollectionController::captureConfiguredPoints(bool saveOriginal) {
    startCapture(CaptureMode::Configured, 0, 0, {}, saveOriginal);
}
QVariantList CollectionController::ninePointDefaultYawOffsets() const {
    QVariantList result;
    for (double angle : collection::defaultNinePointYawOffsets) result << angle;
    return result;
}
QVariantList CollectionController::ninePointYawOffsets() const {
    QVariantList result;
    for (double angle : m_ninePointYawDeg) result << angle;
    return result;
}

void CollectionController::startCapture(CaptureMode mode, double xMm, double yMm,
                                        const QVariantList& yawValues, bool saveOriginal) {
    if (m_collectionBusy) return;
    if (!m_cameraOpened || !m_previewActive || m_previewFrameCount == 0) {
        setStatus("请打开相机并等待预览画面后拍照");
        return;
    }
    const bool ninePoints = mode == CaptureMode::NinePoints;
    const bool configured = mode == CaptureMode::Configured;
    const bool automatic = mode != CaptureMode::Single;
    const QString task = configured ? "按配置采图" : "九点采图";
    if (automatic && !m_robotConnected) {
        setStatus(task + "需要连接机器人");
        return;
    }
    std::vector<ConfiguredPoint> offsets;
    if (configured) {
        try { offsets = readConfiguredOffsets(m_paramPath); }
        catch (const std::exception& error) { setStatus(QString::fromUtf8(error.what())); return; }
    }
    collection::NinePointAngles yawOffsets{};
    double angleToleranceDeg = 0.2;
    if (ninePoints) {
        if (yawValues.size() != static_cast<int>(yawOffsets.size())) {
            setStatus("请填写全部九个 Rz 角度"); return;
        }
        for (int point = 0; point < yawValues.size(); ++point) {
            bool valid = false;
            yawOffsets[point] = yawValues[point].toDouble(&valid);
            if (!valid || yawValues[point].isNull() || yawValues[point].type() == QVariant::Bool) {
                setStatus(QString("第 %1 点 Rz 角度无效").arg(point + 1)); return;
            }
            if (yawOffsets[point] != 0)
                angleToleranceDeg = std::min(angleToleranceDeg, std::abs(yawOffsets[point]) / 4);
        }
        try { collection::ninePointTargets({}, xMm, yMm, yawOffsets); }
        catch (const std::exception& error) { setStatus(QString::fromUtf8(error.what())); return; }
    }
    std::vector<int> exposures;
    const auto errorKind = collection::parseExposureList(m_exposureList, exposures);
    if (errorKind == collection::ExposureListError::InvalidValue) {
        setStatus("曝光列表应包含 2～5 个不同的正整数，单位 μs，最大 1000000");
        return;
    }
    if (errorKind == collection::ExposureListError::InvalidCount) {
        setStatus("每次拍照至少需要 2 个、最多 5 个曝光值");
        return;
    }
    const auto camera = m_camera;
    const auto pendingPreview = m_previewFuture;
    const int speed = m_robotSpeedMmS;
    const auto undistortion = m_undistortionMaps;
    const auto handle = m_robotHandle;
    const auto session = robotSession();
    std::function<std::array<double, 6>()> readPose;
    if (m_robotConnected) readPose = [handle] { return readFlangePose(handle); };
    const auto shutterTime = QDateTime::currentDateTime();
    const auto output = m_outputDirectory;
    beginCollectionOperation(false);
    const auto sampleTcp = tcpSamplingCallback(session);
    setStatus(automatic ? task + "：正在检查当前基准位及目标…"
                        : QString("正在采集 %1 个曝光并融合…").arg(exposures.size()));
    try {
        m_collectionThread = std::thread([this, camera, undistortion, handle, readPose, shutterTime,
                                         output, exposures, ninePoints, automatic, configured, task, xMm, yMm, yawOffsets,
                                         angleToleranceDeg, offsets = std::move(offsets), saveOriginal,
                                         session, sampleTcp, speed, pendingPreview] {
            // Drain the one in-flight frame on this worker before using the camera for capture.
            if (pendingPreview.valid()) pendingPreview.wait();
            std::unique_lock<std::mutex> robotLock;
            if (session) robotLock = session->acquire(m_cancelRequested);
            if (session && !robotLock.owns_lock()) {
                QMetaObject::invokeMethod(this, [this] { finishCollectionOperation("操作已取消"); }, Qt::QueuedConnection);
                return;
            }
            QString message, path;
            int saved = 0;
            auto progress = [this, task](const QString& text) {
                QMetaObject::invokeMethod(this, [this, text, task] {
                    m_ninePointProgress = m_cancelRequested ? "正在停止" + task + "…" : text;
                    emit collectionChanged();
                    setStatus(m_ninePointProgress);
                }, Qt::QueuedConnection);
            };
            const collection::ExposureCameraGate gate{
                [this] { suspendPreviewForExposure(); },
                [this] { resumePreviewAfterExposure(); }};
            const int count = configured ? static_cast<int>(offsets.size()) : static_cast<int>(yawOffsets.size());
            if (automatic) m_robotStopUnconfirmed = true;
            try {
                if (!automatic) {
                    const auto result = collection::captureAndSave(*camera, output, shutterTime,
                        exposures, undistortion, readPose, m_cancelRequested, gate);
                    message = result.message;
                    path = result.path;
                } else {
                    if (ninePoints) {
                        QString configError;
                        if (!persistNinePointPlan(xMm, yMm, yawOffsets, configError))
                            throw std::runtime_error(configError.toStdString());
                    }
                    collection::checkCaptureExposures(*camera, exposures);
                    const auto origin = collection::readStationaryFlange(handle);
                    std::vector<collection::FlangePose> targets;
                    std::vector<std::vector<collection::FlangePose>> route;
                    QStringList names, fileLabels;
                    double arrivalAngle = angleToleranceDeg;
                    if (configured) {
                        std::vector<std::string> labels;
                        const int numberWidth = std::max(2, QString::number(offsets.back().number).size());
                        for (const auto& [number, offset] : offsets) {
                            const auto name = QString("p%1").arg(number);
                            names << name;
                            labels.push_back(name.toStdString());
                            fileLabels << QString("P%1").arg(number, numberWidth, 10, QLatin1Char('0'));
                            try { targets.push_back(configuredTarget(origin, offset)); }
                            catch (const std::exception& error) {
                                throw std::runtime_error(name.toStdString() + "：" + error.what());
                            }
                        }
                        route = collection::planConfiguredRoute(handle, origin, targets, labels, m_cancelRequested);
                        auto previous = origin;
                        for (const auto& leg : route) {
                            for (const auto& stop : leg) {
                                const double angle = collection::flangeRotationDistanceDeg(previous, stop);
                                if (angle > 1e-5) arrivalAngle = std::min(arrivalAngle, angle / 4);
                                previous = stop;
                            }
                        }
                    } else {
                        const auto grid = collection::ninePointTargets(origin, xMm, yMm, yawOffsets);
                        collection::checkNinePointRoute(handle, origin, grid);
                        targets.assign(grid.begin(), grid.end());
                        names = QStringList{"X− / Y+", "Y+", "X+ / Y+", "X−", "中心位", "X+", "X− / Y−", "Y−", "X+ / Y−"};
                    }
                    APP_LOG_EVENT_INFO("collection", configured ? "configured_points" : "nine_point", "baseline",
                        {{"x", QString::number(origin[0], 'f', 6)}, {"y", QString::number(origin[1], 'f', 6)},
                         {"z", QString::number(origin[2], 'f', 6)}, {"rx", QString::number(origin[3], 'f', 6)},
                         {"ry", QString::number(origin[4], 'f', 6)}, {"rz", QString::number(origin[5], 'f', 6)},
                         {"offset_x_mm", QString::number(xMm)}, {"offset_y_mm", QString::number(yMm)},
                         {"points", QString::number(count)}});
                    progress(task + " · 基准位稳定等待");
                    collection::LineMotionWait wait;
                    if (configured) wait.angleToleranceDeg = arrivalAngle;
                    collection::waitForStableFlange(handle, origin, m_cancelRequested, wait, sampleTcp);
                    wait.angleToleranceDeg = arrivalAngle;
                    wait.timeoutMs = std::max(wait.timeoutMs, wait.settlingMs + 5000 +
                        static_cast<int>(std::ceil(collection::maximumNinePointOffsetMm * 1000 / speed)));

                    auto pointProgress = [&](int point) {
                        return QString("第 %1/%2 点 · %3").arg(point + 1).arg(count).arg(names[point]);
                    };
                    auto previous = origin;
                    collection::runPointCapture(origin, targets,
                        [&](const collection::FlangePose& target, int point) {
                            if (point < 0) progress(QString("已保存 %1/%2 · 返回基准位").arg(saved).arg(count));
                            else {
                                auto text = pointProgress(point);
                                if (ninePoints) text += QString(" · 欧拉 Rz %1°").arg(yawOffsets[point]);
                                progress(text + " · 移动及稳定等待");
                            }
                            if (configured) {
                                collection::moveFlangeAlongConfiguredLeg(handle, origin, previous,
                                    route[point < 0 ? count : point], m_cancelRequested, wait, sampleTcp, speed);
                                previous = target;
                            } else {
                                collection::moveFlangeAlongGrid(handle, origin, target, m_cancelRequested, wait, sampleTcp, speed);
                            }
                        },
                        [&](int point) {
                            if (!collection::flangeAtTarget(collection::readStationaryFlange(handle), targets[point], arrivalAngle))
                                throw std::runtime_error("实际位姿偏离本次拍摄目标");
                            progress(pointProgress(point) + " · 正在采集及保存");
                            const auto shotTime = QDateTime::currentDateTime();
                            const auto result = collection::captureAndSave(*camera, output, configured ? shutterTime : shotTime,
                                exposures, undistortion, readPose, m_cancelRequested, gate,
                                configured ? collection::CaptureNaming{fileLabels[point], saveOriginal} : collection::CaptureNaming{});
                            if (!result.path.isEmpty()) {
                                ++saved;
                                QMetaObject::invokeMethod(this, [this, image = result.path] { recordCapture(image); }, Qt::QueuedConnection);
                                APP_LOG_EVENT_INFO("collection", configured ? "configured_point_image" : "nine_point_image", "saved",
                                    {{"index", QString::number(point + 1)}, {"point", names[point]},
                                     {"euler_rz_offset_deg", QString::number(configured ? offsets[point].offset[5] : yawOffsets[point])},
                                     {"shot_time", shotTime.toString(Qt::ISODateWithMs)},
                                     {"image", result.path}, {"complete", result.complete ? "true" : "false"}});
                            }
                            if (!result.complete) throw std::runtime_error(result.message.toStdString());
                            if (!collection::flangeAtTarget(collection::readStationaryFlange(handle), targets[point], arrivalAngle))
                                throw std::runtime_error("采图后检测到机器人位姿变化，已停止后续动作");
                        }, m_cancelRequested);
                    message = QString("%1完成 · 已保存 %2/%2，已回到基准位").arg(task).arg(count);
                }
            } catch (const std::exception& error) {
                message = automatic ? QString("%1停止 · 已保存 %2/%3：%4；未继续移动")
                    .arg(task).arg(saved).arg(count).arg(QString::fromUtf8(error.what())) : QString::fromUtf8(error.what());
            } catch (...) { message = "采图发生未知异常，已终止后续操作"; }
            if (automatic) {
                try { confirmMotionStopped(); }
                catch (const std::exception& error) { message += "；" + QString::fromUtf8(error.what()); }
            }
            QMetaObject::invokeMethod(this, [this, message, path, automatic] {
                if (automatic) m_ninePointProgress = message;
                finishCollectionOperation(message, path);
            }, Qt::QueuedConnection);
        });
    } catch (const std::exception& error) {
        finishCollectionOperation(QString::fromUtf8(error.what()));
    }
}

void CollectionController::setSelectedTcp(const QString& name) {
    if (m_collectionBusy || !m_robotConnected || (!name.isEmpty() && !m_tcpNames.contains(name))) return;
    if (m_selectedTcp == name) return;
    m_selectedTcp = name;
    m_tcpPoseLastFailure.clear();
    ++m_tcpPoseGeneration;
    invalidateTcpPose(name.isEmpty() ? "请选择 TCP 工具" : "正在读取所选 TCP…");
    emit tcpToolsChanged();
    pollTcpPose();
}

void CollectionController::refreshTcpTools() {
    if (m_collectionBusy || !m_robotConnected) return;
    const QString previous = m_selectedTcp;
    m_tcpNames.clear();
    m_selectedTcp.clear();
    ++m_tcpPoseGeneration;
    invalidateTcpPose("正在读取 TCP 列表…");
    emit tcpToolsChanged();
    const auto handle = m_robotHandle;
    const auto session = robotSession();
    beginCollectionOperation();
    setStatus("正在读取 TCP 列表…");
    try {
        m_collectionThread = std::thread([this, handle, previous, session]() {
            auto robotLock = session->acquire(m_cancelRequested);
            if (!robotLock.owns_lock()) {
                QMetaObject::invokeMethod(this, [this] { finishCollectionOperation("操作已取消"); }, Qt::QueuedConnection);
                return;
            }
            collection::TcpList tools;
            QString selected, message;
            try {
                tools = collection::readTcpList(handle);
                if (m_cancelRequested) throw std::runtime_error("TCP 列表读取已取消");
                selected = previous.isEmpty() ? tools.current :
                    tools.names.contains(previous) ? previous : QString();
                message = tools.names.isEmpty() ? "机器人未配置 TCP 工具" :
                    QString("已加载 %1 个 TCP").arg(tools.names.size());
                if (!tools.warning.isEmpty()) message += "；" + tools.warning;
                if (!tools.names.isEmpty() && selected.isEmpty()) message += "；请选择 TCP 工具";
            } catch (const std::exception& error) {
                tools = {};
                message = QString::fromUtf8(error.what());
            } catch (...) {
                tools = {};
                message = "TCP 列表读取发生未知异常";
            }
            QMetaObject::invokeMethod(this, [this, tools, selected, message]() {
                m_tcpNames = tools.names;
                m_selectedTcp = selected;
                emit tcpToolsChanged();
                finishCollectionOperation(message);
            }, Qt::QueuedConnection);
        });
    } catch (const std::exception& error) { finishCollectionOperation(QString::fromUtf8(error.what())); }
}

void CollectionController::saveTcpPose(const QString& name) {
    if (m_collectionBusy) return;
    if (!m_robotConnected) { setStatus("请先连接机器人再保存 TCP 坐标"); return; }
    if (m_selectedTcp.isEmpty() || !m_tcpNames.contains(m_selectedTcp)) {
        setStatus("请选择有效的 TCP 工具"); return;
    }
    const QString pointName = name.trimmed();
    const bool invalidCharacter = std::any_of(pointName.begin(), pointName.end(), [](QChar c) {
        return c.unicode() < 32 || QStringLiteral("\\/:*?\"<>|").contains(c);
    });
    if (pointName.size() > 100 || invalidCharacter ||
        pointName == "." || pointName == ".." || pointName.endsWith('.')) {
        setStatus("点位名称不能包含路径、特殊字符或超过 100 个字符"); return;
    }
    const auto time = QDateTime::currentDateTime();
    const QString directory = QDir(m_outputDirectory).filePath(time.toString("yyyy-MM-dd"));
    const QString selected = m_selectedTcp;
    QString fileTool;
    for (QChar c : selected)
        fileTool += c.unicode() < 32 || QStringLiteral("\\/:*?\"<>|").contains(c) ? QChar('_') : c;
    const QString base = "tcp_" + fileTool + "_" +
        (pointName.isEmpty() ? time.toString("yyyyMMdd_HHmmss_zzz") : pointName);
    const auto handle = m_robotHandle;
    const auto session = robotSession();
    beginCollectionOperation();
    setStatus("正在读取 TCP 坐标：" + selected + "…");
    try {
        m_collectionThread = std::thread([this, handle, directory, base, selected, session]() {
            auto robotLock = session->acquire(m_cancelRequested);
            if (!robotLock.owns_lock()) {
                QMetaObject::invokeMethod(this, [this] { finishCollectionOperation("操作已取消"); }, Qt::QueuedConnection);
                return;
            }
            QString message; bool saved = false;
            try {
                const auto tcp = collection::readTcpPose(handle, selected);
                if (m_cancelRequested) throw std::runtime_error("点位保存已取消");
                QString path;
                message = collection::saveStandaloneTcpPose(directory, base, tcp.pose.data(),
                    tcp.name, tcp.offset.data(), path);
                saved = !path.isEmpty();
                if (saved) message = "TCP 坐标已保存：" + QFileInfo(path).fileName();
            } catch (const std::exception& error) { message = QString::fromUtf8(error.what()); }
            catch (...) { message = "TCP 位姿读取或保存发生未知异常"; }
            QMetaObject::invokeMethod(this, [this, message, saved]() {
                if (saved) ++m_savedPoseCount;
                finishCollectionOperation(message);
            }, Qt::QueuedConnection);
        });
    } catch (const std::exception& error) { finishCollectionOperation(QString::fromUtf8(error.what())); }
}
