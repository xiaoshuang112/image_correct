#include "CollectionController.hpp"
#include "DiagnosticLogger.hpp"
#include "CollectionPoseRecord.hpp"
#include "CollectionCapture.hpp"
#include "FivePointCapture.hpp"
#include "RobotTcp.hpp"
#include "ExposureList.hpp"
#include "utils/collection_undistortion.hpp"
#include <QDateTime>
#include <QDesktopServices>
#include <QDir>
#include <QFileInfo>
#include <QUrl>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace {
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

void CollectionController::beginCollectionOperation() {
    if (m_collectionThread.joinable()) m_collectionThread.join();
    ++m_tcpPoseGeneration;
    m_collectionWasPreviewing = m_previewActive;
    ++m_previewGeneration;
    m_previewTimer->stop();
    m_cancelRequested = false;
    m_collectionBusy = true;
    m_fivePointProgress.clear();
    m_manualMoveProgress.clear();
    emit collectionChanged();
}

void CollectionController::finishCollectionOperation(const QString& message, const QString& savedPath) {
    if (!m_manualMoveProgress.isEmpty()) m_manualMoveProgress = message;
    m_manualMoveActive = false;
    m_motionFault = m_robotStopUnconfirmed.load();
    m_collectionBusy = m_motionFault;
    m_stopRetrying = false;
    if (!savedPath.isEmpty()) recordCapture(savedPath);
    if (!m_collectionBusy && m_collectionWasPreviewing && m_previewActive && m_cameraOpened)
        updatePreviewTimer();
    QString status = savedPath.isEmpty() ? message : QString("已保存第 %1 张融合图片").arg(m_savedImageCount);
    if (!savedPath.isEmpty() && message != "多曝光融合完成，图片已保存")
        status += "；" + message;
    if (m_motionFault) {
        status += "；未确认机器人停止，操作已锁定，请点击“重试停止”";
        m_fivePointProgress = status;
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
    m_fivePointProgress = "正在重试停止并确认机器人状态…";
    setStatus(m_fivePointProgress);
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
                const auto result = cr_stop(m_robotHandle);
                confirmMotionStopped();
                message = result == sucess ? "已确认机器人停止，操作已解锁"
                    : QString("停止指令返回错误码 %1，但已独立确认机器人停止，操作已解锁").arg(result);
            } catch (const std::exception& error) {
                message = QString::fromUtf8(error.what());
            }
            QMetaObject::invokeMethod(this, [this, message] {
                m_fivePointProgress = message;
                finishCollectionOperation(message);
            }, Qt::QueuedConnection);
        });
    } catch (const std::exception& error) { finishCollectionOperation(QString::fromUtf8(error.what())); }
}

void CollectionController::cancelCollectionOperation() {
    if (m_settingsSaving) { setStatus("正在保存配置，请等待完成"); return; }
    if (m_coordinatesImporting) { setStatus("正在导入坐标，请等待完成"); return; }
    if (m_motionFault) { retryRobotStop(); return; }
    if (m_collectionBusy) {
        m_cancelRequested = true;
        if (m_manualMoveActive) {
            m_manualMoveProgress = "正在停止 TCP 移动…";
            emit collectionChanged();
        }
        if (!m_fivePointProgress.isEmpty()) {
            m_fivePointProgress = "正在停止五点采图…";
            emit collectionChanged();
        }
        setStatus("正在停止当前操作…");
    }
}

void CollectionController::recordCapture(const QString& path) {
    m_lastCapturePath = path;
    ++m_savedImageCount;
    emit collectionChanged();
}

void CollectionController::capturePhoto() { startCapture(false); }
void CollectionController::captureFivePoints(double xMm, double yMm) { startCapture(true, xMm, yMm); }

void CollectionController::startCapture(bool fivePoints, double xMm, double yMm) {
    if (m_collectionBusy) return;
    if (!m_cameraOpened || !m_previewActive || m_previewFrameCount == 0) {
        setStatus("请打开相机并等待预览画面后拍照");
        return;
    }
    if (fivePoints) {
        if (!m_robotConnected) { setStatus("五点采图需要连接机器人"); return; }
        try { collection::fivePointTargets({}, xMm, yMm); }
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
    beginCollectionOperation();
    const auto sampleTcp = tcpSamplingCallback(session);
    setStatus(fivePoints ? "五点采图：正在检查当前基准位及目标…"
                        : QString("正在采集 %1 个曝光并融合…").arg(exposures.size()));
    try {
        m_collectionThread = std::thread([this, camera, undistortion, handle, readPose, shutterTime,
                                         output, exposures, fivePoints, xMm, yMm, session, sampleTcp, speed, pendingPreview] {
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
            auto progress = [this](const QString& text) {
                QMetaObject::invokeMethod(this, [this, text] {
                    m_fivePointProgress = m_cancelRequested ? "正在停止五点采图…" : text;
                    emit collectionChanged();
                    setStatus(m_fivePointProgress);
                }, Qt::QueuedConnection);
            };
            if (fivePoints) m_robotStopUnconfirmed = true;
            try {
                if (!fivePoints) {
                    const auto result = collection::captureAndSave(*camera, output, shutterTime,
                        exposures, undistortion, readPose, m_cancelRequested);
                    message = result.message;
                    path = result.path;
                } else {
                    collection::checkCaptureExposures(*camera, exposures);
                    const auto origin = collection::readStationaryFlange(handle);
                    const auto targets = collection::fivePointTargets(origin, xMm, yMm);
                    collection::checkFivePointReachability(handle, targets);
                    APP_LOG_EVENT_INFO("collection", "five_point", "baseline",
                        {{"x", QString::number(origin[0], 'f', 6)}, {"y", QString::number(origin[1], 'f', 6)},
                         {"z", QString::number(origin[2], 'f', 6)}, {"rx", QString::number(origin[3], 'f', 6)},
                         {"ry", QString::number(origin[4], 'f', 6)}, {"rz", QString::number(origin[5], 'f', 6)},
                         {"offset_x_mm", QString::number(xMm)}, {"offset_y_mm", QString::number(yMm)}});
                    progress("五点采图 · 基准位稳定等待");
                    collection::waitForStableFlange(handle, origin, m_cancelRequested, {}, sampleTcp);
                    const QStringList names{"基准位", "X+", "X−", "Y+", "Y−"};
                    collection::runFivePointCapture(targets,
                        [&](const collection::FlangePose& target, int point) {
                            progress(point == 0 ? QString("已保存 %1/5 · 返回基准位").arg(saved)
                                : QString("第 %1/5 点 · %2 · 移动及稳定等待").arg(point + 1).arg(names[point]));
                            // Allow the longest 50 mm leg at the selected speed, plus settling and a margin.
                            collection::LineMotionWait wait;
                            wait.timeoutMs = std::max(wait.timeoutMs, wait.settlingMs + 5000 +
                                static_cast<int>(std::ceil(collection::maximumFivePointOffsetMm * 1000 / speed)));
                            collection::moveFlangeLinear(handle, target, m_cancelRequested, wait, sampleTcp, speed);
                        },
                        [&](int point) {
                            if (!collection::flangeAtTarget(collection::readStationaryFlange(handle), targets[point]))
                                throw std::runtime_error("实际位姿偏离本次拍摄目标");
                            progress(QString("第 %1/5 点 · %2 · 正在采集及保存").arg(point + 1).arg(names[point]));
                            const auto result = collection::captureAndSave(*camera, output, QDateTime::currentDateTime(),
                                exposures, undistortion, readPose, m_cancelRequested);
                            if (!result.path.isEmpty()) {
                                ++saved;
                                QMetaObject::invokeMethod(this, [this, image = result.path] { recordCapture(image); }, Qt::QueuedConnection);
                                APP_LOG_EVENT_INFO("collection", "five_point_image", "saved",
                                    {{"point", names[point]}, {"image", result.path}, {"complete", result.complete ? "true" : "false"}});
                            }
                            if (!result.complete) throw std::runtime_error(result.message.toStdString());
                            if (!collection::flangeAtTarget(collection::readStationaryFlange(handle), targets[point]))
                                throw std::runtime_error("采图后检测到机器人位姿变化，已停止后续动作");
                        }, m_cancelRequested);
                    message = "五点采图完成 · 已保存 5/5，已回到基准位";
                }
            } catch (const std::exception& error) {
                message = fivePoints ? QString("五点采图停止 · 已保存 %1/5：%2；未继续移动")
                    .arg(saved).arg(QString::fromUtf8(error.what())) : QString::fromUtf8(error.what());
            } catch (...) { message = "采图发生未知异常，已终止后续操作"; }
            if (fivePoints) {
                try { confirmMotionStopped(); }
                catch (const std::exception& error) { message += "；" + QString::fromUtf8(error.what()); }
            }
            QMetaObject::invokeMethod(this, [this, message, path, fivePoints] {
                if (fivePoints) m_fivePointProgress = message;
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
