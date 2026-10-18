#include "CollectionController.hpp"
#include "DiagnosticLogger.hpp"
#include "CollectionPoseRecord.hpp"
#include "CollectionImageRecord.hpp"
#include "ExposureList.hpp"
#include "utils/collection_undistortion.hpp"
#include "config_resolver.hpp"
#include "utils/collection_utils.hpp"
#include "utils/calib_transform.hpp"
#include "utils/robot_util.hpp"
#include <QCoreApplication>
#include <QDateTime>
#include <QDesktopServices>
#include <QDir>
#include <QFileInfo>
#include <QSaveFile>
#include <QUuid>
#include <QUrl>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <future>
#include <stdexcept>

namespace {
struct FlangeRead {
    std::array<double, 6> pose{};
    bool valid = false;
    QString error;
};

FlangeRead readFlangePose(RobotHandle handle) {
    FlangeRead result;
    double zeroTool[ROB_AXIS_NUM]{};
    const CRresult code = cr_get_AssignTCP_Pose(handle, zeroTool, result.pose.data());
    if (code != sucess) {
        result.error = QString("法兰位姿读取失败（错误码 %1），本张位姿为空").arg(code);
    } else if (!std::all_of(result.pose.begin(), result.pose.end(),
                           [](double value) { return std::isfinite(value); })) {
        result.error = "法兰位姿包含无效数值，本张位姿为空";
    } else {
        result.valid = true;
    }
    return result;
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

void CollectionController::setExposureList(const QString& value) {
    if (m_collectionBusy) return;
    if (m_exposureList == value.trimmed()) return;
    m_exposureList = value.trimmed();
    emit collectionChanged();
    if (m_appPath.empty()) return;
    std::vector<int> exposures;
    const auto errorKind = collection::parseExposureList(m_exposureList, exposures);
    if (errorKind == collection::ExposureListError::InvalidValue) {
        setStatus("融合曝光应为不同的正整数（1～1000000 μs），未写入配置");
        return;
    }
    if (errorKind == collection::ExposureListError::InvalidCount) {
        setStatus("融合曝光应包含 2～5 个数值，未写入配置");
        return;
    }
    std::string error;
    if (!config_resolver::updateFlatIntegerSequenceConfigFile(
            m_appPath + "config.yaml", "fusion_exposures_us", exposures, error)) {
        APP_LOG_EVENT_ERROR("config", "collection_persist", "fail",
            {{"field", "fusion_exposures_us"}, {"error", QString::fromStdString(error)}});
        setStatus("曝光序列已更新，但配置保存失败：" + QString::fromStdString(error));
    }
}

void CollectionController::persistCollectionString(const std::string& key, const QString& value) {
    if (m_appPath.empty()) return;
    std::string error;
    if (!config_resolver::updateFlatStringConfigFile(m_appPath + "config.yaml", key, value.toStdString(), error)) {
        APP_LOG_EVENT_ERROR("config", "collection_persist", "fail",
            {{"field", QString::fromStdString(key)}, {"error", QString::fromStdString(error)}});
        setStatus("本次参数已更新，但配置保存失败：" + QString::fromStdString(error));
    }
}

void CollectionController::persistCollectionInteger(const std::string& key, int value) {
    if (m_appPath.empty()) return;
    std::string error;
    if (!config_resolver::updateFlatIntegerConfigFile(m_appPath + "config.yaml", key, value, error)) {
        APP_LOG_EVENT_ERROR("config", "collection_persist", "fail",
            {{"field", QString::fromStdString(key)}, {"error", QString::fromStdString(error)}});
        setStatus("本次参数已更新，但配置保存失败：" + QString::fromStdString(error));
    }
}
void CollectionController::beginCollectionOperation() {
    if (m_collectionThread.joinable()) m_collectionThread.join();
    m_collectionWasPreviewing = m_previewActive;
    m_previewTimer->stop();
    m_cancelRequested = false;
    m_collectionBusy = true;
    emit collectionChanged();
}

void CollectionController::finishCollectionOperation(const QString& message, const QString& savedPath) {
    m_collectionBusy = false;
    if (!savedPath.isEmpty()) {
        m_lastCapturePath = savedPath;
        ++m_savedImageCount;
    }
    if (m_collectionWasPreviewing && m_previewActive && m_cameraOpened)
        m_previewTimer->start();
    emit collectionChanged();
    QString status = savedPath.isEmpty() ? message : QString("已保存第 %1 张融合图片").arg(m_savedImageCount);
    if (!savedPath.isEmpty() && message != "多曝光融合完成，图片已保存")
        status += "；" + message;
    setStatus(status);
    APP_LOG_EVENT_INFO("collection", "operation", "finished", {{"message", message}, {"image", savedPath}});
}

void CollectionController::cancelCollectionOperation() {
    if (m_collectionBusy) {
        m_cancelRequested = true;
        setStatus("正在停止当前操作…");
    }
}

void CollectionController::capturePhoto() {
    if (m_collectionBusy) return;
    if (!m_cameraOpened || !m_previewActive || m_previewFrameCount == 0) {
        setStatus("请打开相机并等待预览画面后拍照");
        return;
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
    const auto undistortion = m_undistortionMaps;
    const bool robotConnected = m_robotConnected;
    const RobotHandle robotHandle = m_robotHandle;
    // Freeze the date at shutter time so a bracket crossing midnight stays in
    // one daily folder, with a filename that uses the same timestamp.
    const QDateTime captureTime = QDateTime::currentDateTime();
    const QString directory = QDir(m_outputDirectory).filePath(captureTime.toString("yyyy-MM-dd"));
    beginCollectionOperation();
    setStatus(QString("正在采集 %1 个曝光并融合…").arg(exposures.size()));
    try {
        m_collectionThread = std::thread([this, camera, directory, captureTime, exposures, robotConnected, robotHandle, undistortion]() {
            QString message, path;
            try {
                int minimum = 0, maximum = 0;
                if (!camera->getExposureRange(minimum, maximum))
                    throw std::runtime_error("无法读取相机曝光范围");
                for (int exposure : exposures)
                    if (exposure < minimum || exposure > maximum)
                        throw std::runtime_error(QString("曝光 %1 μs 超出相机范围 %2～%3 μs")
                            .arg(exposure).arg(minimum).arg(maximum).toStdString());
                // Read the zero-offset flange pose alongside camera acquisition.
                // Save the PNG before waiting for the RPC result, so robot errors
                // cannot prevent camera acquisition, fusion or image saving.
                FlangeRead flange;
                std::future<FlangeRead> poseRead;
                try {
                    if (robotConnected) {
                        poseRead = std::async(std::launch::async, [robotHandle]() {
                            return readFlangePose(robotHandle);
                        });
                    } else {
                        flange.error = "机器人未连接，本张法兰位姿为空";
                    }
                } catch (const std::exception& error) {
                    flange.error = "无法启动法兰位姿读取：" + QString::fromUtf8(error.what());
                }
                const auto frames = camera->captureMultiExposureImages(exposures);
                if (m_cancelRequested) throw std::runtime_error("拍照已取消");
                if (frames.size() != exposures.size())
                    throw std::runtime_error("未采齐全部曝光帧，本次未保存图片");
                cv::Mat fused = collection::fuseMertens(frames);
                const cv::Mat corrected = undistortion ? undistortion->apply(fused) : cv::Mat();
                if (m_cancelRequested) throw std::runtime_error("拍照已取消");
                if (!QDir().mkpath(directory))
                    throw std::runtime_error("无法创建图片保存目录");
                const QString rawPath = QDir(directory).filePath("capture_" +
                    captureTime.toString("yyyyMMdd_HHmmss_zzz") + "_" +
                    QUuid::createUuid().toString(QUuid::WithoutBraces).left(8) + "_fused.png");
                if (m_cancelRequested) throw std::runtime_error("拍照已取消");
                path = collection::saveCaptureImages(fused, corrected, rawPath);
                message = undistortion ? "多曝光融合及去畸变完成，校正图与原始图已保存" : "多曝光融合完成，图片已保存";
                // Metadata failure is reported separately from the already
                // committed image. Busy remains set until the read finishes,
                // keeping the robot handle alive while the SDK uses it.
                try {
                    if (poseRead.valid()) {
                        try {
                            flange = poseRead.get();
                        } catch (const std::exception& error) {
                            flange.error = "法兰位姿读取异常：" + QString::fromUtf8(error.what());
                        } catch (...) {
                            flange.error = "法兰位姿读取发生未知异常";
                        }
                    }
                    const QString recordError = collection::saveFlangePoseRecords(
                        path, flange.valid ? flange.pose.data() : nullptr);
                    QStringList warnings;
                    if (undistortion) {
                        const auto rawRecordError = collection::saveFlangePoseRecords(
                            rawPath, flange.valid ? flange.pose.data() : nullptr);
                        if (!rawRecordError.isEmpty()) warnings << rawRecordError;
                    }
                    if (!flange.error.isEmpty()) warnings << flange.error;
                    if (!recordError.isEmpty()) warnings << recordError;
                    if (!warnings.isEmpty()) message = warnings.join("；");
                } catch (const std::exception& error) {
                    message = "图片已保存，法兰位姿记录失败：" + QString::fromUtf8(error.what());
                } catch (...) {
                    message = "图片已保存，法兰位姿记录发生未知异常";
                }
            } catch (const std::exception& error) {
                path.clear();
                message = QString::fromUtf8(error.what());
            } catch (...) {
                path.clear(); message = "拍照发生未知异常";
            }
            QMetaObject::invokeMethod(this, [this, message, path]() {
                finishCollectionOperation(message, path);
            }, Qt::QueuedConnection);
        });
    } catch (const std::exception& error) {
        finishCollectionOperation(QString::fromUtf8(error.what()));
    }
}

void CollectionController::saveFlangePose(const QString& name) {
    if (m_collectionBusy) return;
    if (!m_robotConnected) { setStatus("请先连接机器人再保存法兰位姿"); return; }
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
    const QString base = "pose_" + (pointName.isEmpty() ? time.toString("yyyyMMdd_HHmmss_zzz") : pointName);
    const auto handle = m_robotHandle;
    beginCollectionOperation();
    setStatus("正在读取法兰位姿…");
    try {
        m_collectionThread = std::thread([this, handle, directory, base]() {
            QString message; bool saved = false;
            try {
                const auto flange = readFlangePose(handle);
                if (!flange.valid) throw std::runtime_error((QString(flange.error).replace("本张位姿为空", "未保存点位文件")).toStdString());
                if (m_cancelRequested) throw std::runtime_error("点位保存已取消");
                QString path;
                message = collection::saveStandaloneFlangePose(directory, base, flange.pose.data(), path);
                saved = !path.isEmpty();
                if (saved) message = "点位已保存：" + QFileInfo(path).fileName() + (message.isEmpty() ? "" : "；" + message);
            } catch (const std::exception& error) { message = QString::fromUtf8(error.what()); }
            catch (...) { message = "法兰位姿读取或保存发生未知异常"; }
            QMetaObject::invokeMethod(this, [this, message, saved]() {
                if (saved) ++m_savedPoseCount;
                finishCollectionOperation(message);
            }, Qt::QueuedConnection);
        });
    } catch (const std::exception& error) { finishCollectionOperation(QString::fromUtf8(error.what())); }
}

void CollectionController::levelCamera() {
    if (m_collectionBusy) return;
    if (!m_cameraOpened || !m_previewActive || m_previewFrameCount == 0 || !m_robotConnected) {
        setStatus("调平需要预览画面和机器人连接");
        return;
    }
    // Parallel mounting: without calibration, level the flange at its current
    // position. A real matrix instead preserves the calibrated camera centre.
    cv::Mat flangeCamera = cv::Mat::eye(4, 4, CV_64F);
    bool calibrated = false;
    try {
        if (QFileInfo::exists(m_handEyePath)) {
            cv::FileStorage storage;
            try {
                storage.open(m_handEyePath.toStdString(), cv::FileStorage::READ);
            } catch (const cv::Exception&) {
                throw std::runtime_error("手眼标定文件格式无效");
            }
            if (!storage.isOpened()) throw std::runtime_error("无法读取手眼标定文件");
            const auto node = storage["T_flange_camera"];
            if (!node.empty()) {
                try {
                    if (node.isSeq()) {
                        std::vector<double> values; node >> values;
                        if (values.size() != 16) throw std::runtime_error("手眼矩阵需要 16 个数值");
                        flangeCamera = cv::Mat(4, 4, CV_64F, values.data()).clone();
                    } else {
                        node >> flangeCamera;
                        flangeCamera.convertTo(flangeCamera, CV_64F);
                    }
                    collection::validateTransform(flangeCamera);
                    std::string unit = "m";
                    if (!storage["translation_unit"].empty()) storage["translation_unit"] >> unit;
                    if (unit != "m" && unit != "mm")
                        throw std::runtime_error("手眼矩阵平移单位须为 m 或 mm");
                    if (unit == "mm") flangeCamera(cv::Rect(3, 0, 1, 3)) /= 1000.0;
                    calibrated = true;
                } catch (const cv::Exception&) {
                    throw std::runtime_error("手眼矩阵格式无效");
                }
            }
        }
    } catch (const std::exception& error) {
        setStatus(QString("调平失败: %1").arg(QString::fromUtf8(error.what())));
        return;
    }
    const RobotHandle handle = m_robotHandle;
    beginCollectionOperation();
    setRobotMotionCancelCheck(&CollectionController::robotMotionCancellationRequested);
    setStatus(calibrated ? "正在调平：镜头平面水平，相机向下…"
                         : "正在调平：法兰垂直向下，相机与法兰平行…");
    try {
        m_collectionThread = std::thread([this, handle, flangeCamera, calibrated]() {
            QString message;
            try {
                enum RobotModes mode;
                BOOL moving = TRUE;
                if (cr_get_robotMode(handle, &mode) != sucess ||
                    (mode != ProgramStop && mode != Enable) ||
                    cr_get_robotMoveStatus(handle, &moving) != sucess || moving)
                    throw std::runtime_error("请让机器人处于使能且停止状态后调平");
                double joints[ROB_AXIS_NUM]{}, pose[ROB_AXIS_NUM]{}, targetPose[ROB_AXIS_NUM]{}, targetJoints[ROB_AXIS_NUM]{};
                if (cr_get_jointActualPos(handle, joints) != sucess ||
                    cr_kineForward(handle, joints, pose) != sucess)
                    throw std::runtime_error("读取机器人位姿失败");
                const auto target = collection::downwardFlangeTarget(
                    calib_transform::axisAnglePoseToTransform(pose), flangeCamera);
                calib_transform::transformToAxisAnglePose(target, targetPose);
                if (cr_kineInverse(handle, targetPose, joints, targetJoints) != sucess)
                    throw std::runtime_error("调平目标不可达，请调整机器人位置后重试");
                for (double joint : targetJoints)
                    if (!std::isfinite(joint)) throw std::runtime_error("调平逆解包含无效关节角");
                if (m_cancelRequested) throw std::runtime_error("调平已取消");
                const auto result = moveToJointPos(handle, targetJoints);
                if (result == robot_motion_cancelled) throw std::runtime_error("调平已取消");
                if (result != sucess) throw std::runtime_error("机器人调平运动失败");
                if (cr_get_jointActualPos(handle, joints) != sucess || cr_kineForward(handle, joints, pose) != sucess)
                    throw std::runtime_error("无法读取调平后的实际位姿");
                const cv::Mat actualCamera = calib_transform::axisAnglePoseToTransform(pose) * flangeCamera;
                const double tilt = std::acos(std::clamp(-actualCamera.at<double>(2, 2), -1.0, 1.0)) * 180 / CV_PI;
                if (tilt > 1.0) throw std::runtime_error("调平后的相机倾角超过 1°");
                message = QString(calibrated ? "调平完成，相机向下，倾角 %1°"
                                             : "调平完成，法兰垂直向下，倾角 %1°").arg(tilt, 0, 'f', 2);
            } catch (const std::exception& error) {
                message = QString::fromUtf8(error.what());
            } catch (...) {
                message = "调平发生未知异常";
            }
            setRobotMotionCancelCheck(nullptr);
            QMetaObject::invokeMethod(this, [this, message]() {
                finishCollectionOperation(message);
            }, Qt::QueuedConnection);
        });
    } catch (const std::exception& error) {
        setRobotMotionCancelCheck(nullptr);
        finishCollectionOperation(QString::fromUtf8(error.what()));
    }
}
