#include "CalibrationController.hpp"
#include "DiagnosticLogger.hpp"
#include "CollectionPoseRecord.hpp"
#include "utils/collection_utils.hpp"
#include "utils/calib_transform.hpp"
#include "utils/robot_util.hpp"
#include <QCoreApplication>
#include <QDateTime>
#include <QDesktopServices>
#include <QDir>
#include <QFileInfo>
#include <QRegularExpression>
#include <QSaveFile>
#include <QSettings>
#include <QUuid>
#include <QUrl>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <future>
#include <set>
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

void CalibrationController::initializeCollection() {
    QSettings settings;
    m_outputDirectory = QDir::homePath() + "/Pictures/caitu-collect";
    m_exposureList = settings.value("collection/exposures", m_exposureList).toString();
    m_handEyePath = QString::fromStdString(m_paramPath) + "hand_eye_result.yaml";
}

void CalibrationController::openCollectionFolder() {
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

void CalibrationController::setExposureList(const QString& value) {
    if (m_collectionBusy || m_running) return;
    m_exposureList = value.trimmed();
    QSettings().setValue("collection/exposures", m_exposureList);
    emit collectionChanged();
}
void CalibrationController::beginCollectionOperation() {
    if (m_collectionThread.joinable()) m_collectionThread.join();
    m_collectionWasPreviewing = m_previewActive;
    m_previewTimer->stop();
    m_cancelRequested = false;
    m_collectionBusy = true;
    emit collectionChanged();
}

void CalibrationController::finishCollectionOperation(const QString& message, const QString& savedPath) {
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

void CalibrationController::cancelCollectionOperation() {
    if (m_collectionBusy) {
        m_cancelRequested = true;
        setStatus("正在停止当前操作…");
    }
}

void CalibrationController::capturePhoto() {
    if (m_running || m_collectionBusy) return;
    if (!m_cameraOpened || !m_previewActive || m_previewFrameCount == 0) {
        setStatus("请打开相机并等待预览画面后拍照");
        return;
    }
    std::vector<int> exposures;
    std::set<int> distinct;
    const auto tokens = m_exposureList.split(QRegularExpression("[,，;\\s]+"), Qt::SkipEmptyParts);
    for (const auto& token : tokens) {
        bool ok = false;
        const int exposure = token.toInt(&ok);
        if (!ok || exposure < 1 || exposure > 1000000 || !distinct.insert(exposure).second) {
            setStatus("曝光列表应包含 2～8 个不同的正整数，单位 μs，最大 1000000");
            return;
        }
        exposures.push_back(exposure);
    }
    if (exposures.size() < 2 || exposures.size() > 8) {
        setStatus("每次拍照至少需要 2 个、最多 8 个曝光值");
        return;
    }
    const auto camera = m_camera;
    const bool robotConnected = m_robotConnected;
    const RobotHandle robotHandle = m_robotHandle;
    // Freeze the date at shutter time so a bracket crossing midnight stays in
    // one daily folder, with a filename that uses the same timestamp.
    const QDateTime captureTime = QDateTime::currentDateTime();
    const QString directory = QDir(m_outputDirectory).filePath(captureTime.toString("yyyy-MM-dd"));
    beginCollectionOperation();
    setStatus(QString("正在采集 %1 个曝光并融合…").arg(exposures.size()));
    try {
        m_collectionThread = std::thread([this, camera, directory, captureTime, exposures, robotConnected, robotHandle]() {
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
                if (m_cancelRequested) throw std::runtime_error("拍照已取消");
                if (!QDir().mkpath(directory))
                    throw std::runtime_error("无法创建图片保存目录");
                path = QDir(directory).filePath("capture_" +
                    captureTime.toString("yyyyMMdd_HHmmss_zzz") + "_" +
                    QUuid::createUuid().toString(QUuid::WithoutBraces).left(8) + "_fused.png");
                std::vector<uchar> encoded;
                if (!cv::imencode(".png", fused, encoded))
                    throw std::runtime_error("图片编码失败");
                if (m_cancelRequested) throw std::runtime_error("拍照已取消");
                QSaveFile file(path);
                if (!file.open(QIODevice::WriteOnly) ||
                    file.write(reinterpret_cast<const char*>(encoded.data()), encoded.size()) != static_cast<qint64>(encoded.size()) ||
                    !file.commit())
                    throw std::runtime_error(("图片保存失败: " + file.errorString()).toStdString());
                message = "多曝光融合完成，图片已保存";
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

void CalibrationController::levelCamera() {
    if (m_running || m_collectionBusy) return;
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
    setRobotMotionCancelCheck(&CalibrationController::robotMotionCancellationRequested);
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
