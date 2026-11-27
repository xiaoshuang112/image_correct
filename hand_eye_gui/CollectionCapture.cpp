#include "CollectionCapture.hpp"
#include "CollectionImageRecord.hpp"
#include "CollectionPoseRecord.hpp"
#include "utils/collection_undistortion.hpp"
#include "utils/collection_utils.hpp"
#include <QDir>
#include <QFile>
#include <QUuid>
#include <QStringList>
#include <algorithm>
#include <cmath>
#include <future>
#include <stdexcept>

namespace collection {
namespace {
struct CameraOutputLayout {
    bool saveOriginal = true;
    QString rawPath;
    QString correctedPath;
};

CameraOutputLayout prepareCameraOutputLayout(const QString& baseDir, const QString& filename,
    const std::shared_ptr<const UndistortionMaps>& undistortion, bool requestedSaveOriginal) {
    CameraOutputLayout layout;
    layout.saveOriginal = !undistortion || requestedSaveOriginal;
    const QString rawDirectory = undistortion ? QDir(baseDir).filePath("raw") : baseDir;
    const QString correctedDirectory = QDir(baseDir).filePath("undistorted");
    if ((layout.saveOriginal && !QDir().mkpath(rawDirectory)) ||
        (undistortion && !QDir().mkpath(correctedDirectory)))
        throw std::runtime_error("无法创建图片保存子目录");
    layout.rawPath = QDir(rawDirectory).filePath(filename);
    layout.correctedPath = QDir(correctedDirectory).filePath(
        filename.left(filename.size() - 4) + "_undistorted.png");
    return layout;
}

bool readFlangeFromFuture(std::future<std::array<double, 6>>& poseRead,
                          std::array<double, 6>& pose, QString& poseError) {
    if (!poseRead.valid()) return false;
    try {
        pose = poseRead.get();
        const bool valid = std::all_of(pose.begin(), pose.end(),
                                       [](double v) { return std::isfinite(v); });
        if (!valid) poseError = "法兰位姿包含无效数值，本张位姿为空";
        return valid;
    } catch (const std::exception& error) {
        poseError = "法兰位姿读取异常：" + QString::fromUtf8(error.what());
    } catch (...) {
        poseError = "法兰位姿读取发生未知异常";
    }
    return false;
}

QString recordSingleCameraMetadata(const QString& primaryPath, const QString& rawPath,
    const std::shared_ptr<const UndistortionMaps>& undistortion, bool saveOriginal,
    const double* posePtr) {
    QStringList warnings;
    const auto recordError = saveFlangePoseRecords(primaryPath, posePtr);
    if (!recordError.isEmpty()) warnings << recordError;
    if (undistortion) {
        const auto metadataError = saveUndistortionMetadata(primaryPath, undistortion->geometry());
        if (!metadataError.isEmpty()) warnings << metadataError;
        if (saveOriginal) {
            const auto rawError = saveFlangePoseRecords(rawPath, posePtr);
            if (!rawError.isEmpty()) warnings << rawError;
        }
    }
    return warnings.join("；");
}
}  // namespace

void checkCaptureExposures(AICamera& camera, const std::vector<int>& exposures) {
    int minimum = 0, maximum = 0;
    if (!camera.getExposureRange(minimum, maximum))
        throw std::runtime_error("无法读取相机曝光范围");
    for (int exposure : exposures)
        if (exposure < minimum || exposure > maximum)
            throw std::runtime_error(QString("曝光 %1 μs 超出相机范围 %2～%3 μs")
                .arg(exposure).arg(minimum).arg(maximum).toStdString());
}

CaptureResult captureAndSave(AICamera& camera, const QString& outputDirectory,
    const QDateTime& captureTime, const std::vector<int>& exposures,
    const std::shared_ptr<const UndistortionMaps>& undistortion,
    const std::function<std::array<double, 6>()>& readFlange,
    const std::atomic<bool>& cancelled, ExposureCameraGate gate, CaptureNaming naming) {
    CaptureResult result;
    const auto checkCancelled = [&] {
        if (cancelled.load()) throw std::runtime_error("拍照已取消");
    };
    try {
        checkCancelled();
        checkCaptureExposures(camera, exposures);
        std::future<std::array<double, 6>> poseRead;
        QString poseError;
        try {
            if (readFlange) poseRead = std::async(std::launch::async, readFlange);
            else poseError = "机器人未连接，本张法兰位姿为空";
        } catch (const std::exception& error) {
            poseError = "无法启动法兰位姿读取：" + QString::fromUtf8(error.what());
        }
        bool suspended = false;
        if (gate.suspend) {
            gate.suspend();
            suspended = true;
        }
        const auto resume = [&] {
            if (!suspended || !gate.resume) return;
            suspended = false;
            try { gate.resume(); }
            catch (...) {}
        };
        std::vector<cv::Mat> frames;
        try {
            checkCancelled();
            frames = camera.captureMultiExposureImages(exposures);
            resume();
        } catch (...) {
            resume();
            throw;
        }
        checkCancelled();
        if (frames.size() != exposures.size())
            throw std::runtime_error("未采齐全部曝光帧，本次未保存图片");
        const auto fused = fuseMertens(frames);
        const cv::Mat corrected = undistortion ? undistortion->apply(fused) : cv::Mat();
        checkCancelled();
        const QString directory = QDir(outputDirectory).filePath(captureTime.toString("yyyy-MM-dd"));
        if (!QDir().mkpath(directory)) throw std::runtime_error("无法创建图片保存目录");
        const auto label = naming.pointLabel.isEmpty()
            ? QUuid::createUuid().toString(QUuid::WithoutBraces).left(8) : naming.pointLabel;
        const bool saveOriginal = !undistortion || naming.saveOriginal;
        const QString filename = "capture_" +
            captureTime.toString("yyyyMMdd_HHmmss_zzz") + "_" +
            label + "_fused.png";
        const QString rawDirectory = undistortion ? QDir(directory).filePath("raw") : directory;
        const QString correctedDirectory = QDir(directory).filePath("undistorted");
        if ((saveOriginal && !QDir().mkpath(rawDirectory)) || (undistortion && !QDir().mkpath(correctedDirectory)))
            throw std::runtime_error("无法创建图片保存子目录");
        const QString rawPath = QDir(rawDirectory).filePath(filename);
        const QString correctedPath = QDir(correctedDirectory).filePath(
            filename.left(filename.size() - 4) + "_undistorted.png");
        checkCancelled();
        result.path = saveCaptureImages(fused, corrected, rawPath, correctedPath, saveOriginal);
        result.message = undistortion ? (saveOriginal ? "多曝光融合及去畸变完成，校正图与原始图已保存"
                                                      : "多曝光融合及去畸变完成，校正图已保存")
                                     : "多曝光融合完成，图片已保存";
        // Keep manual captures even when pose/metadata fails; automatic motion
        // may continue only when complete is true. Wait for the RPC before returning.
        try {
            std::array<double, 6> pose{};
            const bool valid = readFlangeFromFuture(poseRead, pose, poseError);
            QStringList warnings;
            const auto metaErr = recordSingleCameraMetadata(
                result.path, rawPath, undistortion, saveOriginal, valid ? pose.data() : nullptr);
            if (!metaErr.isEmpty()) warnings << metaErr;
            if (!poseError.isEmpty()) warnings << poseError;
            result.complete = valid && warnings.isEmpty();
            if (!warnings.isEmpty()) result.message = warnings.join("；");
        } catch (const std::exception& error) {
            result.message = "图片已保存，法兰位姿记录失败：" + QString::fromUtf8(error.what());
        } catch (...) { result.message = "图片已保存，法兰位姿记录发生未知异常"; }
    } catch (const std::exception& error) {
        result.message = QString::fromUtf8(error.what());
    } catch (...) { result.message = "拍照发生未知异常"; }
    return result;
}

DualCaptureResult captureDualAndSave(
    AICamera& camera1, AICamera& camera2,
    const QString& outputDirectory, const QDateTime& captureTime,
    const std::vector<int>& exposures1, const std::vector<int>& exposures2,
    const std::shared_ptr<const UndistortionMaps>& undistortion1,
    const std::shared_ptr<const UndistortionMaps>& undistortion2,
    const std::function<std::array<double, 6>()>& readFlange,
    const std::atomic<bool>& cancelled,
    ExposureCameraGate gate, CaptureNaming naming) {
    DualCaptureResult result;
    const auto checkCancelled = [&] {
        if (cancelled.load()) throw std::runtime_error("拍照已取消");
    };
    try {
        checkCancelled();
        try {
            checkCaptureExposures(camera1, exposures1);
        } catch (const std::exception& error) {
            throw std::runtime_error(("相机 1：" + QString::fromUtf8(error.what())).toStdString());
        }
        try {
            checkCaptureExposures(camera2, exposures2);
        } catch (const std::exception& error) {
            throw std::runtime_error(("相机 2：" + QString::fromUtf8(error.what())).toStdString());
        }
        std::future<std::array<double, 6>> poseRead;
        QString poseError;
        try {
            if (readFlange) poseRead = std::async(std::launch::async, readFlange);
            else poseError = "机器人未连接，本组法兰位姿为空";
        } catch (const std::exception& error) {
            poseError = "无法启动法兰位姿读取：" + QString::fromUtf8(error.what());
        }
        bool suspended = false;
        if (gate.suspend) {
            gate.suspend();
            suspended = true;
        }
        const auto resume = [&] {
            if (!suspended || !gate.resume) return;
            suspended = false;
            try { gate.resume(); }
            catch (...) {}
        };
        std::vector<cv::Mat> frames1, frames2;
        try {
            checkCancelled();
            auto future1 = std::async(std::launch::async, [&] {
                return camera1.captureMultiExposureImages(exposures1);
            });
            auto future2 = std::async(std::launch::async, [&] {
                return camera2.captureMultiExposureImages(exposures2);
            });
            std::exception_ptr firstError;
            try {
                frames1 = future1.get();
            } catch (...) {
                firstError = std::current_exception();
            }
            try {
                frames2 = future2.get();
            } catch (...) {
                if (!firstError) firstError = std::current_exception();
            }
            resume();
            if (firstError) std::rethrow_exception(firstError);
        } catch (...) {
            resume();
            throw;
        }
        checkCancelled();
        if (frames1.size() != exposures1.size()) {
            const auto err = camera1.lastError().message;
            throw std::runtime_error(err.empty()
                ? "相机 1 未采齐全部曝光帧，本次未保存图片"
                : "相机 1 未采齐全部曝光帧：" + err);
        }
        if (frames2.size() != exposures2.size()) {
            const auto err = camera2.lastError().message;
            throw std::runtime_error(err.empty()
                ? "相机 2 未采齐全部曝光帧，本次未保存图片"
                : "相机 2 未采齐全部曝光帧：" + err);
        }
        const cv::Mat fused1 = fuseMertens(frames1);
        const cv::Mat fused2 = fuseMertens(frames2);
        const cv::Mat corrected1 = undistortion1 ? undistortion1->apply(fused1) : cv::Mat();
        const cv::Mat corrected2 = undistortion2 ? undistortion2->apply(fused2) : cv::Mat();
        checkCancelled();

        const QString dateDir = QDir(outputDirectory).filePath(captureTime.toString("yyyy-MM-dd"));
        const QString cam1Dir = QDir(dateDir).filePath("cam1");
        const QString cam2Dir = QDir(dateDir).filePath("cam2");
        if (!QDir().mkpath(cam1Dir) || !QDir().mkpath(cam2Dir))
            throw std::runtime_error("无法创建双相机图片保存目录");
        const auto label = naming.pointLabel.isEmpty()
            ? QUuid::createUuid().toString(QUuid::WithoutBraces).left(8) : naming.pointLabel;
        const QString filename = "capture_" +
            captureTime.toString("yyyyMMdd_HHmmss_zzz") + "_" +
            label + "_fused.png";
        const auto layout1 = prepareCameraOutputLayout(cam1Dir, filename, undistortion1, naming.saveOriginal);
        const auto layout2 = prepareCameraOutputLayout(cam2Dir, filename, undistortion2, naming.saveOriginal);
        checkCancelled();

        const QString path1 = saveCaptureImages(fused1, corrected1, layout1.rawPath, layout1.correctedPath, layout1.saveOriginal);
        QString path2;
        try {
            checkCancelled();
            path2 = saveCaptureImages(fused2, corrected2, layout2.rawPath, layout2.correctedPath, layout2.saveOriginal);
        } catch (...) {
            if (layout1.saveOriginal || corrected1.empty()) QFile::remove(layout1.rawPath);
            if (!corrected1.empty()) QFile::remove(layout1.correctedPath);
            throw;
        }
        result.path1 = path1;
        result.path2 = path2;
        const bool anyCorrected = bool(undistortion1) || bool(undistortion2);
        result.message = anyCorrected ? "双相机多曝光融合及去畸变完成，图片已保存"
                                      : "双相机多曝光融合完成，图片已保存";
        try {
            std::array<double, 6> pose{};
            const bool valid = readFlangeFromFuture(poseRead, pose, poseError);
            const double* posePtr = valid ? pose.data() : nullptr;
            QStringList warnings;
            const auto err1 = recordSingleCameraMetadata(
                result.path1, layout1.rawPath, undistortion1, layout1.saveOriginal, posePtr);
            if (!err1.isEmpty()) warnings << "相机 1：" + err1;
            const auto err2 = recordSingleCameraMetadata(
                result.path2, layout2.rawPath, undistortion2, layout2.saveOriginal, posePtr);
            if (!err2.isEmpty()) warnings << "相机 2：" + err2;
            if (!poseError.isEmpty()) warnings << poseError;
            result.complete = valid && warnings.isEmpty();
            if (!warnings.isEmpty()) result.message = warnings.join("；");
        } catch (const std::exception& error) {
            result.message = "图片已保存，法兰位姿记录失败：" + QString::fromUtf8(error.what());
        } catch (...) {
            result.message = "图片已保存，法兰位姿记录发生未知异常";
        }
    } catch (const std::exception& error) {
        result.message = QString::fromUtf8(error.what());
    } catch (...) {
        result.message = "双相机拍照发生未知异常";
    }
    return result;
}
} // namespace collection
