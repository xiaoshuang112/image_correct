#include "CollectionCapture.hpp"
#include "CollectionImageRecord.hpp"
#include "CollectionPoseRecord.hpp"
#include "utils/collection_undistortion.hpp"
#include "utils/collection_utils.hpp"
#include <QDir>
#include <QUuid>
#include <QStringList>
#include <algorithm>
#include <cmath>
#include <future>
#include <stdexcept>

namespace collection {
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
    const std::atomic<bool>& cancelled) {
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
        const auto frames = camera.captureMultiExposureImages(exposures);
        checkCancelled();
        if (frames.size() != exposures.size())
            throw std::runtime_error("未采齐全部曝光帧，本次未保存图片");
        const auto fused = fuseMertens(frames);
        const cv::Mat corrected = undistortion ? undistortion->apply(fused) : cv::Mat();
        checkCancelled();
        const QString directory = QDir(outputDirectory).filePath(captureTime.toString("yyyy-MM-dd"));
        if (!QDir().mkpath(directory)) throw std::runtime_error("无法创建图片保存目录");
        const QString rawPath = QDir(directory).filePath("capture_" +
            captureTime.toString("yyyyMMdd_HHmmss_zzz") + "_" +
            QUuid::createUuid().toString(QUuid::WithoutBraces).left(8) + "_fused.png");
        checkCancelled();
        result.path = saveCaptureImages(fused, corrected, rawPath);
        result.message = undistortion ? "多曝光融合及去畸变完成，校正图与原始图已保存"
                                     : "多曝光融合完成，图片已保存";
        // Keep manual captures even when pose/metadata fails; automatic motion
        // may continue only when complete is true. Wait for the RPC before returning.
        try {
            std::array<double, 6> pose{};
            bool valid = false;
            if (poseRead.valid()) {
                try {
                    pose = poseRead.get();
                    valid = std::all_of(pose.begin(), pose.end(), [](double v) { return std::isfinite(v); });
                    if (!valid) poseError = "法兰位姿包含无效数值，本张位姿为空";
                } catch (const std::exception& error) {
                    poseError = "法兰位姿读取异常：" + QString::fromUtf8(error.what());
                } catch (...) { poseError = "法兰位姿读取发生未知异常"; }
            }
            QStringList warnings;
            const auto recordError = saveFlangePoseRecords(result.path, valid ? pose.data() : nullptr);
            if (!recordError.isEmpty()) warnings << recordError;
            if (undistortion) {
                const auto metadataError = saveUndistortionMetadata(result.path, undistortion->geometry());
                if (!metadataError.isEmpty()) warnings << metadataError;
                const auto rawError = saveFlangePoseRecords(rawPath, valid ? pose.data() : nullptr);
                if (!rawError.isEmpty()) warnings << rawError;
            }
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
} // namespace collection
