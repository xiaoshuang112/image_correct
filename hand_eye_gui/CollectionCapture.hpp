#pragma once

#include "cameras/AICamera.hpp"
#include <QDateTime>
#include <QString>
#include <array>
#include <atomic>
#include <functional>

namespace collection {
class UndistortionMaps;
struct CaptureResult {
    QString message;
    QString path;
    bool complete = false; // Image and all required metadata committed.
};
struct DualCaptureResult {
    QString message;
    QString path1;
    QString path2;
    bool complete = false; // Both images and all required metadata committed.
};
// Brackets only the multi-exposure grab. Suspend runs after the previous preview
// frame has left the camera; resume runs once the original exposure is restored.
struct ExposureCameraGate {
    std::function<void()> suspend;
    std::function<void()> resume;
};
struct CaptureNaming {
    QString pointLabel;
    bool saveOriginal = true;
};

CaptureResult captureAndSave(AICamera& camera, const QString& outputDirectory,
    const QDateTime& captureTime, const std::vector<int>& exposures,
    const std::shared_ptr<const UndistortionMaps>& undistortion,
    const std::function<std::array<double, 6>()>& readFlange,
    const std::atomic<bool>& cancelled, ExposureCameraGate gate = {}, CaptureNaming naming = {});
DualCaptureResult captureDualAndSave(
    AICamera& camera1, AICamera& camera2,
    const QString& outputDirectory, const QDateTime& captureTime,
    const std::vector<int>& exposures1, const std::vector<int>& exposures2,
    const std::shared_ptr<const UndistortionMaps>& undistortion1,
    const std::shared_ptr<const UndistortionMaps>& undistortion2,
    const std::function<std::array<double, 6>()>& readFlange,
    const std::atomic<bool>& cancelled,
    ExposureCameraGate gate = {}, CaptureNaming naming = {});
void checkCaptureExposures(AICamera& camera, const std::vector<int>& exposures);
} // namespace collection
