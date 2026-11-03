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

CaptureResult captureAndSave(AICamera& camera, const QString& outputDirectory,
    const QDateTime& captureTime, const std::vector<int>& exposures,
    const std::shared_ptr<const UndistortionMaps>& undistortion,
    const std::function<std::array<double, 6>()>& readFlange,
    const std::atomic<bool>& cancelled);
void checkCaptureExposures(AICamera& camera, const std::vector<int>& exposures);
} // namespace collection
