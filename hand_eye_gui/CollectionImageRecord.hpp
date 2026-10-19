#pragma once

#include <QString>
#include <opencv2/core.hpp>
#include "utils/collection_undistortion.hpp"

namespace collection {

// Save a raw fused image and, when supplied, a corrected image beside it.
// Return the corrected path when enabled, otherwise the raw path. Failure
// removes files created by this operation; existing files are never replaced.
QString saveCaptureImages(const cv::Mat& raw, const cv::Mat& corrected,
                          const QString& rawPath);

// Sidecar for the corrected image only. Does not replace an existing file and
// does not modify the image or its pose JSON. Empty string means success.
QString saveUndistortionMetadata(const QString& correctedImagePath,
                                 const UndistortionGeometry& geometry);

} // namespace collection
