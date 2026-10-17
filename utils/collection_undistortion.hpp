#pragma once

#include <opencv2/core.hpp>
#include <string>

namespace collection {

// Only the camera intrinsics are used here; the robot hand-eye transform does
// not describe lens distortion. Legacy result files may omit camera identity.
struct UndistortionCalibration {
    cv::Mat cameraMatrix;
    cv::Mat distortion;
    cv::Size calibratedSize;
    std::string cameraSerial;

    static UndistortionCalibration load(const std::string& path);
    void validateCamera(cv::Size size, const std::string& serial) const;
};

// Immutable maps shared by preview and the capture worker. Keep the original
// camera matrix and image dimensions: no automatic crop or change of scale.
class UndistortionMaps {
public:
    UndistortionMaps(const UndistortionCalibration& calibration,
                     cv::Size size, const std::string& serial);
    cv::Mat apply(const cv::Mat& image) const;
private:
    cv::Size m_size;
    cv::Mat m_map1, m_map2;
};

} // namespace collection
