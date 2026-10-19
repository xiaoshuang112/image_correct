#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
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

// Images saved before this output definition have no pixel-frame record.
// Their pixels use the original camera matrix and must keep this name.
inline constexpr char kLegacyOriginalKPixelFrame[] =
    "collect_original_k_undistorted_full_frame";
// Current preview and saved images. Full frame, no validPixROI crop.
inline constexpr char kProductionAlpha1PixelFrame[] =
    "production_alpha1_undistorted_full_frame";

// Parameters of one shared remap. K_new is computed, never a stored constant.
struct UndistortionGeometry {
    cv::Mat originalCameraMatrix;
    cv::Mat computationCameraMatrix;
    cv::Mat distortion;
    cv::Mat outputCameraMatrix;
    cv::Size inputSize;
    cv::Size outputSize;
    double alpha = 1.0;
    cv::Rect validPixROI;
    bool cropped = false;
    bool flipHorizontal = false;
    bool flipVertical = false;
    const char* pixelFrame = kProductionAlpha1PixelFrame;
};

// Immutable maps shared by preview and the capture worker. Output size equals
// the input size. The map uses production alpha=1 new intrinsics.
class UndistortionMaps {
public:
    static constexpr int kMapType = CV_16SC2;
    static constexpr int kInterpolation = cv::INTER_LINEAR;
    static constexpr int kBorder = cv::BORDER_CONSTANT;

    UndistortionMaps(const UndistortionCalibration& calibration,
                     cv::Size size, const std::string& serial);
    cv::Mat apply(const cv::Mat& image) const;
    const UndistortionGeometry& geometry() const { return m_geometry; }
    int fixedPointMapType() const { return m_map1.type(); }
    int interpolationMapType() const { return m_map2.type(); }
private:
    UndistortionGeometry m_geometry;
    cv::Mat m_map1, m_map2;
};

} // namespace collection
