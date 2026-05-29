#ifndef CHARUCO_DETECTOR_HPP
#define CHARUCO_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <string>
#include <vector>

// ChArUco 标定板参数
struct CharucoBoardConfig {
    int squares_x = 7;             // 横向方格数
    int squares_y = 5;             // 纵向方格数
    float square_length = 0.04f;   // 方格边长 (m)
    float marker_length = 0.028f;  // 标记边长 (m)
    int dictionary_id = 4;         // cv::aruco::DICT_5X5_250
};

// ChArUco 标定板检测封装
// 三个层级: detectCorners(无需内参) → detectPose(需内参,两级过滤) → detectAndDraw(带可视化)
class CharucoDetector {
public:
    CharucoDetector(const CharucoBoardConfig& config,
                    const std::string& camera_params_file = "",
                    int min_ids = 12,
                    float filter_thresh = 0.25f);

    // Level 1: 仅检测角点（无需相机内参）
    bool detectCorners(const cv::Mat& image,
                       std::vector<cv::Point2f>& corners,
                       std::vector<int>& ids) const;

    // Level 2: 检测 + 位姿估计 + 两级重投影过滤（需相机内参）
    bool detectPose(const cv::Mat& image,
                    cv::Mat& rvec, cv::Mat& tvec,
                    std::vector<cv::Point2f>& corners,
                    std::vector<int>& ids,
                    int min_corners = 12) const;

    // Level 3: 检测 + 位姿 + 可视化绘制（需相机内参，用于实时预览）
    bool detectAndDraw(const cv::Mat& image,
                       cv::Mat& image_with_detection,
                       cv::Mat& rvec, cv::Mat& tvec,
                       int* out_corner_count = nullptr,
                       int override_min_ids = 0) const;

    // 设置/更新相机内参（构造时未加载时使用）
    void setCameraParams(const cv::Mat& camera_matrix,
                         const cv::Mat& dist_coeffs) {
        camera_matrix_ = camera_matrix;
        dist_coeffs_ = dist_coeffs;
    }

    const cv::Mat& getCameraMatrix() const { return camera_matrix_; }
    const cv::Mat& getDistCoeffs() const { return dist_coeffs_; }

    cv::Size getChessboardSize() const {
#if CV_VERSION_MAJOR >= 4 && CV_VERSION_MINOR >= 7
        return board_->getChessboardSize();
#else
        return board_->checkboradSize;
#endif
    }

    const CharucoBoardConfig& getConfig() const { return config_; }

    const std::vector<cv::Point3f>& getChessboardCorners3D() const {
        return board_corners_3d_;
    }

private:
    bool loadCameraParams(const std::string& camera_params_file);

    // 核心检测（无可视化，各 Level 共用）
    bool detect(const cv::Mat& image_gray,
                std::vector<cv::Point2f>& corners,
                std::vector<int>& ids,
                std::vector<std::vector<cv::Point2f>>& markerCorners,
                std::vector<int>& markerIds) const;

    // 重投影过滤（可选绘制可视化）
    void filterCorners(std::vector<cv::Point2f>& corners,
                       std::vector<int>& ids,
                       const cv::Mat& rvec,
                       const cv::Mat& tvec,
                       float thresh,
                       cv::Mat* viz_image = nullptr) const;

    CharucoBoardConfig config_;
    int min_ids_;
    float filter_thresh_;

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::CharucoBoard> board_;
    std::vector<cv::Point3f> board_corners_3d_;
};

#endif // CHARUCO_DETECTOR_HPP
