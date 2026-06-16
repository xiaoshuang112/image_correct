#include "charuco_detector.hpp"
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <iomanip>
#include <array>

CharucoDetector::CharucoDetector(const CharucoBoardConfig& config,
                                  const std::string& camera_params_file,
                                  int min_ids, float filter_thresh)
    : config_(config)
    , min_ids_(min_ids)
    , filter_thresh_(filter_thresh) {

    int dict_id;
    switch (config.dictionary_id) {
        case 0:  dict_id = cv::aruco::DICT_4X4_250;  break;
        case 1:  dict_id = cv::aruco::DICT_4X4_1000; break;
        case 2:  dict_id = cv::aruco::DICT_5X5_100;  break;
        case 3:  dict_id = cv::aruco::DICT_5X5_1000; break;
        case 4:  dict_id = cv::aruco::DICT_5X5_250;  break;
        case 5:  dict_id = cv::aruco::DICT_6X6_250;  break;
        default: dict_id = cv::aruco::DICT_5X5_250;  break;
    }

    dictionary_ = cv::makePtr<cv::aruco::Dictionary>(
        cv::aruco::getPredefinedDictionary(dict_id));

#if CV_VERSION_MAJOR >= 4 && CV_VERSION_MINOR >= 7
    board_ = cv::makePtr<cv::aruco::CharucoBoard>(
        cv::Size(config.squares_x, config.squares_y),
        config.square_length, config.marker_length, *dictionary_);
#else
    board_ = cv::aruco::CharucoBoard::create(
        config.squares_x, config.squares_y,
        config.square_length, config.marker_length, dictionary_);
#endif

#if CV_MAJOR_VERSION >= 4 && CV_VERSION_MINOR >= 7
    board_corners_3d_ = board_->getChessboardCorners();
#else
    board_corners_3d_ = board_->chessboardCorners;
#endif

    if (!camera_params_file.empty()) {
        loadCameraParams(camera_params_file);
    }
}

bool CharucoDetector::loadCameraParams(const std::string& camera_params_file) {
    cv::FileStorage fs(camera_params_file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[CharucoDetector] 无法打开相机参数文件: "
                  << camera_params_file << std::endl;
        return false;
    }
    fs["camera_matrix"] >> camera_matrix_;
    fs["distortion_coefficients"] >> dist_coeffs_;
    if (camera_matrix_.empty() || dist_coeffs_.empty()) {
        std::cerr << "[CharucoDetector] 相机参数读取失败" << std::endl;
        return false;
    }
    return true;
}

bool CharucoDetector::detect(const cv::Mat& image_gray,
                              std::vector<cv::Point2f>& corners,
                              std::vector<int>& ids,
                              std::vector<std::vector<cv::Point2f>>& markerCorners,
                              std::vector<int>& markerIds) const {

    // 先检测 ArUco 标记（不依赖相机内参，更鲁棒）
    cv::Ptr<cv::aruco::DetectorParameters> dp = cv::makePtr<cv::aruco::DetectorParameters>();
    dp->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    cv::aruco::detectMarkers(image_gray, dictionary_, markerCorners, markerIds, dp);

    // 再插值 ChArUco 角点（可选使用相机内参去畸变）
    if (!markerIds.empty()) {
        if (!camera_matrix_.empty())
            cv::aruco::interpolateCornersCharuco(
                markerCorners, markerIds, image_gray, board_,
                corners, ids, camera_matrix_, dist_coeffs_, 1);
        else
            cv::aruco::interpolateCornersCharuco(
                markerCorners, markerIds, image_gray, board_,
                corners, ids, cv::noArray(), cv::noArray(), 1);
    }

    // 尝试用 CharucoDetector 作为补充（某些场景可能检测到更多）
    if (ids.empty()) {
        cv::aruco::DetectorParameters detectorParams;
        cv::aruco::CharucoParameters charucoParams;
        detectorParams.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        charucoParams.minMarkers = 1;
        charucoParams.tryRefineMarkers = false;
        if (!camera_matrix_.empty()) {
            charucoParams.cameraMatrix = camera_matrix_;
            charucoParams.distCoeffs = dist_coeffs_;
        }

        std::vector<cv::Point2f> refine_corners;
        std::vector<int> refine_ids;
        std::vector<std::vector<cv::Point2f>> refine_markerCorners;
        std::vector<int> refine_markerIds;
        try {
            cv::aruco::CharucoDetector refine_detector(*board_, charucoParams, detectorParams);
            refine_detector.detectBoard(image_gray, refine_corners, refine_ids,
                                        refine_markerCorners, refine_markerIds);
        } catch (...) {
            // 某些版本可能抛异常
        }

        if (!refine_ids.empty()) {
            corners = std::move(refine_corners);
            ids = std::move(refine_ids);
            if (markerCorners.empty())
                markerCorners = std::move(refine_markerCorners);
            if (markerIds.empty())
                markerIds = std::move(refine_markerIds);
        }
    }

    return !ids.empty();
}

bool CharucoDetector::detectCorners(const cv::Mat& image,
                                     std::vector<cv::Point2f>& corners,
                                     std::vector<int>& ids) const {
    cv::Mat gray;
    if (image.channels() == 3)
        cv::extractChannel(image, gray, 0);  // 取 B 通道（黑白相机 Mono8→BGR24 时 B 通道即是原始灰度数据）
    else
        gray = image.clone();

    std::vector<std::vector<cv::Point2f>> markerCorners;
    std::vector<int> markerIds;
    return detect(gray, corners, ids, markerCorners, markerIds);
}

void CharucoDetector::filterCorners(std::vector<cv::Point2f>& corners,
                                     std::vector<int>& ids,
                                     const cv::Mat& rvec,
                                     const cv::Mat& tvec,
                                     float thresh,
                                     cv::Mat* viz_image) const {
    std::vector<cv::Point3f> object_points;
    object_points.reserve(ids.size());
    for (size_t i = 0; i < ids.size(); ++i) {
        int id = ids[i];
        if (id >= 0 && static_cast<size_t>(id) < board_corners_3d_.size())
            object_points.push_back(board_corners_3d_[static_cast<size_t>(id)]);
    }
    if (object_points.size() != corners.size() || object_points.empty())
        return;

    std::vector<cv::Point2f> projected;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix_, dist_coeffs_, projected);

    double sum = 0.0, sum_sq = 0.0, max_e = 0.0;
    std::vector<float> vError;
    vError.reserve(projected.size());
    for (size_t i = 0; i < projected.size(); ++i) {
        double e = cv::norm(corners[i] - projected[i]);
        sum += e;
        sum_sq += e * e;
        max_e = std::max(max_e, e);
        vError.push_back((float)e);
        if (viz_image) {
            cv::circle(*viz_image, projected[i], 3, cv::Scalar(0, 0, 255), -1);
            cv::line(*viz_image, corners[i], projected[i], cv::Scalar(0, 255, 255), 1);
        }
    }

    if (viz_image) {
        double mean = sum / projected.size();
        double rms = std::sqrt(sum_sq / projected.size());
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2);
        oss << "Reproj(px) mean=" << mean << " rms=" << rms << " max=" << max_e;
        cv::putText(*viz_image, oss.str(), cv::Point(10, 120),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
    }

    if (thresh > 0.001f) {
        for (int i = (int)ids.size() - 1; i >= 0; --i) {
            if (vError[i] >= thresh) {
                ids.erase(ids.begin() + i);
                corners.erase(corners.begin() + i);
            }
        }
    }
}

bool CharucoDetector::detectPose(const cv::Mat& image,
                                  cv::Mat& rvec, cv::Mat& tvec,
                                  std::vector<cv::Point2f>& corners,
                                  std::vector<int>& ids,
                                  int min_corners) const {
    if (!detectCorners(image, corners, ids)) return false;
    if ((int)ids.size() < min_corners) return false;

    auto threshes = (image.rows == 800) ? std::array{0.35f, 0.25f}
                                         : std::array{0.6f, 0.4f};

    if (cv::aruco::estimatePoseCharucoBoard(corners, ids, board_,
                                             camera_matrix_, dist_coeffs_,
                                             rvec, tvec)) {
        filterCorners(corners, ids, rvec, tvec, threshes[0], nullptr);
        if ((int)ids.size() < min_corners) return false;
    }
    if (cv::aruco::estimatePoseCharucoBoard(corners, ids, board_,
                                             camera_matrix_, dist_coeffs_,
                                             rvec, tvec)) {
        filterCorners(corners, ids, rvec, tvec, threshes[1], nullptr);
        if ((int)ids.size() < min_corners) return false;
    }
    return cv::aruco::estimatePoseCharucoBoard(corners, ids, board_,
                                                camera_matrix_, dist_coeffs_,
                                                rvec, tvec);
}

bool CharucoDetector::detectAndDraw(const cv::Mat& image,
                                     cv::Mat& image_with_detection,
                                     cv::Mat& rvec, cv::Mat& tvec,
                                     int* out_corner_count,
                                     int override_min_ids) const {

    cv::Mat gray;
    if (image.channels() == 3)
        cv::extractChannel(image, gray, 0);  // 取 B 通道（黑白相机 Mono8→BGR24 时 B 通道即是原始灰度数据）
    else
        gray = image.clone();

    std::vector<cv::Point2f> corners;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    std::vector<int> markerIds;
    detect(gray, corners, ids, markerCorners, markerIds);

    image.copyTo(image_with_detection);
    cv::aruco::drawDetectedMarkers(image_with_detection, markerCorners, markerIds);

    int minIds = (override_min_ids > 0) ? override_min_ids : min_ids_;
    if ((int)ids.size() < minIds) {
        if (out_corner_count) *out_corner_count = 0;
        return false;
    }

    cv::aruco::drawDetectedCornersCharuco(image_with_detection, corners, ids);

    // 需要相机内参才能做位姿估计和重投影过滤
    if (camera_matrix_.empty()) {
        if (out_corner_count) *out_corner_count = (int)corners.size();
        return true;
    }

    bool valid = cv::aruco::estimatePoseCharucoBoard(corners, ids, board_,
                                                      camera_matrix_, dist_coeffs_,
                                                      rvec, tvec);
    if (!valid) {
        if (out_corner_count) *out_corner_count = (int)corners.size();
        return true;
    }

    filterCorners(corners, ids, rvec, tvec, filter_thresh_, &image_with_detection);
    if ((int)corners.size() >= std::max(6, minIds))
        cv::aruco::estimatePoseCharucoBoard(corners, ids, board_,
                                             camera_matrix_, dist_coeffs_,
                                             rvec, tvec);

    if (out_corner_count) *out_corner_count = (int)corners.size();
    return true;
}
