#ifndef SEMI_AUTO_EYE_IN_HAND_CALIBRATION_HPP
#define SEMI_AUTO_EYE_IN_HAND_CALIBRATION_HPP

#include <opencv2/opencv.hpp>
#include <string>

#include "charuco_detector.hpp"

#include "auto_eye_in_hand_charuco_calibration.hpp"  // CalibrationResult
#include "calibration_runtime_status.hpp"

class CalibrationSession;

/**
 * @brief 从当前采集会话的已保存数据求解手眼标定
 *
 * 读取 save_dir 下的图像和 rob_pos.csv，检测 ChArUco 角点后执行手眼标定。
 * 需要预先提供相机内参。
 *
 * @param save_dir        数据目录（内含 *.jpg + rob_pos.csv）
 * @param camera_matrix   相机内参矩阵 3x3
 * @param dist_coeffs     畸变系数 5x1
 * @param board_config    ChArUco 标定板参数
 * @param result          标定结果
 * @return true/false
 */
bool calibrateHandEyeFromDisk(
    const std::string& save_dir,
    cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    const CharucoBoardConfig& board_config,
    CalibrationResult& result,
    const std::string& output_yaml,
    CalibrationSession& session,
    CalibrationRuntimeStatus* runtime_status = nullptr,
    const CalibrationRuntimeCallback& runtime_callback = {});

#endif // SEMI_AUTO_EYE_IN_HAND_CALIBRATION_HPP
