#ifndef SEMI_AUTO_EYE_IN_HAND_CALIBRATION_HPP
#define SEMI_AUTO_EYE_IN_HAND_CALIBRATION_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <memory>

#include "cameras/AICamera.hpp"
#include "charuco_detector.hpp"

#include "eye_to_hand_calibration.hpp"           // PoseGenerationConfig
#include "auto_eye_in_hand_charuco_calibration.hpp"  // CalibrationResult

#include "det.h"

class CalibrationSession;

/**
 * @brief 半自动眼在手内 ChArUco 标定（在线）
 *
 * 用户示教到起始位姿 → 自动生成 N 个偏移位姿 → 逐点采集 → 标定相机内参和手眼矩阵。
 *
 * @param robot_handle  CGXi RobotHandle
 * @param pchRobotIP    机器人 IP
 * @param camera        已打开的相机
 * @param detector      ChArUco 检测器
 * @param pose_config   位姿生成配置（偏移量组合）
 * @param board_config  ChArUco 标定板参数
 * @param camera_matrix 输入已知内参 / 空则自动标定；输出最终内参
 * @param dist_coeffs   同上
 * @param result        标定结果（T_flange_camera）
 * @param save_dir      数据保存目录
 * @param output_yaml   结果 YAML 输出路径
 * @param callback      进度回调
 * @return true/false
 */
bool semiAutoEyeInHandCalibration(
    int robot_handle,
    char* pchRobotIP,
    std::shared_ptr<AICamera>& camera,
    CharucoDetector& detector,
    const PoseGenerationConfig& pose_config,
    const CharucoBoardConfig& board_config,
    cv::InputOutputArray camera_matrix,
    cv::InputOutputArray dist_coeffs,
    CalibrationResult& result,
    const std::string& save_dir = "",
    const std::string& output_yaml = "",
    AI_det_callback callback = nullptr);

/**
 * @brief 半自动眼在手内 ChArUco 标定（离线）
 *
 * 读取已采集的磁盘数据，重新标定相机内参和手眼矩阵。
 *
 * @param save_dir        采集数据目录（内含 *.jpg + rob_pos.csv）
 * @param board_config    ChArUco 标定板参数
 * @param camera_matrix   输入已知内参 / 空则自动标定；输出最终内参
 * @param dist_coeffs     同上
 * @param result          标定结果（T_flange_camera）
 * @param output_yaml     结果 YAML 输出路径
 * @return true/false
 */
bool calibrateEyeInHandFromDisk(
    const std::string& save_dir,
    const CharucoBoardConfig& board_config,
    cv::InputOutputArray camera_matrix,
    cv::InputOutputArray dist_coeffs,
    CalibrationResult& result,
    const std::string& output_yaml = "");

/**
 * @brief 从磁盘保存的数据重新运行手眼标定（无需连接机械臂）
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
    const std::string& output_yaml = "",
    CalibrationSession* session = nullptr,
    const std::string& intrinsic_source_file = "");

#endif // SEMI_AUTO_EYE_IN_HAND_CALIBRATION_HPP
