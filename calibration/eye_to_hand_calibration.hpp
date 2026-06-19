#ifndef EYE_TO_HAND_CALIBRATION_HPP
#define EYE_TO_HAND_CALIBRATION_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <memory>

#include "cameras/AICamera.hpp"
#include "charuco_detector.hpp"
#include "robotapi.h"
#include "basestruct.h"

#include "det.h"

// 眼在手外标定结果
struct EyeToHandResult {
    bool success = false;
    cv::Mat T_base_camera;       // 4x4 相机在基坐标系下的位姿
    cv::Mat camera_matrix;       // 3x3 相机内参
    cv::Mat dist_coeffs;        // 畸变系数
    int valid_frames = 0;        // 参与标定的有效帧数
    double reproj_error = 0.0;   // 重投影误差（平移一致性）
    std::string message;
};

// 位姿生成配置
struct PoseGenerationConfig {
    std::vector<double> offset_x{300, 0, -300};
    std::vector<double> offset_y{300, 0, -300};
    std::vector<double> offset_z{100, 0, -100};
    std::vector<double> offset_rx{10, 0, -10};
    std::vector<double> offset_ry{10, 0, -10};
    std::vector<double> offset_rz{10, 0, -10};

    int min_corners = 12;
    int min_valid_frames = 10;
};

/**
 * @brief 自动眼在手外 ChArUco 标定
 *
 * 半自动采集：示教器定位原点 → 自动生成 N 个偏移位姿 → 逐点采集 → 标定
 *
 * @param robot_handle  CGXi RobotHandle
 * @param pchRobotIP    机器人 IP
 * @param camera        已打开的相机
 * @param detector      ChArUco 检测器
 * @param pose_config   位姿生成配置（偏移量、过滤参数）
 * @param result        标定结果
 * @param save_dir      数据保存目录（为空则不保存）
 * @return true/false
 */
bool autoEyeToHandCharucoCalibration(
    int robot_handle,
    char* pchRobotIP,
    std::shared_ptr<AICamera>& camera,
    CharucoDetector& detector,
    const PoseGenerationConfig& pose_config,
    EyeToHandResult& result,
    const std::string& save_dir = "",
    AI_det_callback callback = nullptr);

/**
 * @brief 从已采集的本地数据标定相机内参 + 眼在手外手眼矩阵
 *
 * 读取 save_dir 下的采集数据（图像、机器人位姿），依次执行：
 *   Phase 3.5 — 从 ChArUco 图像标定相机内参
 *   Phase 3.75 — 用标定后的内参计算每帧标定板位姿
 *   Phase 4   — 手眼标定
 *
 * @param save_dir  采集数据目录（包含 000.jpg、rob_pos.csv 等）
 * @param result    标定结果
 * @return true/false
 */
bool calibrateEyeToHandFromDisk(const std::string& save_dir,
                                 EyeToHandResult& result);

#endif // EYE_TO_HAND_CALIBRATION_HPP
