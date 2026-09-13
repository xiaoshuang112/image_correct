#ifndef EYE_TO_HAND_CALIBRATION_HPP
#define EYE_TO_HAND_CALIBRATION_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>

#include "cameras/AICamera.hpp"
#include "charuco_detector.hpp"
#include "robotapi.h"
#include "basestruct.h"

#include "det.h"
#include "calibration_runtime_status.hpp"

// 眼在手外标定结果
struct EyeToHandResult {
    bool success = false;
    cv::Mat T_base_camera;       // 4x4 相机在基坐标系下的位姿
    cv::Mat camera_matrix;       // 3x3 相机内参
    cv::Mat dist_coeffs;        // 畸变系数
    int valid_frames = 0;        // 参与标定的有效帧数
    double reproj_error = 0.0;   // 法兰坐标系下标定板平移一致性 RMS（m），不是像素误差
    std::string message;
    std::string data_directory; // 本次采集目录，或离线重放的输入目录
    CalibrationRuntimeStatus quality;
};

// 无硬件求解：G=T_base_flange, C=T_camera_board（平移单位 m）。
// 返回 X=T_base_camera，使 inv(G_i)*X*C_i 为固定的 T_flange_board。
// 复用眼在手内的五算法共识、MAD 和质量门限；失败清空旧解。
bool solveEyeToHandPoses(const std::vector<cv::Mat>& robot_poses,
                        const std::vector<cv::Mat>& camera_poses,
                        EyeToHandResult& result);

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
    int maximum_candidates = 90;
    double maximum_joint_step_degrees = 60.0;
    std::string source_file;
};

inline bool tryLoadPoseGenerationConfig(const std::string& yaml_path,
                                        PoseGenerationConfig& cfg,
                                        std::string& error) {
    error.clear();
    cv::FileStorage fs;
    try {
        fs.open(yaml_path, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            error = "cannot open pose generation config: " + yaml_path;
            return false;
        }
        cv::FileNode fn;
        fn = fs["offset_x"]; if (!fn.empty()) fn >> cfg.offset_x;
        fn = fs["offset_y"]; if (!fn.empty()) fn >> cfg.offset_y;
        fn = fs["offset_z"]; if (!fn.empty()) fn >> cfg.offset_z;
        fn = fs["offset_rx"]; if (!fn.empty()) fn >> cfg.offset_rx;
        fn = fs["offset_ry"]; if (!fn.empty()) fn >> cfg.offset_ry;
        fn = fs["offset_rz"]; if (!fn.empty()) fn >> cfg.offset_rz;
        fn = fs["min_corners"]; if (!fn.empty()) fn >> cfg.min_corners;
        fn = fs["min_valid_frames"]; if (!fn.empty()) fn >> cfg.min_valid_frames;
        fn = fs["maximum_candidates"]; if (!fn.empty()) fn >> cfg.maximum_candidates;
        fn = fs["maximum_joint_step_degrees"];
        if (!fn.empty()) fn >> cfg.maximum_joint_step_degrees;
        fs.release();
    } catch (const cv::Exception& exception) {
        error = "invalid pose generation config " + yaml_path + ": " +
                exception.what();
        return false;
    }

    const std::vector<const std::vector<double>*> offsets{
        &cfg.offset_x, &cfg.offset_y, &cfg.offset_z,
        &cfg.offset_rx, &cfg.offset_ry, &cfg.offset_rz};
    for (const std::vector<double>* values : offsets) {
        if (values->empty() ||
            !std::all_of(values->begin(), values->end(),
                         [](double value) { return std::isfinite(value); })) {
            error = "pose generation offsets must be non-empty and finite";
            return false;
        }
    }
    if (cfg.min_corners <= 0 || cfg.min_valid_frames <= 0) {
        error = "min_corners and min_valid_frames must be positive";
        return false;
    }
    if (cfg.maximum_candidates <= 0) {
        error = "maximum_candidates must be positive";
        return false;
    }
    if (!std::isfinite(cfg.maximum_joint_step_degrees) ||
        cfg.maximum_joint_step_degrees <= 0.0 ||
        cfg.maximum_joint_step_degrees > 180.0) {
        error = "maximum_joint_step_degrees must be in (0, 180]";
        return false;
    }
    std::cout << "加载配置: " << yaml_path << std::endl;
    cfg.source_file = yaml_path;
    return true;
}

inline PoseGenerationConfig loadPoseGenerationConfig(const std::string& yaml_path) {
    PoseGenerationConfig cfg;
    std::string error;
    if (!tryLoadPoseGenerationConfig(yaml_path, cfg, error))
        std::cerr << "加载位姿配置失败，使用默认参数: " << error << std::endl;
    return cfg;
}

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
