#ifndef AUTO_EYE_IN_HAND_CHARUCO_CALIBRATION_HPP
#define AUTO_EYE_IN_HAND_CHARUCO_CALIBRATION_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "cameras/AICamera.hpp"
#include "charuco_detector.hpp"
#include "camera_calibration.hpp"

#include "robotapi.h"
#include "basestruct.h"
#include "utils/robot_util.hpp"

#include "det.h"

// 自动眼在手 ChArUco 标定结果
struct CalibrationResult {
    bool success = false;
    cv::Mat T_flange_camera;   // 4x4 手眼矩阵（法兰→相机）
    cv::Mat camera_matrix;     // 3x3 相机内参（标定或输入值）
    cv::Mat dist_coeffs;      // 畸变系数
    int valid_frames = 0;      // 参与标定的有效帧数
    // 基座坐标系下标定板平移一致性 RMS（m）。
    // 字段名为兼容旧接口保留，不是像素重投影误差。
    double reproj_error = 0.0;
    std::string message;       // 状态/错误信息
    std::string session_dir;   // 本次标定审计 Session 目录
};

/**
 * @brief 自动眼在手 ChArUco 标定（CGXi SDK）
 *
 * 从起始轴角位姿出发，生成约 360 个兼顾高度、平移和旋转覆盖的候选位姿，
 * IK 预过滤并按最近关节路径排序后逐点运动采集，自动完成手眼标定。
 * 如 camera_matrix 为空，从采集帧自动标定相机内参。
 *
 * @param robot_handle    CGXi RobotHandle（来自 cr_create_robot）
 * @param init_tcp_pose   起始 TCP 轴角位姿 [x,y,z,rx,ry,rz] (mm, deg)
 * @param tcp_offset      TCP 偏移量 [x,y,z,rx,ry,rz] (mm, deg)
 * @param camera          已打开的 OrbbecCameraV2
 * @param board_config    ChArUco 标定板参数
 * @param camera_matrix   输入：已知内参 / 空 Mat 则自动标定；输出：最终内参
 * @param dist_coeffs     同上
 * @param result          标定结果
 * @param save_dir        数据保存目录（为空则不保存）。保存内容：
 *                        - 000.jpg, 001.jpg, ... 采集图像
 *                        - rob_pos.csv 法兰轴角位姿 (mm,deg)
 *                        - rob_joint.csv 关节角 (deg)
 * @return true/false
 */
bool autoEyeInHandCharucoCalibration(
    int robot_handle,
    char* pchRobotIP,
    std::shared_ptr<AICamera>& camera,
    const CharucoBoardConfig& board_config,
    cv::InputOutputArray camera_matrix,
    cv::InputOutputArray dist_coeffs,
    int cameraToTcpEnd,
    CalibrationResult& result,
    const std::string& save_dir = "",
    const std::string& output_yaml = "",
    AI_det_callback callback = nullptr,
    const std::string& intrinsic_source_file = ""
);

#endif // AUTO_EYE_IN_HAND_CHARUCO_CALIBRATION_HPP
