#ifndef AUTO_EYE_IN_HAND_CHARUCO_CALIBRATION_HPP
#define AUTO_EYE_IN_HAND_CHARUCO_CALIBRATION_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "cameras/AICamera.hpp"
#include "charuco_detector.hpp"

#include "robotapi.h"
#include "basestruct.h"
#include "utils/robot_util.hpp"

#ifdef XIALIAO
#include "det.h"
#else
typedef int (*AI_det_callback)(char* msg, int msg_len, int flag);
#endif // XIALIAO

// 自动眼在手 ChArUco 标定结果
struct CalibrationResult {
    bool success = false;
    cv::Mat T_flange_camera;   // 4x4 手眼矩阵（法兰→相机）
    cv::Mat camera_matrix;     // 3x3 相机内参（标定或输入值）
    cv::Mat dist_coeffs;      // 畸变系数
    int valid_frames = 0;      // 参与标定的有效帧数
    double reproj_error = 0.0; // 重投影误差
    std::string message;       // 状态/错误信息
};

/**
 * @brief 自动眼在手 ChArUco 标定（CGXi SDK）
 *
 * 从起始轴角位姿出发，在欧拉空间生成 729（3⁶）个候选位姿组合，
 * IK 预过滤后逐点运动采集，自动完成手眼标定。
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
    AI_det_callback callback=nullptr
);

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
    const std::string& output_yaml = "");

bool calibrateCameraFromDisk(const std::string& save_dir,
                             CharucoDetector& detector,
                             cv::Mat& camera_matrix,
                             cv::Mat& dist_coeffs);


#endif // AUTO_EYE_IN_HAND_CHARUCO_CALIBRATION_HPP
