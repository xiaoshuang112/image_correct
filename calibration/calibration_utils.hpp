#ifndef CALIBRATION_UTILS_HPP
#define CALIBRATION_UTILS_HPP

#include <string>
#include <vector>
#include <memory>

#include <opencv2/opencv.hpp>

#include "cameras/AICamera.hpp"
#include "charuco_detector.hpp"
#include "det.h"
#include "utils/calib_transform.hpp"
#include "calibration_runtime_status.hpp"

class CalibrationSession;

// 采集候选位姿
struct PoseCandidate {
    double axis_angle[6];   // TCP 轴角位姿 (mm, deg)
    double joint[6];        // 关节角 (deg)
};

/**
 * @brief 断线重连机械臂
 */
int reconnectRobotHandle(int old_handle, char* pchRobotIP);

/**
 * @brief 逐点运动 + 采集
 */
bool executeCollection(int robot_handle,
                       char* pchRobotIP,
                       std::shared_ptr<AICamera>& camera,
                       CharucoDetector& detector,
                       const std::vector<PoseCandidate>& candidates,
                       const std::string& save_dir,
                       AI_det_callback callback,
                       CalibrationSession* session = nullptr,
                       const std::string& robot_pose_dir = "",
                       CalibrationRuntimeStatus* runtime_status = nullptr,
                       const CalibrationRuntimeCallback& runtime_callback = {});

// ============================================================
// 图像/视觉辅助函数
// ============================================================

/**
 * @brief 从 ChArUco 角点计算标定板与图像对应边的夹角 (rad)
 */
double computeBoardAngle(const std::vector<cv::Point2f>& corners,
                         const std::vector<int>& ids,
                         int squares_x, int squares_y,
                         int image_width, int image_height);

/**
 * @brief 计算角点凸包在两个方向上的像素跨度
 * @return 最大方向跨度
 */
float computePixelSpan(const std::vector<cv::Point2f>& corners,
                       float* out_w = nullptr,
                       float* out_h = nullptr);

/**
 * @brief 从两组共同 ID 的角点中，找第一帧最远点对在两帧中的距离
 */
bool computeCommonSpan(const std::vector<cv::Point2f>& corners1,
                       const std::vector<int>& ids1,
                       const std::vector<cv::Point2f>& corners2,
                       const std::vector<int>& ids2,
                       float& dist1, float& dist2);

/**
 * @brief 计算 ChArUco 角点凸包区域的平均深度
 */
double computeAvgDepth(const cv::Mat& depth, const std::vector<cv::Point2f>& corners);

/**
 * @brief 估计相机到标定板的距离（深度相机优先，无深度时回退到双图像法）
 */
bool estimateCameraHeight(std::shared_ptr<AICamera>& camera,
                          CharucoDetector& detector,
                          int robot_handle,
                          const double oriFlangeEulePos[6],
                          cv::Mat& out_frame,
                          std::vector<cv::Point2f>& out_corners,
                          std::vector<int>& out_ids,
                          double& heightCamera);

/**
 * @brief 将法兰 rx=-180, ry=0 旋转到水平姿态
 */
bool rotateFlangeToHorizon(int robot_handle, double* pFlangeEulePos);

/**
 * @brief 将角度限制在 [-90, 90] 范围
 */
float bound_camera_angle(float angle);

// ============================================================
// 位姿加载与转换（眼在手外 / 眼在手上共用）
// ============================================================

/**
 * @brief 从 CSV 加载机器人位姿（法兰轴角 → 4x4）
 */
bool loadRobotPosesFromCSV(const std::string& csv_file,
                            std::vector<cv::Mat>& robot_poses);

/**
 * @brief 将 rob_pos.csv（欧拉角）转换为 rob_axis.csv（轴角）
 */
bool convertEulerCsvToAxisCsv(const std::string& save_dir);

/**
 * @brief 将标定结果保存到 YAML 文件
 * @param yaml_path      输出路径
 * @param transform_key  "T_flange_camera" 或 "T_base_camera"
 * @param transform      4x4 变换矩阵
 * @param camera_matrix  3x3 相机内参
 * @param dist_coeffs    畸变系数
 * @param reproj_error_mm 平移一致性 RMS (mm)；YAML 键名为兼容旧版保留
 * @param valid_frames   有效帧数
 */
bool saveCalibrationResult(const std::string& yaml_path,
                            const std::string& transform_key,
                            const cv::Mat& transform,
                            const cv::Mat& camera_matrix,
                            const cv::Mat& dist_coeffs,
                            double reproj_error_mm,
                            int valid_frames);

#endif // CALIBRATION_UTILS_HPP
