#ifndef CAMERA_CALIBRATION_HPP
#define CAMERA_CALIBRATION_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "charuco_detector.hpp"

/**
 * @brief 从磁盘读取图像文件列表，检测图像尺寸
 *
 * @param save_dir    图像目录
 * @param img_files   [out] 排序后的 .jpg 文件路径列表
 * @param image_size  [out] 图像尺寸
 * @return true/false
 */
bool loadImagesFromDisk(const std::string& save_dir,
                        std::vector<std::string>& img_files,
                        cv::Size& image_size);

/**
 * @brief 多轮筛选：分批标定 → 取误差最小的 wantedNum 帧
 *
 * @return true/false
 */
bool filterDataForCameraCalibration(std::vector<std::vector<cv::Point3f>>& object_points,
    std::vector<std::vector<cv::Point2f>>& image_points,
    const cv::Size& image_size,
    cv::Mat& camera_matrix,
    cv::Mat& dist_coeffs, int wantedNum);

/**
 * @brief 从已采集的 ChArUco 图像标定相机内参
 *
 * @param save_dir       采集数据目录（包含 000.jpg ...）
 * @param detector       ChArUco 检测器
 * @param camera_matrix  [out] 相机内参矩阵 3x3
 * @param dist_coeffs    [out] 畸变系数 5x1
 * @return true/false
 */
bool calibrateCameraFromDisk(const std::string& save_dir,
                             CharucoDetector& detector,
                             cv::Mat& camera_matrix,
                             cv::Mat& dist_coeffs);

#endif // CAMERA_CALIBRATION_HPP
