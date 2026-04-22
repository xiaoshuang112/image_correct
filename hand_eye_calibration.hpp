#ifndef HAND_EYE_CALIBRATION_HPP
#define HAND_EYE_CALIBRATION_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <vector>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iomanip>
#include <filesystem>

class VectorWriter {
private:
    std::string filename;
    int precision = 3;
    char delimiter = ',';  // 分隔符，默认为逗号
    
public:
    VectorWriter(const std::string& fname, int prec = 6, char delim = ',') 
        : filename(fname), precision(prec), delimiter(delim) {}
    
    // 写入单个vector到文件
    bool writeDoubleVector(const std::vector<double>& vec) {
        std::ofstream file(filename);
        if (!file) {
            return false;
        }
        
        file << std::fixed << std::setprecision(precision);
        
        for (size_t i = 0; i < vec.size(); ++i) {
            file << vec[i];
            if (i < vec.size() - 1) {
                file << delimiter;
            }
        }
        
        return true;
    }
    
    // 追加模式写入vector
    bool appendDoubleVector(const std::vector<double>& vec) {
        std::ofstream file(filename, std::ios::app);
        if (!file) {
            return false;
        }
        
        // 如果文件不为空，先添加换行符
        file.seekp(0, std::ios::end);
        if (file.tellp() > 0) {
            file << "\n";
        }
        
        file << std::fixed << std::setprecision(precision);
        for (size_t i = 0; i < vec.size(); ++i) {
            file << vec[i];
            if (i < vec.size() - 1) {
                file << delimiter;
            }
        }
        
        return true;
    }
    
    // 写入多个vector（每个vector一行）
    bool writeMultipleVectors(const std::vector<std::vector<double>>& vectors) {
        std::ofstream file(filename);
        if (!file) {
            return false;
        }
        
        file << std::fixed << std::setprecision(precision);
        
        for (size_t i = 0; i < vectors.size(); ++i) {
            for (size_t j = 0; j < vectors[i].size(); ++j) {
                file << vectors[i][j];
                if (j < vectors[i].size() - 1) {
                    file << delimiter;
                }
            }
            if (i < vectors.size() - 1) {
                file << "\n";
            }
        }
        
        return true;
    }
    
    // 从文本文件按行读取向量到 vector<vector<double>>
    std::vector<std::vector<double>> readVectorsFromFile() {
        std::vector<std::vector<double>> result;
        std::ifstream file(filename);
        
        if (!file.is_open()) {
            std::cerr << "无法打开文件: " << filename << std::endl;
            return result;
        }
        
        std::string line;
        int lineNum = 0;
        
        while (std::getline(file, line)) {
            lineNum++;
            
            // 跳过空行
            if (line.empty()) {
                continue;
            }
            
            std::vector<double> vec;
            std::stringstream ss(line);
            std::string token;
            
            // 按分隔符读取每个数字
            while (std::getline(ss, token, delimiter)) {
                // 去除前后的空格
                size_t start = token.find_first_not_of(" \t");
                size_t end = token.find_last_not_of(" \t");
                
                if (start != std::string::npos) {
                    token = token.substr(start, end - start + 1);
                    
                    if (!token.empty()) {
                        try {
                            double value = std::stod(token);
                            vec.push_back(value);
                        } catch (const std::exception& e) {
                            std::cerr << "第 " << lineNum << " 行解析错误: " 
                                      << token << " 不是有效的数字" << std::endl;
                        }
                    }
                }
            }
            
            if (!vec.empty()) {
                result.push_back(vec);
            }
        }
        
        file.close();
        return result;
    }
    
    // 读取指定行
    std::vector<double> readVectorFromLine(int lineNumber) {
        std::vector<double> result;
        std::ifstream file(filename);
        
        if (!file.is_open()) {
            std::cerr << "无法打开文件: " << filename << std::endl;
            return result;
        }
        
        std::string line;
        int currentLine = 0;
        
        while (std::getline(file, line)) {
            currentLine++;
            if (currentLine == lineNumber) {
                std::stringstream ss(line);
                std::string token;
                
                while (std::getline(ss, token, delimiter)) {
                    token.erase(0, token.find_first_not_of(" \t"));
                    token.erase(token.find_last_not_of(" \t") + 1);
                    
                    if (!token.empty()) {
                        try {
                            result.push_back(std::stod(token));
                        } catch (const std::exception& e) {
                            std::cerr << "解析错误: " << token << std::endl;
                        }
                    }
                }
                break;
            }
        }
        
        file.close();
        return result;
    }
    
    // 获取文件中的行数（向量个数）
    int getVectorCount() {
        std::ifstream file(filename);
        if (!file.is_open()) {
            return 0;
        }
        
        int count = 0;
        std::string line;
        while (std::getline(file, line)) {
            if (!line.empty()) {
                count++;
            }
        }
        
        return count;
    }
    
    // 打印所有读取的向量
    void printVectors(const std::vector<std::vector<double>>& vectors) {
        for (size_t i = 0; i < vectors.size(); ++i) {
            std::cout << "向量 " << i + 1 << " [" << vectors[i].size() << "个元素]: ";
            for (size_t j = 0; j < vectors[i].size(); ++j) {
                std::cout << vectors[i][j];
                if (j < vectors[i].size() - 1) {
                    std::cout << ", ";
                }
            }
            std::cout << std::endl;
        }
    }
    
    // 设置分隔符
    void setDelimiter(char delim) {
        delimiter = delim;
    }
    
    // 设置精度
    void setPrecision(int prec) {
        precision = prec;
    }
};



class HandEyeCalibration {
public:
    /**
     * @brief 构造函数
     * @param camera_params_file 相机内参文件路径（YAML格式）
     */
    HandEyeCalibration(const std::string& camera_params_file = "");

    /**
     * @brief 加载相机内参
     */
    bool loadCameraParams(const std::string& camera_params_file);

    /**
     * @brief 采集标定数据
     * @param num_poses 需要采集的位姿数量
     * @param save_path 数据保存路径（可选）
     */
    void collectCalibrationData(int num_poses = 15, const std::string& save_path = "");

    /**
     * @brief 执行手眼标定
     * @param method 标定方法：Tsai-Lenz/Navy/Park
     */
    bool calibrate(cv::HandEyeCalibrationMethod method = cv::CALIB_HAND_EYE_TSAI);

    /**
     * @brief 将相机坐标系下的点转换到法兰坐标系
     * @param point_camera 相机坐标系下的点 (x, y, z)
     * @return 法兰坐标系下的点
     */
    cv::Point3f cameraToFlange(const cv::Point3f& point_camera) const;

    /**
     * @brief 将相机坐标系下的点转换到基坐标系
     * @param point_camera 相机坐标系下的点
     * @param T_base_flange 基坐标系到法兰坐标系的变换矩阵
     * @return 基坐标系下的点
     */
    cv::Point3f cameraToBase(const cv::Point3f& point_camera, const cv::Mat& T_base_flange) const;

    /**
     * @brief 将点从相机坐标系转换到机械臂坐标系
     * @param points_camera 相机坐标系下的点集
     * @param T_base_flange 基坐标系到法兰坐标系的变换矩阵
     * @param target_frame 目标坐标系：0=法兰, 1=基座
     * @return 转换后的点集
     */
    std::vector<cv::Point3f> transformPoints(
        const std::vector<cv::Point3f>& points_camera,
        const cv::Mat& T_base_flange = cv::Mat(),
        int target_frame = 0) const;

    /**
     * @brief 解析机器人位姿字符串为变换矩阵
     * @param pose_str 位姿字符串
     * @param use_quaternion 是否使用四元数格式
     * @return 4x4齐次变换矩阵
     */
    cv::Mat parseRobotPose(const std::string& pose_str, bool use_quaternion = false);
    cv::Mat parseRobotPose(std::vector<double>& values, bool use_quaternion = false);

    /**
     * @brief 欧拉角转旋转矩阵
     */
    cv::Mat eulerToRotationMatrix(double rx, double ry, double rz, bool degrees = true);

    /**
     * @brief 位姿转变换矩阵
     */
    cv::Mat poseToTransformMatrix(double x, double y, double z,
        double rx, double ry, double rz, bool degrees = true);

    /**
     * @brief 获取标定结果
     * @return T_flange_camera 4x4齐次变换矩阵
     */
    cv::Mat getCalibrationResult() const { return T_flange_camera_; }

    /**
     * @brief 获取标定误差
     */
    float getCalibrationError() const { return calibration_error_; }

    /**
     * @brief 保存标定结果到文件
     */
    void saveCalibration(const std::string& filename) const;

    /**
     * @brief 加载标定结果
     */
    bool loadCalibration(const std::string& filename);

    /**
     * @brief 可视化标定板检测结果
     */
    bool detectCalibrationBoard(const cv::Mat& image,
        cv::Mat& image_with_detection,
        cv::Mat& rvec, cv::Mat& tvec) const;



    bool detectCalibrationBoard_ChArUco_file(const std::string& filename,
        cv::Mat& image_with_detection,
        cv::Mat& rvec, cv::Mat& tvec) const;


    // 从文件加载相机标定数据
    bool detectChessboardFromFile(
        const std::string& image_path,
        cv::Mat& image_with_detection,
        cv::Mat& rvec,
        cv::Mat& tvec);

    // 新增函数：直接从棋盘格数据集批量读取
    bool loadChessboardDataset(
        const std::string& image_folder,
        const std::string& robot_pose_file);     

    /**
     * @brief 获取机器人位姿数量
     */
    size_t getPoseCount() const { return robot_poses_.size(); }

    /**
     * @brief 清空所有数据
     */
    void clearData() {
        robot_poses_.clear();
        camera_poses_.clear();
        calibration_error_ = 0.0;
        T_flange_camera_ = cv::Mat();
    }

private:
    // 相机内参
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    void drawCharucoReprojectionError(
        cv::Mat& image_with_detection,
        const std::vector<cv::Point2f>& charuco_corners,
        const std::vector<int>& charuco_ids,
        const cv::Mat& rvec,
        const cv::Mat& tvec) const;

    // 标定板参数
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::CharucoBoard> board_;
    cv::Ptr<cv::aruco::DetectorParameters> params_;

    // 标定数据
    std::vector<cv::Mat> robot_poses_;      // 机器人位姿（T_base_flange）
    std::vector<cv::Mat> camera_poses_;     // 相机位姿（T_camera_calib）

    // 标定结果
    cv::Mat T_flange_camera_;              // 法兰到相机的变换矩阵
    float calibration_error_;

    // 标定板角点世界坐标
    std::vector<cv::Point3f> board_corners_3d_;

    /**
     * @brief 初始化标定板
     */
    void initCalibrationBoard();
};

#endif // HAND_EYE_CALIBRATION_HPP