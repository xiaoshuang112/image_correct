#include "hand_eye_calibration.hpp"
#include <opencv2/calib3d.hpp>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iomanip>
#include <filesystem>

#include "my_base.h"
#include "OrbbecCamera.hpp"
#include "putText.h"
#include "robotapi.h"
#include "basestruct.h"

extern CONFIG g_cfg;
extern std::string g_appPath;
extern std::string g_calibrationPath;
extern std::string g_paramPath;

extern std::shared_ptr<OrbbecCameraV2> g_camera;
extern RobotHandle robotHandle;

extern std::string string_format(const char* format, ...);

namespace fs = std::filesystem;

#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <algorithm>


HandEyeCalibration::HandEyeCalibration(const std::string& camera_params_file, int minIdsNum, float thresh)
    : calibration_error_(0.0) {
    minIdsNum_ = minIdsNum;
    thresh_ = thresh;
    // 初始化标定板
    initCalibrationBoard();

    // 加载相机内参
    if (!camera_params_file.empty()) {
        loadCameraParams(camera_params_file);
    }
}

void HandEyeCalibration::initCalibrationBoard() {
    // 创建 ChArUco 板（更适合 3D 相机）
    int squares_x = 7;      // 横向方格数
    int squares_y = 5;      // 纵向方格数
    float square_length = 0.04f;  // 方格边长 30mm
    float marker_length = 0.028;// 0.023f; // 标记边长 23mm

    // 使用 6x6 的 ArUco 标记字典
#if defined(CV_VERSION_MINOR) && CV_VERSION_MINOR >= 7
    // OpenCV 4.7.0+：使用构造函数

    dictionary_ = cv::makePtr<cv::aruco::Dictionary>(
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250));

    board_ = cv::makePtr<cv::aruco::CharucoBoard>(
        cv::Size(squares_x, squares_y),
        square_length, marker_length, *dictionary_);

    // 检测参数
    params_ = cv::makePtr<cv::aruco::DetectorParameters>();
#else
    // OpenCV < 4.7.0：使用 create() 静态方法
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    // 创建 ChArUco 板
    board_ = cv::aruco::CharucoBoard::create(
        squares_x, squares_y, square_length, marker_length, dictionary_);

    // 检测参数
    params_ = cv::aruco::DetectorParameters::create();
#endif

    // 生成标定板图像
    cv::Mat board_image;
    int margin_pixels = 50;
    int border_bits = 1;

    // 计算图像尺寸（假设1mm对应10像素）
    int pixels_per_mm = 10;
    int image_width = squares_x * (int)(square_length * 1000) * pixels_per_mm + 2 * margin_pixels;
    int image_height = squares_y * (int)(square_length * 1000) * pixels_per_mm + 2 * margin_pixels;

    // 生成标定板图像
    board_->generateImage(cv::Size(image_width, image_height), board_image, margin_pixels, border_bits);

    //// 保存标定板图像
    //cv::imwrite("charuco_board.png", board_image);
    //std::cout << "标定板图像已生成并保存为 charuco_board.png" << std::endl;
    //std::cout << "请打印此图像用于标定，确保打印尺寸正确" << std::endl;

    // 生成标定板角点的 3D 世界坐标
#if CV_MAJOR_VERSION >= 4 && CV_VERSION_MINOR >= 7
    // OpenCV 4.7.0+：使用 getter 方法
    board_corners_3d_ = board_->getChessboardCorners();
#else
    // OpenCV 4.6.x 及以下：直接访问成员变量
    board_corners_3d_ = board_->chessboardCorners;
#endif
    int temp = 0;
    
}

bool HandEyeCalibration::detectChArUco(const cv::Mat& image_gray,
    std::vector<cv::Point2f>& charucoCorners,
    std::vector<int>& charucoIds,
    std::vector<std::vector<cv::Point2f>>& markerCorners,
    std::vector<int>& markerIds) const {

    // 方案1: CharucoDetector (新 API, OpenCV 4.7+)
    cv::aruco::DetectorParameters detectorParams;
    cv::aruco::CharucoParameters charucoParams;
    detectorParams.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    charucoParams.cameraMatrix = camera_matrix_;
    charucoParams.distCoeffs = dist_coeffs_;
    charucoParams.minMarkers = 1;
    charucoParams.tryRefineMarkers = true;
    cv::aruco::CharucoDetector detector(*board_, charucoParams, detectorParams);
    detector.detectBoard(image_gray, charucoCorners, charucoIds, markerCorners, markerIds);

    // 方案2: detectBoard 找到了 marker 但没得到 charuco 角点
    if (charucoIds.empty() && !markerIds.empty()) {
        cv::aruco::interpolateCornersCharuco(
            markerCorners, markerIds, image_gray, board_,
            charucoCorners, charucoIds, camera_matrix_, dist_coeffs_, 1);
    }

    // 方案3: 旧 API detectMarkers + interpolateCornersCharuco
    if (charucoIds.empty()) {
        cv::aruco::detectMarkers(image_gray, dictionary_, markerCorners, markerIds, params_);
        if (!markerIds.empty()) {
            cv::aruco::interpolateCornersCharuco(
                markerCorners, markerIds, image_gray, board_,
                charucoCorners, charucoIds, camera_matrix_, dist_coeffs_, 1);
        }
    }

    return !charucoIds.empty();
}

bool HandEyeCalibration::loadCameraParams(const std::string& camera_params_file) {
    cv::FileStorage fs(camera_params_file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "无法打开相机参数文件: " << camera_params_file << std::endl;
        return false;
    }

    fs["camera_matrix"] >> camera_matrix_;
    fs["distortion_coefficients"] >> dist_coeffs_;

    if (camera_matrix_.empty() || dist_coeffs_.empty()) {
        std::cerr << "相机参数读取失败或格式不正确" << std::endl;
        return false;
    }

    std::cout << "相机内参加载成功" << std::endl;
    std::cout << "相机矩阵: \n" << camera_matrix_ << std::endl;

    return true;
}

void HandEyeCalibration::drawCharucoReprojectionError(
    cv::Mat& image_with_detection,
    std::vector<cv::Point2f>& charuco_corners,
    std::vector<int>& charuco_ids,
    const cv::Mat& rvec,
    const cv::Mat& tvec, float thresh) const {

    std::vector<cv::Point3f> object_points;
    object_points.reserve(charuco_ids.size());

    
#if CV_MAJOR_VERSION >= 4 && CV_VERSION_MINOR >= 7
    // OpenCV 4.7.0+：使用 getter 方法
    const auto& board_object_points = board_->getChessboardCorners();
#else
    // OpenCV 4.6.x 及以下：直接访问成员变量
    const auto& board_object_points = board_->chessboardCorners;
#endif


    for (size_t i = 0; i < charuco_ids.size(); ++i) {
        const int id = charuco_ids[i];
        if (id < 0 || static_cast<size_t>(id) >= board_object_points.size()) {
            continue;
        }
        object_points.push_back(board_object_points[static_cast<size_t>(id)]);
    }

    if (object_points.size() != charuco_corners.size() || object_points.empty()) {
        return;
    }

    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix_, dist_coeffs_, projected_points);

    double sum_err = 0.0;
    double sum_sq_err = 0.0;
    double max_err = 0.0;
    std::vector<float> vError;
    for (size_t i = 0; i < projected_points.size(); ++i) {
        const cv::Point2f& p_det = charuco_corners[i];
        const cv::Point2f& p_proj = projected_points[i];
        const double err = cv::norm(p_det - p_proj);

        sum_err += err;
        sum_sq_err += err * err;
        max_err = std::max(max_err, err);
        vError.push_back(err);
        cv::circle(image_with_detection, p_proj, 3, cv::Scalar(0, 0, 255), -1);
        cv::line(image_with_detection, p_det, p_proj, cv::Scalar(0, 255, 255), 1);
    }

    const double mean_err = sum_err / static_cast<double>(projected_points.size());
    const double rms_err = std::sqrt(sum_sq_err / static_cast<double>(projected_points.size()));

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "Reproj(px) mean=" << mean_err << " rms=" << rms_err << " max=" << max_err;
    cv::putText(image_with_detection, oss.str(), cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
     // filter outer
    if (thresh > 0.01f && !charuco_ids.empty()){
        for (int i = static_cast<int>(charuco_ids.size()) - 1; i >= 0; i--) {
            if (vError[i] >= thresh) {
                charuco_ids.erase(charuco_ids.begin() + i);
                charuco_corners.erase(charuco_corners.begin() + i);                
            }
        }
    }
}

bool HandEyeCalibration::detectCalibrationBoard_aruco(const cv::Mat& image,
    cv::Mat& image_with_detection,
    cv::Mat& rvec, cv::Mat& tvec) const {
    if (camera_matrix_.empty()) return false;

    cv::Mat gray;
    if (image.channels() == 3) cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    else gray = image.clone();

    std::vector<cv::Point2f> charucoCorners;
    std::vector<int> charucoIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    std::vector<int> markerIds;
    if (!detectChArUco(gray, charucoCorners, charucoIds, markerCorners, markerIds))
        return false;

    image.copyTo(image_with_detection);
    cv::aruco::drawDetectedMarkers(image_with_detection, markerCorners, markerIds);

    if (charucoIds.size() >= 6) {
        cv::aruco::drawDetectedCornersCharuco(image_with_detection, charucoCorners, charucoIds);
        if (cv::aruco::estimatePoseCharucoBoard(
                charucoCorners, charucoIds, board_, camera_matrix_, dist_coeffs_, rvec, tvec)) {
            cv::drawFrameAxes(image_with_detection, camera_matrix_, dist_coeffs_, rvec, tvec, 0.05f);
            return true;
        }
    }
    return false;
}


bool HandEyeCalibration::detectCalibrationBoard(const cv::Mat& image,
    cv::Mat& image_with_detection,
    cv::Mat& rvec, cv::Mat& tvec, int* out_corner_count, int override_minIds) const {
    if (camera_matrix_.empty()) return false;

    cv::Mat gray;
    if (image.channels() == 3) cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    else gray = image.clone();

    std::vector<cv::Point2f> charucoCorners;
    std::vector<int> charucoIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    std::vector<int> markerIds;
    detectChArUco(gray, charucoCorners, charucoIds, markerCorners, markerIds);

    image.copyTo(image_with_detection);
    cv::aruco::drawDetectedMarkers(image_with_detection, markerCorners, markerIds);

    int minIds = (override_minIds > 0) ? override_minIds : minIdsNum_;
    if ((int)charucoIds.size() < minIds) {
        if (out_corner_count) *out_corner_count = 0;
        return false;
    }

    cv::aruco::drawDetectedCornersCharuco(image_with_detection, charucoCorners, charucoIds);
    bool valid = false;
    if(charucoIds.size() >= 6)
        valid = cv::aruco::estimatePoseCharucoBoard(
        charucoCorners, charucoIds, board_, camera_matrix_, dist_coeffs_, rvec, tvec);
    if (!valid) {
        if (out_corner_count) *out_corner_count = 0;
        return false;
    }

    drawCharucoReprojectionError(image_with_detection, charucoCorners, charucoIds, rvec, tvec, thresh_);
    if ((int)charucoCorners.size() >= std::max(6, minIds))
        cv::aruco::estimatePoseCharucoBoard(
            charucoCorners, charucoIds, board_, camera_matrix_, dist_coeffs_, rvec, tvec);

    if (out_corner_count) *out_corner_count = (int)charucoCorners.size();
    return true;
}


bool HandEyeCalibration::detectCalibrationBoard_ChArUco_file(const std::string& filename,
    cv::Mat& image_with_detection,
    cv::Mat& rvec, cv::Mat& tvec) const {

    return detectCalibrationBoard_ChArUco_file(filename, image_with_detection, rvec, tvec, nullptr, 0);
}

bool HandEyeCalibration::detectCalibrationBoard_ChArUco_file(const std::string& filename,
    cv::Mat& image_with_detection,
    cv::Mat& rvec, cv::Mat& tvec,
    int* out_corner_count,
    int override_minIds) const {

    cv::Mat image = cv::imread(filename);
    if(0==image.cols || 0 == image.rows)
       return false;
    return detectCalibrationBoard(image, image_with_detection, rvec, tvec, out_corner_count, override_minIds);
}

cv::Mat HandEyeCalibration::parseRobotPose(const std::string& pose_str, bool use_quaternion) {
    std::vector<double> values;
    std::stringstream ss(pose_str);
    std::string token;

    // 去除空格
    std::string trimmed = pose_str;
    trimmed.erase(std::remove(trimmed.begin(), trimmed.end(), ' '), trimmed.end());
    ss = std::stringstream(trimmed);

    while (std::getline(ss, token, ',')) {
        try {
            values.push_back(std::stod(token));
        }
        catch (const std::exception& e) {
            std::cerr << "解析位姿失败: " << e.what() << std::endl;
            return cv::Mat();
        }
    }

    return parseRobotPose(values, use_quaternion);
}


cv::Mat HandEyeCalibration::parseRobotPose(std::vector<double>& values, bool use_quaternion) {

    if (use_quaternion) {
        if (values.size() != 7) {
            std::cerr << "需要7个值 (x,y,z,qx,qy,qz,qw)，但提供了 " << values.size() << " 个" << std::endl;
            return cv::Mat();
        }

        double x = values[0] / 1000.0;  // mm to m
        double y = values[1] / 1000.0;
        double z = values[2] / 1000.0;
        double qx = values[3];
        double qy = values[4];
        double qz = values[5];
        double qw = values[6];

        // 归一化四元数
        double norm = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
        if (norm > 0) {
            qw /= norm;
            qx /= norm;
            qy /= norm;
            qz /= norm;
        }

        // 四元数转旋转矩阵
        cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
        R.at<double>(0, 0) = 1 - 2 * qy * qy - 2 * qz * qz;
        R.at<double>(0, 1) = 2 * qx * qy - 2 * qz * qw;
        R.at<double>(0, 2) = 2 * qx * qz + 2 * qy * qw;

        R.at<double>(1, 0) = 2 * qx * qy + 2 * qz * qw;
        R.at<double>(1, 1) = 1 - 2 * qx * qx - 2 * qz * qz;
        R.at<double>(1, 2) = 2 * qy * qz - 2 * qx * qw;

        R.at<double>(2, 0) = 2 * qx * qz - 2 * qy * qw;
        R.at<double>(2, 1) = 2 * qy * qz + 2 * qx * qw;
        R.at<double>(2, 2) = 1 - 2 * qx * qx - 2 * qy * qy;

        cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
        R.copyTo(T(cv::Rect(0, 0, 3, 3)));
        T.at<double>(0, 3) = x;
        T.at<double>(1, 3) = y;
        T.at<double>(2, 3) = z;

        return T;
    }
    else {
        if (values.size() != 6) {
            std::cerr << "需要6个值 (x,y,z,rx,ry,rz)，但提供了 " << values.size() << " 个" << std::endl;
            return cv::Mat();
        }

        // SDK 位姿为轴角格式：x,y,z(mm), Rx,Ry,Rz(°)
        // 使用 Rodrigues 将轴角转换为旋转矩阵
        double x = values[0] / 1000.0;  // mm to m
        double y = values[1] / 1000.0;
        double z = values[2] / 1000.0;
        double rx_rad = values[3] * CV_PI / 180.0;  // degrees to radians
        double ry_rad = values[4] * CV_PI / 180.0;
        double rz_rad = values[5] * CV_PI / 180.0;

        cv::Mat rvec = (cv::Mat_<double>(3, 1) << rx_rad, ry_rad, rz_rad);
        cv::Mat R;
        cv::Rodrigues(rvec, R);

        cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
        R.copyTo(T(cv::Rect(0, 0, 3, 3)));
        T.at<double>(0, 3) = x;
        T.at<double>(1, 3) = y;
        T.at<double>(2, 3) = z;

        return T;
    }
}



cv::Mat HandEyeCalibration::eulerToRotationMatrix(double rx, double ry, double rz, bool degrees) {
    if (degrees) {
        rx = rx * CV_PI / 180.0;
        ry = ry * CV_PI / 180.0;
        rz = rz * CV_PI / 180.0;
    }

    // 假设旋转顺序为 ZYX (常见于机器人)
    cv::Mat Rz = (cv::Mat_<double>(3, 3) <<
        cos(rz), -sin(rz), 0,
        sin(rz), cos(rz), 0,
        0, 0, 1);

    cv::Mat Ry = (cv::Mat_<double>(3, 3) <<
        cos(ry), 0, sin(ry),
        0, 1, 0,
        -sin(ry), 0, cos(ry));

    cv::Mat Rx = (cv::Mat_<double>(3, 3) <<
        1, 0, 0,
        0, cos(rx), -sin(rx),
        0, sin(rx), cos(rx));

    return Rz * Ry * Rx;
}

cv::Mat HandEyeCalibration::poseEulerToTransformMatrix(double x, double y, double z,
    double rx, double ry, double rz,
    bool degrees) {
    cv::Mat R = eulerToRotationMatrix(rx, ry, rz, degrees);
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(T(cv::Rect(0, 0, 3, 3)));
    T.at<double>(0, 3) = x;
    T.at<double>(1, 3) = y;
    T.at<double>(2, 3) = z;

    return T;
}


// 新增函数：直接从棋盘格数据集批量读取
bool HandEyeCalibration::loadChessboardDataset(
    const std::string& image_folder,
    const std::string& robot_pose_file,
    bool clear_existing,
    int override_minIds) {

    if (clear_existing) {
        robot_poses_.clear();
        camera_poses_.clear();
        image_paths_.clear();
        corner_counts_.clear();
    }

    size_t base_count = robot_poses_.size();  // 已有数据量，新数据从这之后开始

    // 1. 读取机器人位姿（从Excel导出的CSV）
    std::ifstream file(robot_pose_file);
    std::string line;

    while (std::getline(file, line)) {
        cv::Mat T_base_flange = parseRobotPose(line, false);
        if (!T_base_flange.empty()) {
            robot_poses_.push_back(T_base_flange);
        }
    }

    // 2. 遍历图像文件夹（只遍历本次新增的位姿）
    size_t cam_count_before = camera_poses_.size();
    for (size_t i = base_count, idx = 0; i < robot_poses_.size(); i++, idx++) {
        std::stringstream ss;
        ss << image_folder << "/" << std::setw(3) << std::setfill('0') << idx << ".jpg";
        std::string img_path = ss.str();

        cv::Mat rvec, tvec;
        cv::Mat image_with_detection;
        int corner_count = 0;
        if (detectCalibrationBoard_ChArUco_file(img_path, image_with_detection, rvec, tvec,
                                                 &corner_count, override_minIds)) {
            cv::Mat R;
            cv::Rodrigues(rvec, R);
            cv::Mat T_camera_calib = cv::Mat::eye(4, 4, CV_64F);
            R.copyTo(T_camera_calib(cv::Rect(0, 0, 3, 3)));
            tvec.copyTo(T_camera_calib(cv::Rect(3, 0, 1, 3)));

            camera_poses_.push_back(T_camera_calib);
            image_paths_.push_back(img_path);
            corner_counts_.push_back(corner_count);
        }
        else {
            std::cerr << "第" << idx << "张图像检测失败，跳过" << std::endl;
            // 从robot_poses_中移除对应的位姿
            robot_poses_.erase(robot_poses_.begin() + i);
            i--;
        }
    }

    std::cout << "成功加载 " << camera_poses_.size() << " 组有效数据（本次新增 "
              << (camera_poses_.size() - cam_count_before) << "）" << std::endl;
    return camera_poses_.size() >= 3;
}

void HandEyeCalibration::collectCalibrationData(int num_poses, 
    const std::string& save_name) {
    robot_poses_.clear();
    camera_poses_.clear();

    if (fs::exists(g_calibrationPath))
    {
        fs::remove_all(g_calibrationPath);
    }

    if (!fs::exists(g_calibrationPath))
        fs::create_directory(g_calibrationPath);

    std::string rob_pos_file = g_calibrationPath + "rob_pos.csv";
    std::string rob_pos_euler_file = g_calibrationPath + "rob_pos_euler.csv";
    std::string rob_joint_file = g_calibrationPath + "rob_joint.csv";
    
    std::cout << "\n=== 开始采集标定数据 ===" << std::endl;
    std::cout << "需要采集 " << num_poses << " 个位姿" << std::endl;
    std::cout << "操作说明:" << std::endl;
    std::cout << "  's' - 保存当前位姿" << std::endl;
    std::cout << "  'q' - 退出采集" << std::endl;
    std::cout << "  'r' - 重新采集上一个位姿" << std::endl;

    int collected_poses = 0;
    while (collected_poses < num_poses) {
        cv::Mat frame = g_camera->captureOneFrame(1000);
        if (frame.empty()) break;

        cv::Mat image_with_detection;
        cv::Mat rvec, tvec;

        std::string img_name = string_format("%s%03d.jpg", g_calibrationPath.c_str(), collected_poses);

        cv::imwrite(img_name, frame);

        // 检测角点
        bool detected = detectChessboardFromFile(img_name, image_with_detection, rvec, tvec);

        if (image_with_detection.empty())
        {
            std::string hint = "未能检测到角点，请重新 拖动摄头到合适位置";
            putTextZH(frame, hint.c_str(), cv::Point(10, 30),
                        cv::Scalar(0, 0, 255), 20);

            cv::imshow("数据采集", frame);
            char key = cv::waitKey(0);    
            continue;        
        }
        

        // 显示提示信息
        std::string status = "已采集: " + std::to_string(collected_poses) +
            "/" + std::to_string(num_poses);
        putTextZH(image_with_detection, status.c_str(), cv::Point(10, 30),
            cv::Scalar(0, 255, 0), 20);

        std::string hint = "按 's' 保存位姿，'q' 退出";
        putTextZH(image_with_detection, hint.c_str(), cv::Point(10, 60),
            cv::Scalar(255, 255, 0), 20);

        if (detected) {
            putTextZH(image_with_detection, "检测到标定板 - 可以保存", cv::Point(10, 90),
                cv::Scalar(0, 255, 0), 20);
        }
        else {
            putTextZH(image_with_detection, "未检测到标定板", cv::Point(10, 90),
                cv::Scalar(0, 0, 255), 20);
        }

        if (detected) {
            // 从用户输入获取机器人位姿
            std::cout << "\n=== 保存位姿 " << (collected_poses + 1) << " ===" << std::endl;
            std::cout << "开始获取机器人法兰位姿:" << std::endl;
            std::cout << "格式: x,y,z,rx,ry,rz (单位:mm,度)" << std::endl;
            std::cout << "例如: 500,0,300,0,0,0\n" << std::endl;

            std::vector<double> tcp_pose(ROB_AXIS_NUM);
            CRresult result = cr_get_tcpActualPose(robotHandle, tcp_pose.data());
            assert(result == sucess);

            // TCP 转法兰位姿
            TCPMsg tcpMsg;
            result = cr_get_currentTCPmsg(robotHandle, &tcpMsg);
            assert(result == sucess);

            std::vector<double> flange_pose(ROB_AXIS_NUM);
            result = cr_TcpToFlangePose(tcp_pose.data(), tcpMsg.tcpOffset, flange_pose.data());
            assert(result == sucess);

            for (int i = 0; i < ROB_AXIS_NUM; i++)
            {
                printf("%.2f   ", flange_pose[i]);
            }

            std::cout << " \n  flangePose \n" << std::endl;

            std::vector<double> actual_joint_pos(ROB_AXIS_NUM), eule_pos(ROB_AXIS_NUM);
            result = cr_get_jointActualPos(robotHandle, actual_joint_pos.data());
            for (int i = 0; i < ROB_AXIS_NUM; i++)
            {
                printf("%.2f   ", actual_joint_pos[i]);
            }

            std::cout << " \n" << std::endl;

            // 将法兰位姿保存到文件
            VectorWriter writer_tcp(rob_pos_file, 6);
            writer_tcp.appendDoubleVector(flange_pose);

            VectorWriter writer_joint(rob_joint_file, 6);
            writer_joint.appendDoubleVector(actual_joint_pos);
            // for euler angle
            VectorWriter writer_euler(rob_pos_euler_file, 6);
            cr_AxisAngle2Eule(tcp_pose.data(), eule_pos.data());
            writer_euler.appendDoubleVector(eule_pos);
            

            //std::string robot_pose_str;
            //std::cin.ignore();  // 清除缓冲区
            //std::getline(std::cin, robot_pose_str);

            //cv::Mat T_base_flange = parseRobotPose(robot_pose_str, false);

            cv::Mat T_base_flange = parseRobotPose(flange_pose, false);

            if (!T_base_flange.empty()) {
                // 将旋转向量和平移向量转换为变换矩阵
                cv::Mat R_camera_calib;
                cv::Rodrigues(rvec, R_camera_calib);

                cv::Mat T_camera_calib = cv::Mat::eye(4, 4, CV_64F);
                R_camera_calib.copyTo(T_camera_calib(cv::Rect(0, 0, 3, 3)));
                tvec.copyTo(T_camera_calib(cv::Rect(3, 0, 1, 3)));

                robot_poses_.push_back(T_base_flange);
                camera_poses_.push_back(T_camera_calib);

                collected_poses++;
                std::cout << "✓ 已保存位姿 " << collected_poses << std::endl;
            }
            else {
                std::cout << "✗ 位姿解析失败，请重新输入" << std::endl;
            }
        }

        cv::imshow("数据采集", image_with_detection);

        char key = cv::waitKey(0);

        if (key == 'q' || key == 27) {  // 27 是 ESC 键
            std::cout << "用户中断采集" << std::endl;
            break;
        }        

        if (key == 'r' && collected_poses > 0) {
            // 重新采集上一个位姿
            robot_poses_.pop_back();
            camera_poses_.pop_back();
            collected_poses--;
            std::cout << "重新采集上一个位姿，当前已采集: " << collected_poses << std::endl;
        }
    }


    std::cout << "\n数据采集完成，共采集 " << collected_poses << " 个位姿" << std::endl;

    // 保存数据
    if (!save_name.empty() && !robot_poses_.empty()) {
        std::string save_path = g_paramPath + save_name;
        cv::FileStorage fs(save_path, cv::FileStorage::WRITE);
        fs << "num_poses" << (int)robot_poses_.size();
       
        for (size_t i = 0; i < robot_poses_.size(); ++i) {
            fs << "robot_pose_" + std::to_string(i) << robot_poses_[i];
            fs << "camera_pose_" + std::to_string(i) << camera_poses_[i];
        }
        fs.release();
        std::cout << "数据已保存到: " << save_path << std::endl;
    }
}

bool HandEyeCalibration::calibrate(cv::HandEyeCalibrationMethod method, int min_corners) {
    if (robot_poses_.size() < 3 || camera_poses_.size() < 3) {
        std::cerr << "需要至少3个位姿进行标定" << std::endl;
        return false;
    }

    if (robot_poses_.size() != camera_poses_.size()) {
        std::cerr << "机器人位姿和相机位姿数量不匹配" << std::endl;
        return false;
    }

    std::vector<cv::Mat> R_base_flange, t_base_flange;
    std::vector<cv::Mat> R_camera_calib, t_camera_calib;

    int used = 0;
    for (size_t i = 0; i < robot_poses_.size(); ++i) {
        if (min_corners > 0 && !corner_counts_.empty() && corner_counts_[i] < min_corners) {
            continue;
        }
        used++;
        cv::Mat R, t;

        // 机器人位姿
        robot_poses_[i](cv::Rect(0, 0, 3, 3)).copyTo(R);
        robot_poses_[i](cv::Rect(3, 0, 1, 3)).copyTo(t);
        R_base_flange.push_back(R.clone());
        t_base_flange.push_back(t.clone());

        // 相机位姿
        camera_poses_[i](cv::Rect(0, 0, 3, 3)).copyTo(R);
        camera_poses_[i](cv::Rect(3, 0, 1, 3)).copyTo(t);
        R_camera_calib.push_back(R.clone());
        t_camera_calib.push_back(t.clone());
    }

    // 执行手眼标定
    cv::Mat R_flange_camera, t_flange_camera;
    cv::calibrateHandEye(R_base_flange, t_base_flange,
        R_camera_calib, t_camera_calib,
        R_flange_camera, t_flange_camera,
        method);

    // 组合成齐次变换矩阵
    T_flange_camera_ = cv::Mat::eye(4, 4, CV_64F);
    R_flange_camera.copyTo(T_flange_camera_(cv::Rect(0, 0, 3, 3)));
    t_flange_camera.copyTo(T_flange_camera_(cv::Rect(3, 0, 1, 3)));

    // 计算标定误差
    calibration_error_ = 0.0;
    int count = 0;
    for (size_t i = 0; i < robot_poses_.size(); ++i) {
        for (size_t j = i + 1; j < robot_poses_.size(); ++j) {
            // 计算 A 和 B
            cv::Mat A = robot_poses_[j].inv() * robot_poses_[i];
            cv::Mat B = camera_poses_[j].inv() * camera_poses_[i];

            // 计算 AX 和 XB
            cv::Mat AX = A * T_flange_camera_;
            cv::Mat XB = T_flange_camera_ * B;

            // 计算误差
            cv::Mat diff = AX - XB;
            calibration_error_ += cv::norm(diff);
            count++;
        }
    }

    if (count > 0) {
        calibration_error_ /= count;
    }

    std::cout << "\n标定完成！" << std::endl;
    std::cout << "标定误差: " << calibration_error_ << std::endl;
    std::cout << "参与位姿: " << used << " / " << (int)robot_poses_.size() << std::endl;
    std::cout << "T_flange_camera (相机相对于法兰的变换矩阵):" << std::endl;
    std::cout << T_flange_camera_ << std::endl;

    // 分解为平移和欧拉角显示
    cv::Mat R = T_flange_camera_(cv::Rect(0, 0, 3, 3));
    cv::Mat t = T_flange_camera_(cv::Rect(3, 0, 1, 3));

    std::cout << "平移 (米): [" << t.at<double>(0) << ", "
        << t.at<double>(1) << ", " << t.at<double>(2) << "]" << std::endl;

    return true;
}

void HandEyeCalibration::evaluateCalibration(const std::string& output_file, int min_corners) {
    if (T_flange_camera_.empty() || robot_poses_.empty()) {
        std::cerr << "请先进行标定" << std::endl;
        return;
    }

    size_t n = robot_poses_.size();
    std::vector<cv::Point3d> board_positions(n);
    std::vector<double> board_angles(n);
    std::vector<bool> valid(n, false);
    cv::Point3d pos_sum(0, 0, 0);
    double ang_sum = 0;
    int valid_cnt = 0;

    auto is_valid = [&](size_t i) -> bool {
        if (min_corners > 0 && !corner_counts_.empty() && corner_counts_[i] < min_corners)
            return false;
        return true;
    };

    // 1. 计算每帧标定板在基坐标系下的位姿（仅有效样本）
    for (size_t i = 0; i < n; ++i) {
        if (!is_valid(i)) continue;
        cv::Mat T_base_calib = robot_poses_[i] * T_flange_camera_ * camera_poses_[i];

        board_positions[i].x = T_base_calib.at<double>(0, 3) * 1000.0;  // m -> mm
        board_positions[i].y = T_base_calib.at<double>(1, 3) * 1000.0;
        board_positions[i].z = T_base_calib.at<double>(2, 3) * 1000.0;

        pos_sum.x += board_positions[i].x;
        pos_sum.y += board_positions[i].y;
        pos_sum.z += board_positions[i].z;

        cv::Mat R = T_base_calib(cv::Rect(0, 0, 3, 3));
        cv::Mat rvec;
        cv::Rodrigues(R, rvec);
        board_angles[i] = cv::norm(rvec) * 180.0 / CV_PI;
        ang_sum += board_angles[i];
        valid[i] = true;
        valid_cnt++;
    }

    if (valid_cnt < 2) {
        std::cerr << "有效样本不足，无法评估" << std::endl;
        return;
    }

    cv::Point3d pos_mean(pos_sum.x / valid_cnt, pos_sum.y / valid_cnt, pos_sum.z / valid_cnt);
    double ang_mean = ang_sum / valid_cnt;

    // 2. 计算标准差（仅有效样本）
    double pos_var = 0, ang_var = 0;
    double pos_var_x = 0, pos_var_y = 0, pos_var_z = 0;
    for (size_t i = 0; i < n; ++i) {
        if (!valid[i]) continue;
        double dx = board_positions[i].x - pos_mean.x;
        double dy = board_positions[i].y - pos_mean.y;
        double dz = board_positions[i].z - pos_mean.z;
        pos_var += dx * dx + dy * dy + dz * dz;
        pos_var_x += dx * dx;  pos_var_y += dy * dy;  pos_var_z += dz * dz;
        double da = board_angles[i] - ang_mean;
        ang_var += da * da;
    }
    double pos_std = sqrt(pos_var / valid_cnt);
    double pos_std_x = sqrt(pos_var_x / valid_cnt);
    double pos_std_y = sqrt(pos_var_y / valid_cnt);
    double pos_std_z = sqrt(pos_var_z / valid_cnt);
    double ang_std = sqrt(ang_var / valid_cnt);

    std::cout << "\n=== 标定质量评估 (有效 " << valid_cnt << "/" << n << ") ===" << std::endl;
    std::cout << "标定板在基座标系下的位置:" << std::endl;
    std::cout << "  均值 (mm):   " << std::fixed << std::setprecision(3)
              << pos_mean.x << ", " << pos_mean.y << ", " << pos_mean.z << std::endl;
    std::cout << "  标准差 (mm): " << pos_std_x << ", " << pos_std_y << ", " << pos_std_z << std::endl;
    std::cout << "  位置总标准差: " << pos_std << " mm" << std::endl;
    std::cout << "  姿态角度标准差: " << ang_std << " deg" << std::endl;

    // 3. 重投影误差（在图像上，仅有效样本）
    double reproj_sum = 0, reproj_sum_sq = 0, reproj_max = 0;
    int reproj_count = 0;

    if (!image_paths_.empty() && !board_corners_3d_.empty()) {
        for (size_t i = 0; i < n && i < image_paths_.size(); ++i) {
            if (!valid[i]) continue;

            cv::Mat R_cam = camera_poses_[i](cv::Rect(0, 0, 3, 3));
            cv::Mat t_cam = camera_poses_[i](cv::Rect(3, 0, 1, 3)).clone();
            cv::Mat rvec_cam;
            cv::Rodrigues(R_cam, rvec_cam);

            cv::Mat img = cv::imread(image_paths_[i]);
            if (img.empty()) continue;

            cv::Mat gray;
            cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

            std::vector<cv::Point2f> charucoCorners;
            std::vector<int> charucoIds;
            std::vector<std::vector<cv::Point2f>> markerCorners;
            std::vector<int> markerIds;
            if (!detectChArUco(gray, charucoCorners, charucoIds, markerCorners, markerIds))
                continue;

            double per_img_err = 0;
            int per_img_cnt = 0;
            for (size_t j = 0; j < charucoIds.size(); ++j) {
                int id = charucoIds[j];
                if (id < 0 || static_cast<size_t>(id) >= board_corners_3d_.size()) continue;
                std::vector<cv::Point3f> obj_pts = {board_corners_3d_[id]};
                std::vector<cv::Point2f> img_pts;
                cv::projectPoints(obj_pts, rvec_cam, t_cam, camera_matrix_, dist_coeffs_, img_pts);
                if (img_pts.empty()) continue;
                double err = cv::norm(charucoCorners[j] - img_pts[0]);
                reproj_sum += err; reproj_sum_sq += err * err;
                reproj_max = std::max(reproj_max, err);
                reproj_count++; per_img_err += err; per_img_cnt++;
            }
        }
    }

    if (reproj_count > 0) {
        double mean_err = reproj_sum / reproj_count;
        double rms_err = sqrt(reproj_sum_sq / reproj_count);
        std::cout << "\n重投影误差:" << std::endl;
        std::cout << "  均值: " << std::fixed << std::setprecision(4) << mean_err << " px" << std::endl;
        std::cout << "  RMS:  " << rms_err << " px" << std::endl;
        std::cout << "  最大: " << reproj_max << " px" << std::endl;
        std::cout << "  参与角点总数: " << reproj_count << std::endl;
    }

    std::cout << "=== 评估结束 ===" << std::endl;

    // 4. 可选：保存到文件
    if (!output_file.empty()) {
        cv::FileStorage fs(output_file, cv::FileStorage::WRITE);
        fs << "board_position_in_base_mm" << pos_mean;
        fs << "board_position_stddev_mm" << cv::Vec3d(pos_std_x, pos_std_y, pos_std_z);
        fs << "board_position_total_stddev_mm" << pos_std;
        fs << "board_orientation_stddev_deg" << ang_std;
        fs << "handeye_calibration_error" << calibration_error_;
        fs << "valid_poses" << valid_cnt;
        fs << "total_poses" << (int)n;
        if (reproj_count > 0) {
            fs << "reprojection_error_mean_px" << (reproj_sum / reproj_count);
            fs << "reprojection_error_rms_px" << sqrt(reproj_sum_sq / reproj_count);
            fs << "reprojection_error_max_px" << reproj_max;
        }
        fs.release();
        std::cout << "评估报告已保存到: " << output_file << std::endl;
    }
}

void HandEyeCalibration::dumpCalibrationDebug(const std::string& csv_path, int min_corners) {
    std::ofstream csv(csv_path);
    if (!csv.is_open()) {
        std::cerr << "无法创建调试文件: " << csv_path << std::endl;
        return;
    }

    // 表头
    csv << "idx,"
        << "robot_x_mm,robot_y_mm,robot_z_mm,robot_rx_deg,robot_ry_deg,robot_rz_deg,"
        << "cam_x_m,cam_y_m,cam_z_m,cam_rx_deg,cam_ry_deg,cam_rz_deg,"
        << "board_x_mm,board_y_mm,board_z_mm,board_rx_deg,board_ry_deg,board_rz_deg,"
        << "corner_count,valid\n";

    auto is_valid = [&](size_t i) -> bool {
        if (min_corners > 0 && !corner_counts_.empty() && corner_counts_[i] < min_corners)
            return false;
        return true;
    };

    for (size_t i = 0; i < robot_poses_.size(); ++i) {
        bool v = is_valid(i);

        // 机器人位姿（mm, deg）
        double rpx = robot_poses_[i].at<double>(0, 3) * 1000.0;
        double rpy = robot_poses_[i].at<double>(1, 3) * 1000.0;
        double rpz = robot_poses_[i].at<double>(2, 3) * 1000.0;

        cv::Mat RR = robot_poses_[i](cv::Rect(0, 0, 3, 3));
        cv::Mat rvecR;
        cv::Rodrigues(RR, rvecR);
        double rangle = cv::norm(rvecR) * 180.0 / CV_PI;

        // 相机位姿（m, deg）
        double cpx = camera_poses_[i].at<double>(0, 3);
        double cpy = camera_poses_[i].at<double>(1, 3);
        double cpz = camera_poses_[i].at<double>(2, 3);

        cv::Mat CR = camera_poses_[i](cv::Rect(0, 0, 3, 3));
        cv::Mat crvec;
        cv::Rodrigues(CR, crvec);
        double cangle = cv::norm(crvec) * 180.0 / CV_PI;

        // 标定板在基座标系下的位姿
        double bx = 0, by = 0, bz = 0, bangle = 0;
        if (!T_flange_camera_.empty()) {
            cv::Mat T_base_calib = robot_poses_[i] * T_flange_camera_ * camera_poses_[i];
            bx = T_base_calib.at<double>(0, 3) * 1000.0;
            by = T_base_calib.at<double>(1, 3) * 1000.0;
            bz = T_base_calib.at<double>(2, 3) * 1000.0;
            cv::Mat BR = T_base_calib(cv::Rect(0, 0, 3, 3));
            cv::Mat brvec;
            cv::Rodrigues(BR, brvec);
            bangle = cv::norm(brvec) * 180.0 / CV_PI;
        }

        int cc = (!corner_counts_.empty() && i < corner_counts_.size()) ? corner_counts_[i] : 0;

        csv << i << ","
            << rpx << "," << rpy << "," << rpz << ","
            << rvecR.at<double>(0) * 180.0 / CV_PI << ","
            << rvecR.at<double>(1) * 180.0 / CV_PI << ","
            << rvecR.at<double>(2) * 180.0 / CV_PI << ","
            << cpx << "," << cpy << "," << cpz << ","
            << crvec.at<double>(0) * 180.0 / CV_PI << ","
            << crvec.at<double>(1) * 180.0 / CV_PI << ","
            << crvec.at<double>(2) * 180.0 / CV_PI << ","
            << bx << "," << by << "," << bz << ","
            << bangle << ",,,"
            << cc << ","
            << (v ? "1" : "0") << "\n";
    }

    csv.close();
    std::cout << "调试数据已保存到: " << csv_path << std::endl;
}


cv::Point3f HandEyeCalibration::cameraToFlange(const cv::Point3f& point_camera) const {
    if (T_flange_camera_.empty()) {
        std::cerr << "请先进行标定" << std::endl;
        return cv::Point3f(0, 0, 0);
    }

    cv::Mat point_camera_mat = (cv::Mat_<double>(4, 1) <<
        point_camera.x, point_camera.y, point_camera.z, 1.0);

    cv::Mat point_flange_mat = T_flange_camera_ * point_camera_mat;

    return cv::Point3f(
        point_flange_mat.at<double>(0),
        point_flange_mat.at<double>(1),
        point_flange_mat.at<double>(2)
    );
}

cv::Point3f HandEyeCalibration::cameraToBase(const cv::Point3f& point_camera,
    const cv::Mat& T_base_flange) const {
    if (T_flange_camera_.empty()) {
        std::cerr << "请先进行标定" << std::endl;
        return cv::Point3f(0, 0, 0);
    }

    cv::Mat point_camera_mat = (cv::Mat_<double>(4, 1) <<
        point_camera.x, point_camera.y, point_camera.z, 1.0);

    // P_base = T_base_flange * T_flange_camera * P_camera
    cv::Mat point_base_mat = T_base_flange * T_flange_camera_ * point_camera_mat;

    return cv::Point3f(
        point_base_mat.at<double>(0),
        point_base_mat.at<double>(1),
        point_base_mat.at<double>(2)
    );
}

std::vector<cv::Point3f> HandEyeCalibration::transformPoints(
    const std::vector<cv::Point3f>& points_camera,
    const cv::Mat& T_base_flange,
    int target_frame) const {

    std::vector<cv::Point3f> transformed_points;

    for (const auto& point : points_camera) {
        cv::Point3f transformed_point;

        if (target_frame == 0) {
            // 转换到法兰坐标系
            transformed_point = cameraToFlange(point);
        }
        else if (target_frame == 1) {
            // 转换到基坐标系
            if (T_base_flange.empty()) {
                std::cerr << "需要提供 T_base_flange 矩阵" << std::endl;
                break;
            }
            transformed_point = cameraToBase(point, T_base_flange);
        }
        else {
            std::cerr << "无效的目标坐标系" << std::endl;
            break;
        }

        transformed_points.push_back(transformed_point);
    }

    return transformed_points;
}

void HandEyeCalibration::saveCalibration(const std::string& filename) const {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    fs << "T_flange_camera" << T_flange_camera_;
    fs << "calibration_error" << calibration_error_;
    fs << "camera_matrix" << camera_matrix_;
    fs << "dist_coeffs" << dist_coeffs_;
    fs.release();

    std::cout << "标定结果已保存到: " << filename << std::endl;
}

bool HandEyeCalibration::loadCalibration(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "无法打开标定文件: " << filename << std::endl;
        return false;
    }

    fs["T_flange_camera"] >> T_flange_camera_;
    fs["calibration_error"] >> calibration_error_;
    fs["camera_matrix"] >> camera_matrix_;
    fs["dist_coeffs"] >> dist_coeffs_;

    if (T_flange_camera_.empty()) {
        std::cerr << "标定文件格式不正确" << std::endl;
        return false;
    }

    std::cout << "标定结果加载成功" << std::endl;
    std::cout << "标定误差: " << calibration_error_ << std::endl;

    return true;
}



bool HandEyeCalibration::detectChessboardFromFile(
    const std::string& image_path,
    cv::Mat& image_with_detection,
    cv::Mat& rvec,
    cv::Mat& tvec) {

    if (camera_matrix_.empty()) {
        std::cerr << "请先加载相机内参" << std::endl;
        return false;
    }

    cv::Mat img = cv::imread(image_path);
    if (img.empty()) {
        std::cerr << "无法读取图像: " << image_path << std::endl;
        return false;
    }

    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    // 棋盘格参数：8x8内角点，方格大小30mm（根据数据集调整）
    cv::Size board_size(g_cfg.grid_num_heng-1, g_cfg.grid_num_shu - 1);
    float square_size = g_cfg.grid_size;  // 30mm

    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(gray, board_size, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

    if (!found) {
        std::cout << "未检测到棋盘格: " << image_path << std::endl;
        return false;
    }

    // 亚像素精确化
    cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));

    // 生成3D世界坐标点
    std::vector<cv::Point3f> object_points;
    for (int i = 0; i < board_size.height; i++) {
        for (int j = 0; j < board_size.width; j++) {
            object_points.push_back(cv::Point3f(
                j * square_size, i * square_size, 0));
        }
    }

    // 使用solvePnP估计位姿
    cv::solvePnP(object_points, corners, camera_matrix_, dist_coeffs_,
        rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);

    // 绘制检测到的标记
    image_with_detection = img;

    // 7. 绘制角点到原图
    cv::drawChessboardCorners(image_with_detection, board_size, corners, found);

    return true;
}
