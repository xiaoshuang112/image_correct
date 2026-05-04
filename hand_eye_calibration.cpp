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


HandEyeCalibration::HandEyeCalibration(const std::string& camera_params_file)
    : calibration_error_(0.0) {

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
    const std::vector<cv::Point2f>& charuco_corners,
    const std::vector<int>& charuco_ids,
    const cv::Mat& rvec,
    const cv::Mat& tvec) const {

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

    for (size_t i = 0; i < projected_points.size(); ++i) {
        const cv::Point2f& p_det = charuco_corners[i];
        const cv::Point2f& p_proj = projected_points[i];
        const double err = cv::norm(p_det - p_proj);

        sum_err += err;
        sum_sq_err += err * err;
        max_err = std::max(max_err, err);

        cv::circle(image_with_detection, p_proj, 3, cv::Scalar(0, 0, 255), -1);
        cv::line(image_with_detection, p_det, p_proj, cv::Scalar(0, 255, 255), 1);
    }

    const double mean_err = sum_err / static_cast<double>(projected_points.size());
    const double rms_err = std::sqrt(sum_sq_err / static_cast<double>(projected_points.size()));

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "Reproj(px) mean=" << mean_err << " rms=" << rms_err << " max=" << max_err;
    cv::putText(image_with_detection, oss.str(), cv::Point(10, 120),
        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
}

bool HandEyeCalibration::detectCalibrationBoard_aruco(const cv::Mat& image,
    cv::Mat& image_with_detection,
    cv::Mat& rvec, cv::Mat& tvec) const {
    if (camera_matrix_.empty()) {
        std::cerr << "请先加载相机内参" << std::endl;
        return false;
    }

    cv::Mat image_gray, image_marker;
    if (image.channels() == 3) {
        cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
    }
    else {
        image_gray = image.clone();
    }

    // 检测 ArUco 标记
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::aruco::detectMarkers(image_gray, dictionary_, marker_corners, marker_ids, params_);

    // 绘制检测结果
    image.copyTo(image_with_detection);
    image.copyTo(image_marker);

    if (marker_ids.empty()) {
        return false;
    }

    // 绘制检测到的标记
    cv::aruco::drawDetectedMarkers(image_marker, marker_corners, marker_ids);

    // 插值 ChArUco 角点
    std::vector<cv::Point2f> charuco_corners;
    std::vector<int> charuco_ids;

    if (marker_ids.size() > 0) {
        cv::aruco::interpolateCornersCharuco(
            marker_corners, marker_ids, image_gray, board_,
            charuco_corners, charuco_ids, camera_matrix_, dist_coeffs_);
    }

    // 如果找到足够多的角点，估计位姿
    if (charuco_ids.size() >= 4) {
        cv::aruco::drawDetectedCornersCharuco(
            image_with_detection, charuco_corners, charuco_ids);

        // 估计标定板位姿
        bool valid = cv::aruco::estimatePoseCharucoBoard(
            charuco_corners, charuco_ids, board_,
            camera_matrix_, dist_coeffs_, rvec, tvec);

        if (valid) {
            // 计算反投影误差并可视化
            drawCharucoReprojectionError(image_with_detection, charuco_corners, charuco_ids, rvec, tvec);

            // 绘制坐标系
            cv::drawFrameAxes(image_with_detection, camera_matrix_,
                dist_coeffs_, rvec, tvec, 0.05f);
            return true;
        }
    }

    return false;
}


// charuco calibration
bool HandEyeCalibration::detectCalibrationBoard(const cv::Mat& image,
    cv::Mat& image_with_detection,
    cv::Mat& rvec, cv::Mat& tvec) const {
    if (camera_matrix_.empty()) {
        std::cerr << "请先加载相机内参" << std::endl;
        return false;
    }

    cv::Mat image_gray, image_marker;
    if (image.channels() == 3) {
        cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
    }
    else {
        image_gray = image.clone();
    }

    // 创建 Charuco 检测器，并启用标记角点细化以提高精度
    cv::aruco::DetectorParameters detectorParams;
    cv::aruco::CharucoParameters charucoParams;
    detectorParams.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    charucoParams.cameraMatrix = camera_matrix_;
    charucoParams.distCoeffs = dist_coeffs_;
    charucoParams.minMarkers = 1;  
    charucoParams.tryRefineMarkers = true; // 启用标记角点细化，可获得更精准的 ChArUco 角点
    cv::aruco::CharucoDetector detector(*board_, charucoParams, detectorParams);

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedMarkers;
    std::vector<cv::Point2f> charucoCorners;
    std::vector<int> charucoIds;

    // 核心：检测当前帧中的 ChArUco 角点
    detector.detectBoard(image_gray, charucoCorners, charucoIds, markerCorners, markerIds);

    if (charucoIds.empty() && !markerIds.empty()) {
        cv::aruco::interpolateCornersCharuco(
            markerCorners, markerIds, image_gray, board_,
            charucoCorners, charucoIds, camera_matrix_, dist_coeffs_, 1);
    }

    if (charucoIds.empty()) {
        std::vector<int> markerIds2;
        std::vector<std::vector<cv::Point2f>> markerCorners2;
        cv::aruco::detectMarkers(image_gray, dictionary_, markerCorners2, markerIds2, params_);

        if (!markerIds2.empty()) {
            markerIds = markerIds2;
            markerCorners = markerCorners2;
            cv::aruco::interpolateCornersCharuco(
                markerCorners, markerIds, image_gray, board_,
                charucoCorners, charucoIds, camera_matrix_, dist_coeffs_, 1);
        }
    }

    
    // 绘制检测结果
    image.copyTo(image_with_detection);
    image.copyTo(image_marker);

    
    // 绘制检测到的标记
    cv::aruco::drawDetectedMarkers(image_marker, markerCorners, markerIds);

   
    

    // 如果找到足够多的角点，估计位姿
    if (charucoIds.size() >= 4) {
        cv::aruco::drawDetectedCornersCharuco(
            image_with_detection, charucoCorners, charucoIds);

        // 估计标定板位姿
        bool valid = cv::aruco::estimatePoseCharucoBoard(
            charucoCorners, charucoIds, board_,
            camera_matrix_, dist_coeffs_, rvec, tvec);
        /*if (valid) {
            drawCharucoReprojectionError(image_with_detection, charuco_corners, charuco_ids, rvec, tvec, 0.5);
            if (charuco_corners.size() >= 4)
                valid = cv::aruco::estimatePoseCharucoBoard(
                    charuco_corners, charuco_ids, board_, camera_matrix_, dist_coeffs_, rvec, tvec);
            else
                valid = false;
        }*/


        if (valid) {
            // 计算反投影误差并可视化
            //drawCharucoReprojectionError(image_with_detection, charucoCorners, charucoIds, rvec, tvec, -1);

            // 绘制坐标系
            // cv::drawFrameAxes(image_with_detection, camera_matrix_,
            //     dist_coeffs_, rvec, tvec, 0.05f);
            return true;
        }
    }

    return false;
}


bool HandEyeCalibration::detectCalibrationBoard_ChArUco_file(const std::string& filename,
    cv::Mat& image_with_detection,
    cv::Mat& rvec, cv::Mat& tvec) const {

    cv::Mat image = cv::imread(filename);
    if(0==image.cols || 0 == image.rows)
       return false;
    return detectCalibrationBoard(image, image_with_detection, rvec, tvec);
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

        return poseToTransformMatrix(
            values[0] / 1000.0,  // mm to m
            values[1] / 1000.0,
            values[2] / 1000.0,
            values[3],           // degrees
            values[4],
            values[5],
            true
        );
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

cv::Mat HandEyeCalibration::poseToTransformMatrix(double x, double y, double z,
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
    bool clear_existing) {

    if (clear_existing) {
        robot_poses_.clear();
        camera_poses_.clear();
    }

    size_t base_count = robot_poses_.size();  // 已有数据量，新数据从这之后开始

    // 1. 读取机器人位姿（从Excel导出的CSV）
    std::ifstream file(robot_pose_file);
    std::string line;

    while (std::getline(file, line)) {
        // 假设CSV格式: x,y,z,rx,ry,rz (单位mm,度)
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

        cv::Mat rvec, tvec;
        cv::Mat image_with_detection;
        if (detectCalibrationBoard_ChArUco_file(ss.str(), image_with_detection, rvec, tvec)) {
            // 转换为变换矩阵
            cv::Mat R;
            cv::Rodrigues(rvec, R);
            cv::Mat T_camera_calib = cv::Mat::eye(4, 4, CV_64F);
            R.copyTo(T_camera_calib(cv::Rect(0, 0, 3, 3)));
            tvec.copyTo(T_camera_calib(cv::Rect(3, 0, 1, 3)));

            camera_poses_.push_back(T_camera_calib);
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

            std::vector<double> actual_pose(ROB_AXIS_NUM);
            CRresult result = cr_get_tcpActualPose(robotHandle, actual_pose.data());
            assert(result == sucess);
            for (int i = 0; i < ROB_AXIS_NUM; i++)
            {
                printf("%.2f   ", actual_pose[i]);
            }

            std::cout << " \n  jointActualPos \n" << std::endl;

            std::vector<double> actual_joint_pos(ROB_AXIS_NUM);
            result = cr_get_jointActualPos(robotHandle, actual_joint_pos.data());
            for (int i = 0; i < ROB_AXIS_NUM; i++)
            {
                printf("%.2f   ", actual_joint_pos[i]);
            }    
            
            std::cout << " \n" << std::endl;

            // 将pos保存到文件
            VectorWriter writer_tcp(rob_pos_file, 6);
            writer_tcp.appendDoubleVector(actual_pose);

            VectorWriter writer_joint(rob_joint_file, 6);
            writer_joint.appendDoubleVector(actual_joint_pos);

            //std::string robot_pose_str;
            //std::cin.ignore();  // 清除缓冲区
            //std::getline(std::cin, robot_pose_str);

            //cv::Mat T_base_flange = parseRobotPose(robot_pose_str, false);

            cv::Mat T_base_flange = parseRobotPose(actual_pose, false);

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
        // fs << "board_squares_x" << board_->getChessboardSize().width;
        // fs << "board_squares_y" << board_->getChessboardSize().height;
        // fs << "square_length" << board_->getSquareLength();
        // fs << "marker_length" << board_->getMarkerLength();

        for (size_t i = 0; i < robot_poses_.size(); ++i) {
            fs << "robot_pose_" + std::to_string(i) << robot_poses_[i];
            fs << "camera_pose_" + std::to_string(i) << camera_poses_[i];
        }
        fs.release();
        std::cout << "数据已保存到: " << save_path << std::endl;
    }
}

bool HandEyeCalibration::calibrate(cv::HandEyeCalibrationMethod method) {
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

    // 提取旋转和平移
    for (size_t i = 0; i < robot_poses_.size(); ++i) {
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
    std::cout << "T_flange_camera (相机相对于法兰的变换矩阵):" << std::endl;
    std::cout << T_flange_camera_ << std::endl;

    // 分解为平移和欧拉角显示
    cv::Mat R = T_flange_camera_(cv::Rect(0, 0, 3, 3));
    cv::Mat t = T_flange_camera_(cv::Rect(3, 0, 1, 3));

    std::cout << "平移 (米): [" << t.at<double>(0) << ", "
        << t.at<double>(1) << ", " << t.at<double>(2) << "]" << std::endl;

    return true;
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
