#include "hand_eye_calibration.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iomanip>
#include <filesystem>

#ifdef _WIN32
    #define WIN32_LEAN_AND_MEAN
    #define NOMINMAX
    #include <windows.h>
    #include <io.h>
    #include <fcntl.h>

#else
    #include<unistd.h>
#endif

#include "my_base.h"
#include "OrbbecCamera.hpp"
#include "robotapi.h"
#include "basestruct.h"
#include "putText.h"

CONFIG g_cfg;
std::string g_appPath = "";
std::string g_paramPath = "";
std::string g_posPath = "";
std::string g_calibrationPath = "";

std::shared_ptr<OrbbecCameraV2> g_camera = nullptr;
RobotHandle robotHandle;

namespace fs = std::filesystem;


std::string string_format(const char* format, ...) {
#if 1 // 最大长度限制：1024 - 1
    char buff[4096] = { 0 };
    va_list args;
    va_start(args, format);
    vsprintf(buff, format, args);
    va_end(args);
    std::string str(buff);
    return str;
#else // 无长度限制
    va_list args;
    va_start(args, format);
    int count = vsnprintf(NULL, 0, format, args); // 使用vsnprintf：warning C4996; 使用vsnprintf_s：无法自动计算长度
    va_end(args);
    va_start(args, format);
    char* buff = (char*)malloc(count * sizeof(wchar_t));
    vsnprintf(buff, count, format, args);
    va_end(args);
    std::string str(buff, count);
    free(buff);
    return str;
#endif
}



// 机器人控制相关的函数
namespace RobotControl {

    // 模拟从机器人控制器读取位姿
    static cv::Mat getRobotPose(HandEyeCalibration& calibrator) {
        // 这里应该从机器人控制器读取实时位姿
        // 示例：返回随时间变化的位姿
        static double angle = 0;
        angle += 5.0;
        if (angle > 360) angle = 0;

        return calibrator.poseToTransformMatrix(
            0.5 + 0.1 * sin(angle * CV_PI / 180.0),  // x坐标有小幅变化
            0.0 + 0.1 * cos(angle * CV_PI / 180.0),  // y坐标有小幅变化
            0.3,                                      // z坐标固定
            0, 0, angle, true);                      // 绕Z轴旋转
    }

    // 显示机器人位姿信息
    static void displayRobotPoseInfo(const cv::Mat& pose_matrix) {
        if (pose_matrix.empty()) {
            std::cout << "无效的位姿矩阵" << std::endl;
            return;
        }

        double x = pose_matrix.at<double>(0, 3);
        double y = pose_matrix.at<double>(1, 3);
        double z = pose_matrix.at<double>(2, 3);

        std::cout << "机器人位姿:" << std::endl;
        std::cout << "  位置: (" << std::fixed << std::setprecision(3)
            << x << ", " << y << ", " << z << ") m" << std::endl;
        std::cout << "  位置: (" << std::fixed << std::setprecision(1)
            << x * 1000 << ", " << y * 1000 << ", " << z * 1000 << ") mm" << std::endl;
    }
}

// 相机相关的函数
namespace CameraControl {

    // 模拟从3D相机获取点云
    static std::vector<cv::Point3f> getCameraPoints() {
        // 这里应该是实际的点云获取代码
        // 示例：返回固定模式的点

        std::vector<cv::Point3f> points;

        // 模拟一个在 XY 平面上的网格点
        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                points.push_back(cv::Point3f(
                    i * 0.05,
                    j * 0.05,
                    0.5  // 固定的深度
                ));
            }
        }

        return points;
    }
}

// 示例1：完整标定流程
void example_full_calibration() {
    std::cout << "=== 手眼标定示例程序 ===" << std::endl;
    std::cout << "========================" << std::endl;

    // 1. 创建标定对象
    HandEyeCalibration calibrator;

    std::string camera_file = g_paramPath +"camera_params.yaml";
    // 2. 加载相机内参（可选，如果没有内参文件可以跳过）
    std::cout << "\n步骤1：加载相机内参" << std::endl;
    std::cout << "请先将 相机内参 写入文件 camera_params.yaml，并将该文件放在标定程序同目录下";

    if (std::filesystem::exists(camera_file)) {
        calibrator.loadCameraParams(camera_file);
    }
    else {
        std::cerr << "相机内参加载 失败" << std::endl;
        return;
    }

    // 3. 采集标定数据
    std::cout << "\n步骤2：数据采集" << std::endl;
    std::cout << "需要采集多少个位姿？(建议10-20个): ";
    int num_poses;
    std::cin >> num_poses;

    calibrator.collectCalibrationData(num_poses, "calibration_data.yaml");

    // 从文件加载 标定数据
    calibrator.loadChessboardDataset(g_calibrationPath,g_calibrationPath+"rob_pos.csv");

    // 4. 执行标定
    std::cout << "\n步骤3：执行标定" << std::endl;
    if (calibrator.calibrate(cv::CALIB_HAND_EYE_TSAI)) {
        // 5. 保存标定结果
        calibrator.saveCalibration("hand_eye_calibration.yaml");

        // 6. 测试转换
        std::cout << "\n步骤4：测试坐标转换" << std::endl;
        cv::Point3f point_camera(0.1, 0.2, 0.5);  // 相机坐标系下的点

        cv::Point3f point_flange = calibrator.cameraToFlange(point_camera);
        std::cout << "测试点转换结果:" << std::endl;
        std::cout << "  相机坐标系: (" << point_camera.x << ", "
            << point_camera.y << ", " << point_camera.z << ") m" << std::endl;
        std::cout << "  法兰坐标系: (" << point_flange.x << ", "
            << point_flange.y << ", " << point_flange.z << ") m" << std::endl;
    }
    else {
        std::cout << "标定失败！请检查数据质量。" << std::endl;
    }
}

// 示例2：使用已有标定结果进行坐标转换
void example_coordinate_transformation() {
    std::cout << "\n=== 坐标转换示例 ===" << std::endl;
    std::cout << "=====================" << std::endl;

    // 1. 加载已有标定结果
    HandEyeCalibration calibrator;
    if (!calibrator.loadCalibration("hand_eye_calibration.yaml")) {
        std::cerr << "加载标定结果失败，请先进行标定" << std::endl;
        return;
    }

    // 2. 模拟从3D相机获取的点云
    std::vector<cv::Point3f> points_camera = CameraControl::getCameraPoints();

    // 3. 从机器人获取当前位姿
    cv::Mat T_base_flange = RobotControl::getRobotPose(calibrator);

    // 显示机器人位姿信息
    RobotControl::displayRobotPoseInfo(T_base_flange);

    // 4. 转换到基坐标系
    std::vector<cv::Point3f> points_base =
        calibrator.transformPoints(points_camera, T_base_flange, 1);

    // 5. 输出结果
    std::cout << "\n坐标转换结果：" << std::endl;
    std::cout << "共转换 " << points_camera.size() << " 个点" << std::endl;

    // 显示前几个点的转换结果
    int display_count = std::min(5, (int)points_camera.size());
    for (int i = 0; i < display_count; ++i) {
        std::cout << "点 " << i + 1 << ":" << std::endl;
        std::cout << "  相机坐标系: (" << std::fixed << std::setprecision(3)
            << points_camera[i].x << ", "
            << points_camera[i].y << ", "
            << points_camera[i].z << ") m" << std::endl;
        std::cout << "  基坐标系:   (" << std::fixed << std::setprecision(3)
            << points_base[i].x << ", "
            << points_base[i].y << ", "
            << points_base[i].z << ") m" << std::endl;
    }

    if (points_camera.size() > display_count) {
        std::cout << "... 还有 " << (points_camera.size() - display_count)
            << " 个点未显示" << std::endl;
    }
}

// 示例3：标定结果验证
void example_calibration_verification() {
    std::cout << "\n=== 标定结果验证 ===" << std::endl;
    std::cout << "====================" << std::endl;

    HandEyeCalibration calibrator;
    if (!calibrator.loadCalibration("hand_eye_calibration.yaml")) {
        std::cerr << "加载标定结果失败" << std::endl;
        return;
    }

    // 获取标定结果
    cv::Mat T_flange_camera = calibrator.getCalibrationResult();
    float error = calibrator.getCalibrationError();

    std::cout << "标定误差: " << error << std::endl;
    std::cout << "变换矩阵 T_flange_camera:" << std::endl;
    std::cout << T_flange_camera << std::endl;

    // 测试转换一致性
    std::cout << "\n转换一致性测试：" << std::endl;

    // 测试点1：相机坐标系原点
    cv::Point3f camera_origin(0, 0, 0);
    cv::Point3f flange_origin = calibrator.cameraToFlange(camera_origin);

    std::cout << "相机原点在法兰坐标系: ("
        << flange_origin.x << ", "
        << flange_origin.y << ", "
        << flange_origin.z << ") m" << std::endl;

    // 测试多个点的转换
    std::vector<cv::Point3f> test_points = {
        cv::Point3f(0.1, 0, 0),
        cv::Point3f(0, 0.1, 0),
        cv::Point3f(0, 0, 0.1),
        cv::Point3f(0.1, 0.1, 0.5)
    };

    cv::Mat T_base_flange = calibrator.poseToTransformMatrix(0, 0, 0, 0, 0, 0, true);

    std::cout << "\n多个测试点转换：" << std::endl;
    for (size_t i = 0; i < test_points.size(); ++i) {
        cv::Point3f point_flange = calibrator.cameraToFlange(test_points[i]);
        cv::Point3f point_base = calibrator.cameraToBase(test_points[i], T_base_flange);

        std::cout << "测试点" << i + 1 << ": 相机("
            << test_points[i].x << "," << test_points[i].y << "," << test_points[i].z << ")"
            << " -> 法兰(" << point_flange.x << "," << point_flange.y << "," << point_flange.z << ")"
            << " -> 基座(" << point_base.x << "," << point_base.y << "," << point_base.z << ")"
            << std::endl;
    }
}

// 示例4：实时处理演示
void example_realtime_demo() {
    std::cout << "\n=== 实时处理演示 ===" << std::endl;
    std::cout << "===================" << std::endl;

    HandEyeCalibration calibrator;
    if (!calibrator.loadCalibration("hand_eye_calibration.yaml")) {
        std::cerr << "加载标定结果失败" << std::endl;
        return;
    }

    std::cout << "开始实时处理演示..." << std::endl;
    std::cout << "模拟处理10帧数据，每帧间隔0.5秒" << std::endl;

    for (int frame = 0; frame < 10; ++frame) {
        std::cout << "\n--- 帧 " << frame + 1 << " ---" << std::endl;

        // 获取机器人当前位姿
        cv::Mat T_base_flange = RobotControl::getRobotPose(calibrator);

        // 获取相机点云
        std::vector<cv::Point3f> points_camera = CameraControl::getCameraPoints();

        // 转换到基坐标系
        std::vector<cv::Point3f> points_base =
            calibrator.transformPoints(points_camera, T_base_flange, 1);

        // 输出结果
        std::cout << "检测到 " << points_camera.size() << " 个点" << std::endl;

        // 计算点云中心
        cv::Point3f center(0, 0, 0);
        for (const auto& point : points_base) {
            center.x += point.x;
            center.y += point.y;
            center.z += point.z;
        }

        if (!points_base.empty()) {
            center.x /= points_base.size();
            center.y /= points_base.size();
            center.z /= points_base.size();

            std::cout << "点云中心在基坐标系: ("
                << std::fixed << std::setprecision(3)
                << center.x << ", " << center.y << ", " << center.z << ") m" << std::endl;

            // 计算中心点相对于机器人的位置
            cv::Point3f robot_position(T_base_flange.at<double>(0, 3),
                T_base_flange.at<double>(1, 3),
                T_base_flange.at<double>(2, 3));

            cv::Point3f relative_position(
                center.x - robot_position.x,
                center.y - robot_position.y,
                center.z - robot_position.z
            );

            std::cout << "相对于机器人: ("
                << relative_position.x << ", "
                << relative_position.y << ", "
                << relative_position.z << ") m" << std::endl;
        }

        // 延时，模拟实时处理
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "\n实时处理演示结束" << std::endl;
}


std::vector<double> parse_robot_joint_pose(const std::string& pose_str) {
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

            values.clear();
        }
    }

    return values;
}

std::string getCurrentTime_to_path_str(std::time_t t=0) {
    std::time_t now = t;
    if (0==t)
    {
        now = std::time(nullptr);
    }
        
    char buf[100];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S", std::localtime(&now));
    return buf;
}


void goto_jointpos(double* jointpos)
{
    // double jointpos[6] = { 184.89, 12.75, -73, -28, 87.34, 173 }; //目标点示例数据
    double pose[6] = { 0,0,0,0,0,0 }; //在MoveJ中无影响
    PointControlParaSimple  pointControlParaSimple;
    for (int index = 0; index < ROB_AXIS_NUM; index++)
    {
        pointControlParaSimple.pose[index] = pose[index];
        pointControlParaSimple.jointpos[index] = jointpos[index];
        pointControlParaSimple.tcpOffset[index] = 0;
        pointControlParaSimple.coordinatePose[index] = 0;
        pointControlParaSimple.speed[index] = 30;
        pointControlParaSimple.acc[index] = 30;
    }
    pointControlParaSimple.coordinateType = jointCoordinate;
    pointControlParaSimple.pointTransType = pointTransStop;
    pointControlParaSimple.motiontriggerMode = MovetriggerbyOnlyRpc;
    PointControlPara  pointControlPara;
    CRresult result = cr_move_pointControlPara_transfer(robotHandle, pointControlParaSimple, &pointControlPara);
    assert(result == sucess);
    result = cr_move_joint(robotHandle, pointControlPara, TRUE);
    assert(result == sucess);
}

// 从csv文件读取 joint pos 然后让机械臂运动到 该pos
void cap_image_by_pos(int pos)
{
    std::string pos_path = g_posPath+std::to_string(pos) + "/";
    if (!fs::exists(pos_path))
    {
        std::cout << "pos path is not exists   "<<  pos_path << std::endl;
        return;
    }

    std::string rob_joint_file = pos_path + "rob_joint.csv";
    if (!fs::exists(rob_joint_file))
    {
        std::cout << "pos_fileis not exists   "<<  rob_joint_file << std::endl;
        return;
    }

    std::string img_path = pos_path + getCurrentTime_to_path_str()+"/";
    fs::create_directories(img_path);

    std::string rob_pos_file = img_path + "rob_pos.csv";

    // 1. 读取机器人位姿（从Excel导出的CSV）
    std::ifstream file(rob_joint_file);
    std::string line;
    std::vector<cv::Mat> robot_poses;

    int idx = 0;
    while (std::getline(file, line)) {
        // 假设CSV格式: x,y,z,rx,ry,rz (单位mm,度)
        std::vector<double> joint_pose = parse_robot_joint_pose(line);
        if (joint_pose.empty()) continue;

        // 控制机械臂到 该位置
        goto_jointpos(joint_pose.data());
        
        // 采集图像
        cv::Mat frame = g_camera->captureOneFrame(1000);
        if (frame.empty()) break;

        std::string img_name = string_format("%s%03d.jpg", img_path.c_str(), idx++);

        cv::imwrite(img_name, frame);


        // 保存法兰坐标
        // 从用户输入获取机器人位姿
        std::cout << "\n=== 保存位姿 " << (idx) << " ===" << std::endl;
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
        VectorWriter writer_tcp(rob_pos_file, 6);
        writer_tcp.appendDoubleVector(actual_pose);
    }
}

void example_caitu_demo()
{
    std::cout << "\n========================================" << std::endl;
    std::cout << "      下面用 已经采集的机械臂位姿 自动采集图像  " << std::endl;
    std::cout << "========================================" << std::endl;

again:    
    std::cout << "退出采集 请输入0  ";
    std::cout << "请选择要使用的位姿 数据 (1-5): ";

    int choice = 0;
    std::cin >> choice;

    if (0==choice) return;
    
    cap_image_by_pos(choice);

    goto again;
}


// 主菜单
void display_menu() {
    std::cout << "\n========================================" << std::endl;
    std::cout << "      机器人手眼标定系统" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "1. 执行完整标定流程" << std::endl;
    std::cout << "2. 坐标转换测试（使用已有标定结果）" << std::endl;
    std::cout << "3. 标定结果验证" << std::endl;
    std::cout << "4. 实时处理演示" << std::endl;
    std::cout << "5. 自动采集图像" << std::endl;    
    std::cout << "6. 退出程序" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "请选择操作 (1-6): ";
}


bool GetBasePathInfo() {

    char szPath[2048] = { 0 };
    char* p = szPath;
#ifdef _WIN32
    GetModuleFileNameA(NULL, szPath, sizeof(szPath)); // �õ�ȫ��·��
    char split = '\\';
#else
    readlink("/proc/self/exe", szPath, sizeof(szPath));
    char split = '/';
#endif

    while (strchr(p, split)) // 从后往前找 分隔符
    {
        p = strchr(p, split);
        p++;
    }
    char t = *p;
    *p = 0; // 当前路径放在 szPath 中了
    g_appPath = szPath;

    g_paramPath = g_appPath + "param/";
    g_calibrationPath = g_paramPath + "calibration/";
    g_posPath = g_paramPath + "pos/";
    

    if (!fs::exists(g_paramPath))
        fs::create_directory(g_paramPath);

    if (!fs::exists(g_calibrationPath))
        fs::create_directory(g_calibrationPath);


    return true;
}

int load_config(std::string cfg_file) { 
    cv::FileStorage fs(cfg_file, cv::FileStorage::READ);

    if (!fs.isOpened()) {
        std::cerr << "无法打开文件进行读取!" << std::endl;
        return -1;
    }

    fs["cam_ip"] >> g_cfg.cam_ip;
    fs["cam_port"] >> g_cfg.cam_port;
    fs["fps"] >> g_cfg.fps;
    fs["cam_width"] >> g_cfg.cam_width;
    fs["cam_height"] >> g_cfg.cam_height;

    fs["grid_num_heng"] >> g_cfg.grid_num_heng;
    fs["grid_num_shu"] >> g_cfg.grid_num_shu;
    fs["grid_size"] >> g_cfg.grid_size;

    std::cout << "图像尺寸: " << g_cfg.cam_width << " x " << g_cfg.cam_height << std::endl;

    fs.release();

    return 0;
}

int main() {

    GetBasePathInfo();

    // cv::Mat image_with_detection(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));

    // putTextZH(image_with_detection, "检测到标定板 - 可以保存", cv::Point(10, 90), cv::Scalar(0, 255, 0),20);



    int choice = 0;

    std::setlocale(LC_ALL, "en_US.utf8");

    std::cout << "当前路径：" << g_appPath << std::endl;
    
    load_config(g_appPath+"config.yaml");

    // 初始化机器人
    CRresult result = cr_create_robot(&robotHandle, "192.168.6.6", 2323, "123");  // 机械臂连接 
    if (result != sucess)
        return -1;

    // 使用默认配置
    g_camera = std::make_shared<OrbbecCameraV2>(g_cfg);
    // 打开相机
    if (!g_camera->open()) {
        std::cerr << "打开相机失败" << std::endl;
        return -1;
    }
    
    while (true) {
        display_menu();
        std::cin >> choice;

        switch (choice) {
        case 1:
            example_full_calibration();
            break;
        case 2:
            example_coordinate_transformation();
            break;
        case 3:
            example_calibration_verification();
            break;
        case 4:
            example_realtime_demo();
            break;
        case 5:
            example_caitu_demo();
            break;            
        case 6:
            std::cout << "感谢使用机器人手眼标定系统，再见！" << std::endl;
            return 0;
        default:
            std::cout << "无效选择，请重新输入 (1-5)" << std::endl;
        }

        // 清空输入缓冲区
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }


    g_camera->close();
    g_camera = nullptr;

    result = cr_destroy_robot(robotHandle);
    if (result != sucess)
        return -1;

    return 0;
}