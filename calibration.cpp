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

OBCameraIntrinsic g_cam_param;


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

std::string get_pos_dir(int pos){
    return g_posPath+std::to_string(pos) + "/";
}

void convert_joint_2_pos(int pos){
   std::string pos_dir = get_pos_dir(pos);
   std::string rob_joint_file = pos_dir + "rob_joint.csv";
   std::string rob_pos_file = pos_dir + "rob_pos.csv";
   std::ifstream file(rob_joint_file);
   std::string line;
   while (std::getline(file, line)) {
        // 假设CSV格式: x,y,z,rx,ry,rz (单位mm,度)
        std::vector<double> actual_pose(ROB_AXIS_NUM);                
        std::vector<double> joint_pose = parse_robot_joint_pose(line);
        if (joint_pose.empty()) 
           continue;
        cr_kineForward(robotHandle, joint_pose.data(), actual_pose.data());
        VectorWriter writer_tcp(rob_pos_file, 6);
        writer_tcp.appendDoubleVector(actual_pose);
   }
}

void convert_pos_2_joint(int pos)
{
    std::string pos_dir = get_pos_dir(pos);
   std::string rob_joint_file = pos_dir + "rob_joint.csv";
   std::string rob_joint_file_n = pos_dir + "rob_joint_n.csv";
   std::string rob_pos_file = pos_dir + "rob_pos.csv";
   std::ifstream i_joint(rob_joint_file), i_pos(rob_pos_file);
   std::string joint_line, pos_line;
   if(fs::exists(rob_joint_file_n))
      std::remove(rob_joint_file_n.data());
   int i = 0;
   while (std::getline(i_joint, joint_line)) {
        i++;
        std::getline(i_pos, pos_line);
        // 假设CSV格式: x,y,z,rx,ry,rz (单位mm,度)
        std::vector<double> new_joint(ROB_AXIS_NUM);                
        std::vector<double> joint_pose = parse_robot_joint_pose(joint_line);
        std::vector<double> line_pose = parse_robot_joint_pose(pos_line);
        if (joint_pose.empty()) 
           continue;
        cr_kineInverse(robotHandle, line_pose.data(), joint_pose.data(), new_joint.data());
        VectorWriter writer_tcp(rob_joint_file_n, 6);
        writer_tcp.appendDoubleVector(new_joint);
   }
   i_joint.close();
   i_pos.close();
   std::string rob_joint_file_old = pos_dir + "rob_joint_old.csv";
   if(fs::exists(rob_joint_file_old))
      std::remove(rob_joint_file_old.data());
   std::rename(rob_joint_file.data(), rob_joint_file_old.data());
   std::rename(rob_joint_file_n.data(), rob_joint_file.data());
}

// 从csv文件读取 joint pos 然后让机械臂运动到 该pos
void cap_image_by_pos(int pos)
{
    std::string pos_path = get_pos_dir(pos);
    if (!fs::exists(pos_path))
    {
        std::cout << "pos path is not exists   "<<  pos_path << std::endl;
        return;
    }
    // for(int i = 1; i < 9; i++)
    //    convert_joint_2_pos(i);
    // convert_pos_2_joint(pos);
    std::string rob_joint_file = pos_path + "rob_joint.csv";
    if (!fs::exists(rob_joint_file))
    {
        std::cout << "pos_fileis not exists   "<<  rob_joint_file << std::endl;
        return;
    }

    std::string img_path = pos_path + getCurrentTime_to_path_str()+"/";
    fs::create_directories(img_path);

    std::string rob_pos_file = img_path + "rob_pos.csv";
    std::string new_joint_file = img_path + "rob_joint.csv";    

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
        // cv::Mat frame = g_camera->captureOneFrame(1000);
        cv::Mat frame,depthMat;
        g_camera->captureOneFrameWithDepth(frame, depthMat,5000);

        if (frame.empty() || depthMat.empty())
            break;

        std::string img_name = string_format("%s%03d.jpg", img_path.c_str(), idx);
        std::string depth_name = string_format("%s%03d.png", img_path.c_str(), idx);

        idx++;

        cv::imwrite(img_name, frame);
        cv::imwrite(depth_name, depthMat);


        // 保存法兰坐标
        // 从用户输入获取机器人位姿
        // std::cout << "\n=== 保存位姿 " << (idx) << " ===" << std::endl;
        // std::cout << "开始获取机器人法兰位姿:" << std::endl;
        // std::cout << "格式: x,y,z,rx,ry,rz (单位:mm,度)" << std::endl;
        // std::cout << "例如: 500,0,300,0,0,0\n" << std::endl;

        std::vector<double> actual_pose(ROB_AXIS_NUM);
        CRresult result = cr_get_tcpActualPose(robotHandle, actual_pose.data());
        assert(result == sucess);
        for (int i = 0; i < ROB_AXIS_NUM; i++)
        {
            printf("%.2f   ", actual_pose[i]);
        }

        std::cout << " \n  tcpActualPose \n" << std::endl;
        VectorWriter writer_tcp(rob_pos_file, 6);
        writer_tcp.appendDoubleVector(actual_pose);


        //轴角 坐标
        std::cout << " \n  jointActualPos \n" << std::endl;

        std::vector<double> actual_joint_pos(ROB_AXIS_NUM);
        result = cr_get_jointActualPos(robotHandle, actual_joint_pos.data());
        for (int i = 0; i < ROB_AXIS_NUM; i++)
        {
            printf("%.2f   ", actual_joint_pos[i]);
        }    
        
        std::cout << " \n" << std::endl;

        std::cout << " \n  jointActualPos \n" << std::endl;
        VectorWriter writer_joint(new_joint_file, 6);
        writer_joint.appendDoubleVector(actual_joint_pos);
    }
}

void example_caitu_demo()
{
    std::cout << "\n========================================" << std::endl;
    std::cout << "      下面用 已经采集的机械臂位姿 自动采集图像  " << std::endl;
    std::cout << "========================================" << std::endl;

again:    
    std::cout << "退出采集 请输入0  ";
    std::cout << "请选择要使用的位姿 数据 (1-8): ";

    int choice = 0;
    std::cin >> choice;

    if (0==choice) return;
    
    cap_image_by_pos(choice);

    goto again;
}


// 主菜单
void display_menu() {
    std::cout << "\n========================================" << std::endl;
    std::cout << "      机器人标定系统" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "1. 固定轨迹采集图像" << std::endl;
    std::cout << "2. 本地数据拟合验证" << std::endl;
    std::cout << "3. 执行完整标定流程" << std::endl;
    std::cout << "4. 退出程序" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "请选择操作 (1-4): ";
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


    // g_cam_param
    ///< Focal length in x direction
    g_cam_param.fx = 608.212;
    ///< Focal length in y direction        
    g_cam_param.fy = 608.21;
    ///< Optical center abscissa        
    g_cam_param.cx = 648.738;
    ///< Optical center ordinate
    g_cam_param.cy = 392.18;

    g_cam_param.width = 1280;
    g_cam_param.height = 800;

    return 0;
}

void do_calibration(HandEyeCalibration &calibrator, int has_data){
    if (!has_data) {
        std::cerr << "没有找到标定数据" << std::endl;
        return;
    }
    // 所有数据汇总后统一标定一次
    if (calibrator.calibrate()) {
        calibrator.saveCalibration(g_appPath + "hand_eye_result.yaml");
        cv::Point3f test_point(0.1, 0.2, 0.5);
        cv::Point3f result = calibrator.cameraToFlange(test_point);
        std::cout << "转换结果: " << result << std::endl;
    }
}

void do_type_calibration(int use_all_data){
   // 如果 dir 只有一级目录（如 "5"），自动找该目录下最新采集；如果传了时间戳（如 "5/2026-04-29_19-36-55"），直接使用
    std::vector<std::string> arr = { "5", "6", "7", "8" };
    HandEyeCalibration calibrator(g_posPath + "camera_params.yaml");
    bool has_data = false;

    for (auto dir : arr)
    {
        std::string base = g_posPath + dir;
        std::string rob_pos_file = base + "/rob_pos.csv";

        // 如果传的是 "5" 这样的一级目录，找最新时间戳子目录
        auto pos = dir.find('/');
        if (pos == std::string::npos) {
            if (!fs::exists(base)) continue;

            std::vector<std::string> sessions;
            for (auto& entry : fs::directory_iterator(base)) {
                if (entry.is_directory()) {
                    sessions.push_back(entry.path().filename().string());
                }
            }
            if (sessions.empty()) continue;
            std::sort(sessions.begin(), sessions.end());
            std::string latest = sessions.back();

            base = base + "/" + latest;
            rob_pos_file = base + "/rob_pos.csv";
        }

        if (!fs::exists(rob_pos_file)) {
            std::cout << "跳过（无 rob_pos.csv）: " << base << std::endl;
            continue;
        }

        calibrator.loadChessboardDataset(base, rob_pos_file, !has_data);
        
        if(!use_all_data)
           do_calibration(calibrator, true); 
        else
           has_data = true;
    }

    if(use_all_data)
        do_calibration(calibrator, has_data);  
}

void local_fit_demo()
{
      do_type_calibration(0);
      do_type_calibration(1);
}

int  open_camera(){
   if (!g_camera->open()) {
                std::cerr << "打开相机失败" << std::endl;
                return -1;
            }
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
    
    while (true) {
        display_menu();
        std::cin >> choice;

        switch (choice) {
        case 1:
            open_camera();
            example_caitu_demo();
            break;
        case 2:
            local_fit_demo();
            break;       
        case 3:
            open_camera();
            example_full_calibration();
            break;
        case 4:
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