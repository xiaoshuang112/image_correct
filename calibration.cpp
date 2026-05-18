#include "hand_eye_calibration.hpp"
#include "auto_eye_in_hand_charuco_calibration.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iomanip>
#include <cstdlib>
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
#include "AICamera.hpp"
#ifdef useORB
#include <OrbbecCamera.hpp>
#endif // REL_DBG

#ifdef useOAK
#include "OAKCamera.hpp"
#endif // REL_DBG
#include "robotapi.h"
#include "basestruct.h"
#include "putText.h"

CONFIG g_cfg;
std::string g_appPath = "";
std::string g_paramPath = "";
std::string g_posPath = "";
std::string g_calibrationPath = "";

std::shared_ptr<AICamera> g_camera = nullptr;
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
        if (angle > 360)
         angle = 0;

        return calibrator.poseEulerToTransformMatrix(
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
void convert_pos_euler_2_joint(int pos)
{
    std::string pos_dir = get_pos_dir(pos);
   std::string rob_joint_file = pos_dir + "rob_joint.csv";
   std::string rob_joint_file_n = pos_dir + "rob_joint_n.csv";
   std::string rob_eule_file = pos_dir + "rob_pos_eule.csv";
   std::string axis_angle_file = pos_dir + "rob_pos.csv";
   std::ifstream i_joint(rob_joint_file), i_eule(rob_eule_file);
   std::string joint_line, eule_line, axis_line;
   if(fs::exists(rob_joint_file_n))
      std::remove(rob_joint_file_n.data());
   std::getline(i_joint, joint_line);
   int i = 0;
   while (std::getline(i_eule, eule_line)) {
        i++;       
        // 假设CSV格式: x,y,z,rx,ry,rz (单位mm,度)
        std::vector<double> new_joint(ROB_AXIS_NUM), axisAngle_pose(ROB_AXIS_NUM);                
        std::vector<double> joint_pose = parse_robot_joint_pose(joint_line);
        std::vector<double> eule_pose = parse_robot_joint_pose(eule_line);
        cr_Eule2AxisAngle(eule_pose.data(), axisAngle_pose.data());
        if (joint_pose.empty()) 
           continue;
        CRresult ret = cr_kineInverse(robotHandle, axisAngle_pose.data(), joint_pose.data(), new_joint.data());
        if( 0 != ret)
           continue;

        VectorWriter writer_tcp(rob_joint_file_n, 6);
        writer_tcp.appendDoubleVector(new_joint);
        VectorWriter writer_eule(axis_angle_file, 6);
        writer_eule.appendDoubleVector(axisAngle_pose);// axis-angle pose
   }
   i_joint.close();
   i_eule.close();
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
    // convert_pos_euler_2_joint(9);
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
        VectorWriter writer_tcp(rob_pos_file, 6);
        writer_tcp.appendDoubleVector(flange_pose);


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
    std::cout << "请选择要使用的位姿 数据 (1-9): ";

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
    std::cout << "4. 自动手眼标定（全自动 729 位姿）" << std::endl;
    std::cout << "5. 退出程序" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "请选择操作 (1-5): ";
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

void do_calibration(HandEyeCalibration &calibrator, int has_data, int min_corners = 0){
    if (!has_data) {
        std::cerr << "没有找到标定数据" << std::endl;
        return;
    }
    if (calibrator.calibrate(cv::CALIB_HAND_EYE_TSAI, min_corners)) {
        std::string report = g_appPath + "calibration_quality.yaml";
        calibrator.evaluateCalibration(report, min_corners);
        calibrator.saveCalibration(g_appPath + "hand_eye_result.yaml");
        cv::Point3f test_point(0.1, 0.2, 0.5);
        cv::Point3f result = calibrator.cameraToFlange(test_point);
        std::cout << "转换结果: " << result << std::endl;
    } 
}

void do_type_calibration(int use_all_data, int minIdsNum, float thresh){
   // 如果 dir 只有一级目录（如 "5"），自动找该目录下最新采集；如果传了时间戳（如 "5/2026-04-29_19-36-55"），直接使用
    std::vector<std::string> arr = { "5", "6", "7", "8" };
    HandEyeCalibration calibrator(g_posPath + "camera_params.yaml", minIdsNum, thresh);
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

void local_fit_demo1()
{
    std::cout << "\n========== 三级过滤对比标定 ==========" << std::endl;
    g_posPath = "/home/irobotic/work/caitu/build/param/";
    std::vector<std::string> arr = {"calibration"};// "5", "6", "7", "8" };  

    // 第一步：用宽松条件收集所有有效检测的样本
    HandEyeCalibration collector(g_posPath + "camera_params.yaml", 12, 0.5);
    bool has_any = false;
    for (auto dir : arr) {
        std::string base = g_posPath + dir;
        std::string rob_pos_file = base + "/rob_pos.csv";
        auto pos = dir.find('/');
        
        if (!fs::exists(rob_pos_file)) {
            std::cout << "跳过（无 rob_pos.csv）: " << base << std::endl;
            continue;
        }
        collector.loadChessboardDataset(base, rob_pos_file, !has_any, 0);
        has_any = true;
    }

    if (!has_any || collector.getPoseCount() < 3) {
        std::cerr << "数据不足，无法标定" << std::endl;
        return;
    }

    // 统计角点分布
    const auto& counts = collector.getCornerCounts();
    int total = (int)counts.size();
    int gt8 = 0, gt12 = 0, gt16 = 0;
    for (int c : counts) { if (c > 12) gt8++; if (c >= 16) gt12++; if (c >= 18) gt16++; }
    std::cout << "\n角点分布: 总计 " << total << " 组"
              << ", >12: " << gt8 << ", >=16: " << gt12 << ", >=18: " << gt16 << std::endl;

    // 二级标定：角点数 > 8
    std::cout << "\n--- 二级: 角点数 > 12 (" << gt8 << " 组) ---" << std::endl;
    if (gt8 >= 3) {
        collector.calibrate(cv::CALIB_HAND_EYE_TSAI, 12);
        collector.evaluateCalibration("", 12);
    }

    // 三级标定：严格过滤
    std::cout << "\n--- 三级: 严格过滤 (>=16角点, thresh=0.25) ---" << std::endl;
    HandEyeCalibration strict_calib(g_posPath + "camera_params.yaml", 16, 0.25);
    bool has_strict = false;
    for (auto dir : arr) {
        std::string base = g_posPath + dir;
        std::string rob_pos_file = base + "/rob_pos.csv";
        auto pos = dir.find('/');
       
        if (!fs::exists(rob_pos_file)) continue;
        strict_calib.loadChessboardDataset(base, rob_pos_file, !has_strict);
        has_strict = true;
    }
    if (has_strict && strict_calib.getPoseCount() >= 3) {
        strict_calib.calibrate();
        strict_calib.evaluateCalibration();
        strict_calib.saveCalibration(g_appPath + "hand_eye_result.yaml");
    }

    // 一级标定：全部样本（在 collector 上运行）
    std::cout << "\n--- 一级: 全部样本 (" << total << " 组) ---" << std::endl;
    collector.calibrate(cv::CALIB_HAND_EYE_TSAI, 1);
    collector.evaluateCalibration("", 1);
    collector.dumpCalibrationDebug(g_appPath + "calib_debug_level1.csv", 1);

    // 三级：dump 严格过滤的数据
    strict_calib.dumpCalibrationDebug(g_appPath + "calib_debug_strict.csv");

    std::cout << "\n========== 三级过滤对比完成 ==========" << std::endl;
}


void local_fit_demo()
{
    std::cout << "\n========== 本地数据标定验证 ==========" << std::endl;
    std::cout << "从磁盘读取已有采集数据，使用 CalibrateHandEyeFromDisk 重新标定" << std::endl;

    std::string save_dir = "/home/irobotic/work/caitu/build/param/calibration";
    // std::cout << "输入采集数据目录路径: ";
    // std::cin >> save_dir;

    if (save_dir.empty() || !fs::exists(save_dir)) {
        std::cerr << "目录不存在: " << save_dir << std::endl;
        return;
    }

    // 检查必要文件
    std::string pos_file = save_dir + "/rob_pos.csv";
    if (!fs::exists(pos_file)) {
        std::cerr << "未找到法兰位姿文件: " << pos_file << std::endl;
        return;
    }

    // 至少有一张图片
    bool has_image = false;
    for (auto& entry : fs::directory_iterator(save_dir)) {
        std::string name = entry.path().filename().string();
        if (name.size() >= 4 && name.substr(name.size() - 4) == ".jpg") {
            has_image = true;
            break;
        }
    }
    if (!has_image) {
        std::cerr << "目录中没有 .jpg 图像文件" << std::endl;
        return;
    }

    // 加载相机内参
    cv::Mat camera_matrix, dist_coeffs;
    std::string cam_file = g_paramPath + "camera_params.yaml";
    cv::FileStorage fs_cam(cam_file, cv::FileStorage::READ);
    if (!fs_cam.isOpened()) {
        std::cerr << "无法加载相机内参: " << cam_file << std::endl;
        return;
    }
    fs_cam["camera_matrix"] >> camera_matrix;
    fs_cam["distortion_coefficients"] >> dist_coeffs;
    fs_cam.release();

    if (camera_matrix.empty()) {
        std::cerr << "相机内参矩阵为空" << std::endl;
        return;
    }

    std::cout << "相机内参已加载" << std::endl;
    std::cout << "数据目录: " << save_dir << std::endl;

    CharucoBoardConfig board_cfg;
    CalibrationResult result;

    if (calibrateHandEyeFromDisk(save_dir, camera_matrix, dist_coeffs, board_cfg, result)) {
        std::cout << "\n========== 标定结果 ==========" << std::endl;
        std::cout << "手眼矩阵 T_flange_camera:\n" << result.T_flange_camera << std::endl;
        std::cout << "有效帧数: " << result.valid_frames << std::endl;
        std::cout << "平均误差: " << (result.reproj_error * 1000.0) << " mm" << std::endl;
        std::cout << result.message << std::endl;

        // 保存结果
        cv::FileStorage out(g_appPath + "hand_eye_result.yaml", cv::FileStorage::WRITE);
        out << "T_flange_camera" << result.T_flange_camera;
        out << "camera_matrix" << result.camera_matrix;
        out << "distortion_coefficients" << result.dist_coeffs;
        out << "reprojection_error_mm" << (result.reproj_error * 1000.0);
        out << "valid_frames" << result.valid_frames;
        out.release();
        std::cout << "结果已保存到: hand_eye_result.yaml" << std::endl;
    } else {
        std::cerr << "\n标定失败: " << result.message << std::endl;
    }
}


void checkAndRestoreRobotHand(){
    // 检查机械臂是否上电
    enum RobotModes robotMode;
    CRresult res = cr_get_robotMode(robotHandle, &robotMode);
    if (res == sucess)
    {
        if (robotMode == JointPowerOff)
        {
            std::cout << "机械臂未上电，上电使能" << std::endl;
            res = cr_FaultReset(robotHandle);
            if (res == sucess)
            {
                std::cout << "机械臂故障复位" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
            else
                std::cout << "故障复位失败" << std::endl;
            
            if (res == sucess)
                res = cr_poweron(robotHandle); // 上电
            if (res == sucess)
            {
                std::cout << "机械臂开始上电" << std::endl;
                while (res == sucess)
                {
                    res = cr_get_robotMode(robotHandle, &robotMode);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    if (robotMode == JointIdle)
                    {
                        break;
                    }
                }

                res = cr_enable(robotHandle); // 使能
                if (res == sucess)
                {
                    std::cout << "机械臂开始使能" << std::endl;
                    while (res == sucess)
                    {
                        res = cr_get_robotMode(robotHandle, &robotMode);
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                        if (robotMode == ProgramStop)
                        {
                            break;
                        }
                    }
                }
                else
                    std::cout << "机械臂使能失败 res: " << res << std::endl;
            }
            else
                std::cout << "机械臂上电失败 res: " << res << std::endl;
        }
    }
    else
        std::cout << "获取机械臂模式失败" << std::endl;
}

// ============================================================
// 曝光采样：在当前位姿采集多张不同曝光值的图像
// ============================================================
static bool captureExposureSamples(
    std::shared_ptr<AICamera>& camera,
    const std::string& save_dir,
    int num_samples = 10) {

    int minGain, maxGain;
    if (!camera->getGainRange(minGain, maxGain) || maxGain <= minGain) {
        std::cerr << "[exposureSamples] 获取增益范围失败，使用默认 [0, 255]" << std::endl;
        minGain = 0; maxGain = 255;
    }
    
    // 保存当前曝光状态
    bool was_auto = camera->getAutoExposure();
    int orig_auto_was_on = was_auto;
    int orig_gain = camera->getColorGain();

    // 关闭自动曝光
    camera->setAutoExposure(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 创建保存目录
    namespace fs = std::filesystem;
    if (!fs::exists(save_dir))
        fs::create_directories(save_dir);

    // 生成线性间隔增益值
    std::vector<int> gain_values(num_samples);
    for (int i = 0; i < num_samples; i++)
        gain_values[i] = minGain + (0.65 * maxGain - minGain) * i / (num_samples - 1);

    std::cout << "\n========== 曝光采样开始 ==========" << std::endl;
    std::cout << "增益范围: " << minGain << " ~ " << maxGain << ", "
              << num_samples << " 级" << std::endl;

    // 计算采集次数序号：扫描已有 *x*.jpg 文件取最大前两位序号 +1
    int session = 1;
    for (auto& entry : fs::directory_iterator(save_dir)) {
        std::string name = entry.path().filename().string();
        if (name.size() >= 7 && name.substr(name.size()-4) == ".jpg") {
            int s = std::atoi(name.c_str());
            if (s > 0) session = std::max(session, s + 1);
        }
    }

    int captured = 0;
    for (int i = 0; i < num_samples; i++) {
        if (!camera->setColorGain(gain_values[i])) {
            std::cerr << "[exposureSamples] 设置增益 " << gain_values[i] << " 失败" << std::endl;
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        cv::Mat frame = camera->captureOneFrame(500);
        if (frame.empty()) {
            std::cerr << "[exposureSamples] 采集图像失败 (增益=" << gain_values[i] << ")" << std::endl;
            continue;
        }

        std::string filename = save_dir + "/" +
            (session < 10 ? "0" : "") + std::to_string(session) + "x" +
            (i + 1 < 10 ? "0" : "") + std::to_string(i + 1) + ".jpg";
        cv::imwrite(filename, frame);
        std::cout << "  [" << i << "/" << num_samples << "] 增益="
                  << gain_values[i] << " → " << filename << std::endl;
        captured++;
    }

    // 恢复曝光状态
    if (orig_auto_was_on) {
        camera->setAutoExposure(true);
    } else if (orig_gain >= 0) {
        camera->setColorGain(orig_gain);
    }

    std::cout << "========== 曝光采样完成 (" << captured << "/" << num_samples << ") ==========\n" << std::endl;
    return captured > 0;
}

// ============================================================
// 自动手眼标定入口（菜单项 5）
// ============================================================
//void moveToJointPos(int robot_handle, const double joint[6]);
void auto_handeye_calibration() {
    // 尝试加载已有相机内参
    cv::Mat camera_matrix, dist_coeffs;
    std::string cam_file = g_paramPath + "camera_params.yaml";
    cv::FileStorage fs(cam_file, cv::FileStorage::READ);
    if (fs.isOpened()) {
        fs["camera_matrix"] >> camera_matrix;
        fs["distortion_coefficients"] >> dist_coeffs;
        fs.release();
        if (!camera_matrix.empty())
            std::cout << "[autoCalib] 加载已有相机内参: " << cam_file << std::endl;
    }

    // 实时预览 — 等待用户拖到最佳位姿后按 's'
    cv::namedWindow("Auto Calibration", cv::WINDOW_NORMAL);
    cv::resizeWindow("Auto Calibration", 960, 600);

    HandEyeCalibration detector(cam_file);
    std::cout << "\n----------------------------------------" << std::endl;
    std::cout << "  请拖拽机器人到任意标定板可见的位姿" << std::endl;
    std::cout << "  系统将自动调整到最佳位姿" << std::endl;
    std::cout << "  按 's' 开始自动标定" << std::endl;
    std::cout << "  按 'q' 取消" << std::endl;
    std::cout << "----------------------------------------\n" << std::endl;
    
    checkAndRestoreRobotHand();   

    // double init_tcp_pose[6] = {-703.396, -198.517, 211.509, 140.34, 112.72, 0.00616}, euler_fixed[6];
    double init_tcp_pose[6] = {196.377,-34.607,-46.971,-9.496,90.077,238.137}, euler_fixed[6];
    // CRresult ret = cr_get_tcpActualPose(robotHandle, init_tcp_pose);
    // ret = cr_AxisAngle2Eule(init_tcp_pose, euler_fixed);
    // if (ret == sucess) {        
    //     euler_fixed[3] = -180.0;
    //     euler_fixed[4] = 0.0;
    //     double fixed_aa[ROB_AXIS_NUM];
    //     ret = cr_Eule2AxisAngle(euler_fixed, init_tcp_pose);        
    // }
    TCPMsg tcp_msg;
    bool confirmed = false;
    cv::Mat frame = g_camera->captureOneFrame(1000);
    moveToJointPos(robotHandle, init_tcp_pose);   
    CRresult ret = cr_get_tcpActualPose(robotHandle, init_tcp_pose);
    while (true) {
        cv::Mat frame = g_camera->captureOneFrame(1000);
        if (frame.empty()) continue;

        // 检测 ChArUco 并叠加显示
        cv::Mat display;
        cv::Mat rvec, tvec;
        cv::Mat empty_depth;
        if (detector.detectCalibrationBoard(frame, empty_depth, display, rvec, tvec)) {
            cv::putText(display, "ChArUco OK  Press 's' to start",
                        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                        0.7, cv::Scalar(0, 255, 0), 2);
        } else {
            frame.copyTo(display);
            cv::putText(display, "No board detected",
                        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                        0.7, cv::Scalar(0, 0, 255), 2);
        }

        cv::imshow("Auto Calibration", display);
        int key = cv::waitKey(30);

        if (key == 's' || key == 'S') {
            // 记录初始位姿
            CRresult ret = cr_get_tcpActualPose(robotHandle, init_tcp_pose);
            if (ret != sucess) {
                std::cerr << "[autoCalib] 获取 TCP 位姿失败" << std::endl;
                continue;
            }
            ret = cr_get_currentTCPmsg(robotHandle, &tcp_msg);
            if (ret != sucess) {
                std::cerr << "[autoCalib] 获取 TCP 偏移失败" << std::endl;
                continue;
            }
            confirmed = true;
            break;
        }
        if (key == 'q' || key == 'Q') {
            std::cout << "[autoCalib] 用户取消" << std::endl;
            cv::destroyWindow("Auto Calibration");
            return;
        }
    }
    
    cv::destroyWindow("Auto Calibration");

    if (!confirmed) return;

    // 输出初始位姿
    std::cout << "[autoCalib] 初始 TCP 位姿 (轴角, mm/deg): ";
    // for (int i = 0; i < 6; i++)
    //     std::cout << init_tcp_pose[i] << (i < 5 ? ", " : "\n");

    // 在当前位姿采集曝光样本
    // captureExposureSamples(g_camera, g_paramPath + "exposure/", 10);

    // 配置 ChArUco 板参数
    CharucoBoardConfig board_cfg;
    // 使用与 HandEyeCalibration 一致的默认值（7x5 方格, 40mm 边长, 28mm 标记）

    // 调用标定模块
    CalibrationResult result;
    bool ok = autoEyeInHandCharucoCalibration(
        robotHandle,
        init_tcp_pose,
        tcp_msg.tcpOffset,
        g_camera,
        board_cfg,
        camera_matrix,
        dist_coeffs,
        result,
        g_calibrationPath
    );

    // 输出/保存结果
    if (ok) {
        std::cout << "\n========== 标定结果 ==========" << std::endl;
        std::cout << "手眼矩阵 T_flange_camera:\n" << result.T_flange_camera << std::endl;
        std::cout << "相机内参:\n" << result.camera_matrix << std::endl;
        std::cout << "畸变系数: " << result.dist_coeffs.t() << std::endl;
        std::cout << "有效帧数: " << result.valid_frames << std::endl;
        std::cout << "平均误差: " << (result.reproj_error * 1000.0) << " mm" << std::endl;

        // 保存到文件
        cv::FileStorage out(g_appPath + "auto_handeye_result.yaml", cv::FileStorage::WRITE);
        out << "T_flange_camera" << result.T_flange_camera;
        out << "camera_matrix" << result.camera_matrix;
        out << "distortion_coefficients" << result.dist_coeffs;
        out << "reprojection_error_mm" << result.reproj_error * 1000.0;
        out << "valid_frames" << result.valid_frames;
        out.release();
        std::cout << "结果已保存到: auto_handeye_result.yaml" << std::endl;

        if (camera_matrix.empty()) {
            cv::FileStorage outCam(g_appPath + "camera_params.yaml", cv::FileStorage::WRITE);
            outCam << "camera_matrix" << result.camera_matrix;
            outCam << "distortion_coefficients" << result.dist_coeffs;
            outCam.release();
            std::cout << "相机内参已保存到: camera_params.yaml" << std::endl;
        }
    } else {
        std::cerr << "\n[autoCalib] 标定失败: " << result.message << std::endl;
    }
}

int  open_camera(){
   if (!g_camera->open()) {
                std::cerr << "打开相机失败" << std::endl;
                return -1;
            }

    // 读取相机内参
    auto m = g_camera->getCameraIntrinsics(g_cfg.cam_width, g_cfg.cam_height);        

    // 保存相机参数到文件
    cv::FileStorage out(g_paramPath + "camera_params.yaml", cv::FileStorage::WRITE);
    cv::Mat camera_matrix = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);

    camera_matrix.at<double>(0, 0) = m[0][0];
    camera_matrix.at<double>(1, 1) = m[1][1];
    camera_matrix.at<double>(2, 2) = 1.0;

    camera_matrix.at<double>(0, 2) = m[0][2];
    camera_matrix.at<double>(1, 2) = m[1][2];

    for (size_t i = 0; i < m[3].size(); i++)
    {
        dist_coeffs.at<double>(i, 0) = m[3][i];
    }
    
    out << "camera_matrix" << camera_matrix;
    out << "distortion_coefficients" << dist_coeffs;

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

        int ret = -1;
        switch (choice) {
        case 1:
            ret = open_camera();
            if (-1==ret) break;
            example_caitu_demo();
            break;
        case 2:
            local_fit_demo();
            break;       
        case 3:
            ret = open_camera();
            if (-1==ret) break;
            example_full_calibration();
            break;
        case 4:         
            ret = open_camera();
            if (-1==ret) break;   
            auto_handeye_calibration();   
            break;
        case 5:
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