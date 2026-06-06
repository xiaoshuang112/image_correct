#include "auto_eye_in_hand_charuco_calibration.hpp"
#include "charuco_detector.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iomanip>
#include <cstdlib>
#include <cstring>
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
#include "cameras/AICamera.hpp"

#ifdef useORB
#include "cameras/OrbbecCamera.hpp"
#endif // REL_DBG

#ifdef useOAK
#include "cameras/OAKCamera.hpp"
#endif // REL_DBG

#ifdef useMVS
#include "cameras/MVSCamera.hpp"
#endif // useMVS


#include "robotapi.h"
#include "basestruct.h"
#include "utils/putText.h"
#include "circlesampler/CircleSampler.h"
#include "piecesampler/PieceSampler.h"
#include "piecesampler/Config.h"
#include "testcameradeepth/DepthTestConfig.h"
#include "testcameradeepth/DepthRemedy.h"
#include "testcameradeepth/DepthTester.h"

CONFIG g_cfg;
std::string g_appPath = "";
std::string g_paramPath = "";
std::string g_posPath = "";
std::string g_calibrationPath = "";

std::shared_ptr<AICamera> g_camera = nullptr;
RobotHandle robotHandle;

namespace fs = std::filesystem;

AICameraIntrinsic g_cam_param;


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



// 主菜单
void display_menu() {
    std::cout << "\n========================================" << std::endl;
    std::cout << "      机器人标定系统" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "1. 本地数据拟合验证" << std::endl;
    std::cout << "2. 自动手眼标定（全自动 729 位姿）" << std::endl;
    std::cout << "3. 螺纹圆形采样" << std::endl;
    std::cout << "4. CircleSample 测试图采集" << std::endl;
    std::cout << "5. 工件检测采样" << std::endl;
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

void local_fit_demo()

{
    std::cout << "\n========== 本地数据标定验证 ==========" << std::endl;
    std::cout << "从磁盘读取已有采集数据，使用 CalibrateHandEyeFromDisk 重新标定" << std::endl;

    std::string save_dir = g_appPath + "param/calibration";
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

    CharucoBoardConfig board_cfg;
    cv::Mat camera_matrix, dist_coeffs;

    // 尝试加载已有相机内参，否则从图像标定
    std::string cam_file = g_paramPath + "camera_params.yaml";
    cv::FileStorage fs_cam(cam_file, cv::FileStorage::READ);
    if (fs_cam.isOpened()) {
        fs_cam["camera_matrix"] >> camera_matrix;
        fs_cam["distortion_coefficients"] >> dist_coeffs;
        fs_cam.release();
    }

    if (true) //camera_matrix.empty()) 
    {
        std::cout << "未找到已有相机内参，从采集图像标定..." << std::endl;
        CharucoDetector detector(board_cfg);
        if (!calibrateCameraFromDisk(save_dir, detector, camera_matrix, dist_coeffs)) {
            std::cerr << "相机内参标定失败" << std::endl;
            return;
        }
    } else {
        std::cout << "相机内参已从文件加载" << std::endl;
    }

    std::cout << "数据目录: " << save_dir << std::endl;

    CalibrationResult result;

    if (calibrateHandEyeFromDisk(save_dir, camera_matrix, dist_coeffs, board_cfg, result,
                                 g_appPath + "hand_eye_result.yaml")) {
        // 结果已由 calibrateHandEyeFromDisk 输出并保存
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
// 自动手眼标定入口（菜单项 5）
// ============================================================
//void moveToJointPos(int robot_handle, const double joint[6]);
// extern bool moveToAnglePos(int robot_handle, double* anglePos);
void auto_handeye_calibration(int cameraToTcpEnd, char *pchRobotIP) {
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

    CharucoBoardConfig board_cfg;
    CharucoDetector detector(board_cfg, cam_file);
    std::cout << "\n----------------------------------------" << std::endl;
    std::cout << "  请拖拽机器人到任意标定板可见的位姿" << std::endl;
    std::cout << "  系统将自动调整到最佳位姿" << std::endl;
    std::cout << "  按 's' 开始自动标定" << std::endl;
    std::cout << "  按 'q' 取消" << std::endl;
    std::cout << "----------------------------------------\n" << std::endl;
    
    checkAndRestoreRobotHand();   
    CRresult ret;
    // double init_tcp_pose[6] = {-703.396, -198.517, 211.509, 140.34, 112.72, 0.00616}, euler_fixed[6];
    //double init_tcp_pose[6] = {-502.377, 204.607, 207.971,1.76, -0.918, -47.680}, euler_fixed[6];// offseted
    double init_flangeAxisPos[6]={-676.9377, 155.607, 275.971, 146.58, 93.1018, 7.71780}; // no-offset
    double init_tcp_pose[6] = {-676.9377, 155.607, 275.971, 146.58, 93.1018, 7.71780}, euler_fixed[6];// no-offseted
    
   
    TCPMsg tcp_msg;
    bool confirmed = false;
   // moveToTcpAxisPos(robotHandle, init_tcp_pose);
    cv::Mat frame = g_camera->captureOneFrame(1000);
   
    //cv::Mat frame2 = g_camera->captureOneFrame(1000);
    ret = cr_get_tcpActualPose(robotHandle, init_tcp_pose);
    convertTcpAxisPos2FlangPos(robotHandle, init_tcp_pose, init_flangeAxisPos);

    while (true) {
        cv::Mat frame = g_camera->captureOneFrame(1000);
        if (frame.empty()) continue;

        // 检测 ChArUco 并叠加显示
        cv::Mat display;
        cv::Mat rvec, tvec;
        if (detector.detectAndDraw(frame, display, rvec, tvec)) {
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

    // CharucoBoardConfig 使用默认值（7x5 方格, 40mm 边长, 28mm 标记）

    // 调用标定模块
    CalibrationResult result;
    bool ok = autoEyeInHandCharucoCalibration(
        robotHandle,
        pchRobotIP,
        g_camera,
        board_cfg,
        camera_matrix,
        dist_coeffs,
        cameraToTcpEnd,
        result,
        g_calibrationPath,
        g_appPath + "auto_handeye_result.yaml"
    );

    if (ok) {
        // 结果已由 calibrateHandEyeFromDisk 输出并保存
        // 额外保存相机内参（仅当本次标定了新内参时）
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

    if (!m.empty())
    {
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
    }
    

    return 0;
}

// ============================================================
// 螺纹圆形采样入口（菜单项 5）
// ============================================================
static void screw_sampling() {
    
    ScrewSamplingConfig sscfg;
    std::string cfg_path = g_appPath + "circlesampler/screw_sampling_config.yaml";
    bool have_cfg = loadScrewSamplingConfig(cfg_path, sscfg);

    if (have_cfg) {
        std::cout << "已加载配置: " << cfg_path << std::endl;
    } else {
        std::cout << "未找到配置文件，使用交互式输入" << std::endl;
        std::cout << "\n目标点坐标 (x y z mm, 每行一组, 输入 0 0 0 完成):" << std::endl;
        double x, y, z;
        while (std::cin >> x >> y >> z) {
            if (x == 0.0 && y == 0.0 && z == 0.0) break;
            sscfg.positions.push_back(cv::Point3d(x, y, z));
        }
        std::cin.clear();

        std::cout << "采样半径 (输入 0 结束, 空格分隔, 如: 80 100 120 0): ";
        double r;
        while (std::cin >> r && r > 0.001)
            sscfg.radii.push_back(r);
        std::cin.clear();

        std::cout << "背景类型 [" << sscfg.background_type << "]: ";
        std::cin >> sscfg.background_type;
    }

    sscfg.output_dir = g_paramPath + "screw_samples";

    std::cout << "\n配置确认:";
    for (size_t i = 0; i < sscfg.positions.size(); i++) {
        std::cout << "\n  目标点 " << i << ": (" << sscfg.positions[i].x << ", "
                  << sscfg.positions[i].y << ", " << sscfg.positions[i].z << ") mm";
    }
    std::cout << "\n  半径: [";
    for (size_t i = 0; i < sscfg.radii.size(); i++) {
        if (i) std::cout << ", ";
        std::cout << sscfg.radii[i];
    }
    std::cout << "] mm";
    std::cout << "\n  偏航角: [";
    for (size_t i = 0; i < sscfg.yaw_angles.size(); i++) {
        if (i) std::cout << ", ";
        std::cout << sscfg.yaw_angles[i] << "°";
    }
    std::cout << "]";
    std::cout << "\n  曝光数/位姿: " << sscfg.num_exposures;
    std::cout << "\n  输出目录: " << sscfg.output_dir;
    std::cout << "\n  总采样数: " << (sscfg.positions.size() * sscfg.radii.size() * sscfg.yaw_angles.size() * 180 * sscfg.num_exposures) << std::endl;

    // std::cout << "\n开始采样? (y/n): ";
    // char confirm = 'n';
    // std::cin >> confirm;
    // if (confirm != 'y' && confirm != 'Y') {
    //     std::cout << "已取消" << std::endl;
    //     return;
    // }

    {
        CircleSampler sampler(g_camera, sscfg);
        //g_camera->setColorExposureTime(42);    
        int total = sampler.run();
        std::cout << "\n采样完成，共 " << total << " 张图像" << std::endl;
    }
}

void try_camera_exposure(){
     for (size_t i = 0; i < 100; i++)
    {
        bool bRet = g_camera->setColorExposureTime(5000+i*2);

        if (!bRet)
        {
            std::cout<< "error "<<std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(200));   
    }
}

// ============================================================
// CircleSample 测试图采集
// ============================================================
static void circle_test_collection(int argc, char** argv) {
    // Parse args: --collect-test <config_path> [--dry-run]
    std::string cfg_path;
    bool dry_run = false;
    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "--dry-run") == 0) {
            dry_run = true;
        } else if (argv[i][0] != '-') {
            cfg_path = argv[i];
        }
    }

    if (cfg_path.empty()) {
        cfg_path = g_appPath + "circlesampler/circle_test_config.yaml";
        std::cout << "使用默认配置路径: " << cfg_path << std::endl;
    }

    CircleTestConfig cfg;
    if (!loadCircleTestConfig(cfg_path, cfg)) {
        std::cerr << "加载测试配置失败" << std::endl;
        return;
    }

    // Try to load T_flange_camera from hand_eye_result.yaml as fallback
    if (cfg.T_flange_camera.empty()) {
        cv::FileStorage hfs(g_appPath + "hand_eye_result.yaml", cv::FileStorage::READ);
        if (hfs.isOpened()) {
            cv::Mat T;
            hfs["T_flange_camera"] >> T;
            if (!T.empty() && T.size() == cv::Size(4, 4))
                cfg.T_flange_camera = T;
        }
    }

    if (!validateCircleTestConfig(cfg)) {
        std::cerr << "测试配置校验失败" << std::endl;
        return;
    }

    // Open camera if needed
    if (!g_camera || !g_camera->isOpened()) {
#ifdef useORB
        g_camera = std::make_shared<OrbbecCameraV2>(g_cfg);
#elif defined(useOAK)
        g_camera = std::make_shared<OAKCamera>(g_cfg);
#else
        g_camera = std::make_shared<MVSCamera>(g_cfg);
#endif
        if (!g_camera->open()) {
            std::cerr << "打开相机失败" << std::endl;
            return;
        }
    }

    if (dry_run) {
        std::cout << "\n========== DRY RUN MODE ==========" << std::endl;
        std::cout << "仅验证 IK 可达性，不拍照" << std::endl;
        CircleSampler sampler(g_camera, cfg);
        sampler.dryRunTestCollection();
        return;
    }

    CircleSampler sampler(g_camera, cfg);
    int total = sampler.runTestCollection();
    std::cout << "\n采集完成，共 " << total << " 张图像" << std::endl;
    std::cout << "输出目录: " << cfg.output_dir << std::endl;
}

// ============================================================
// 工件检测采样（菜单项 5）
// ============================================================
static void piece_sampling() {
    PieceSamplerConfig cfg;
    std::string cfg_path = g_appPath + "piecesampler/piece_sampling_config.yaml";
    if (!loadPieceSamplerConfig(cfg_path, cfg)) {
        std::cerr << "加载工件采样配置失败: " << cfg_path << std::endl;
        return;
    }

    // Try to load T_flange_camera from hand_eye_result.yaml as fallback
    if (cfg.T_flange_camera.empty()) {
        cv::FileStorage hfs(g_appPath + "hand_eye_result.yaml", cv::FileStorage::READ);
        if (hfs.isOpened()) {
            cv::Mat T;
            hfs["T_flange_camera"] >> T;
            if (!T.empty() && T.size() == cv::Size(4, 4))
                cfg.T_flange_camera = T;
        }
    }

    if (!validatePieceSamplerConfig(cfg)) {
        std::cerr << "工件采样配置校验失败" << std::endl;
        return;
    }

    // Open camera if needed
    if (!g_camera || !g_camera->isOpened()) {
#ifdef useORB
        g_camera = std::make_shared<OrbbecCameraV2>(g_cfg);
#elif defined(useOAK)
        g_camera = std::make_shared<OAKCamera>(g_cfg);
#else
        g_camera = std::make_shared<MVSCamera>(g_cfg);
#endif
        if (!g_camera->open()) {
            std::cerr << "打开相机失败" << std::endl;
            return;
        }
    }

    PieceSampler sampler(g_camera, cfg);
    int total = sampler.run();
    std::cout << "\n采样完成，共 " << total << " 张图像" << std::endl;
    std::cout << "输出目录: " << cfg.output_dir << std::endl;
}

// ============================================================
// 深度陈旧帧测试
// ============================================================
static void depth_test(const std::string& cfg_path) {
    DepthTestConfig cfg;
    if (!loadDepthTestConfig(cfg_path, cfg)) {
        std::cerr << "加载深度测试配置失败: " << cfg_path << std::endl;
        return;
    }

    // Open camera
    if (!g_camera || !g_camera->isOpened()) {
#ifdef useORB
        g_camera = std::make_shared<OrbbecCameraV2>(g_cfg);
#elif defined(useOAK)
        g_camera = std::make_shared<OAKCamera>(g_cfg);
#else
        g_camera = std::make_shared<MVSCamera>(g_cfg);
#endif
        if (!g_camera->open()) {
            std::cerr << "打开相机失败" << std::endl;
            return;
        }
    }

    DepthTester tester(g_camera, robotHandle, cfg);
    tester.run();
}

int main(int argc, char** argv) {

    GetBasePathInfo();

    // cv::Mat image_with_detection(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));

    // putTextZH(image_with_detection, "检测到标定板 - 可以保存", cv::Point(10, 90), cv::Scalar(0, 255, 0),20);



    // CLI 参数：--collect-test 直接进入测试采集
    if (argc >= 3 && strcmp(argv[1], "--collect-test") == 0) {
        circle_test_collection(argc, argv);
        return 0;
    }

    // CLI 参数：--depth-test 直接进入深度测试
    if (argc >= 3 && strcmp(argv[1], "--depth-test") == 0) {
        depth_test(argv[2]);
        return 0;
    }

    // CLI 参数：--piece-sampler 直接进入工件采样
    if (argc >= 2 && strcmp(argv[1], "--piece-sampler") == 0) {
        piece_sampling();
        return 0;
    }

    int choice = 0;

    std::setlocale(LC_ALL, "en_US.utf8");

    std::cout << "当前路径：" << g_appPath << std::endl;

    load_config(g_appPath+"config.yaml");

    // 初始化机器人
    char *pchRobotIp = "192.168.6.6";

    CRresult result;
    result = cr_create_robot(&robotHandle, pchRobotIp, 2323, "123");  // 机械臂连接 
    if (result != sucess){
        std::cout <<"[main] 连接机械臂失败\n"<<std::flush;
        return -1;
    }
    // 使用默认配置

#ifdef useORB

    g_camera = std::make_shared<OrbbecCameraV2>(g_cfg);
#else    
    g_camera = std::make_shared<MVSCamera>(g_cfg);
#endif

    // int cam = open_camera();

    //bool bRet = g_camera->setColorExposureTime(12);
   // try_camera_exposure();
        
    while (true) {
        display_menu();
        std::cin >> choice;

        int ret = -1;
        switch (choice) {
        case 1:
            local_fit_demo();
            break;
        case 2:
            ret = open_camera();
            if (-1==ret) break;
            auto_handeye_calibration(150, pchRobotIp);
            break;
        case 3:
            screw_sampling();
            break;
        case 4: 
            circle_test_collection(0, nullptr);
            break;        
        case 5:
            piece_sampling();
            break;
        case 6:
            std::cout << "感谢使用机器人手眼标定系统，再见！" << std::endl;
            return 0;
        default:
            std::cout << "无效选择，请重新输入 (1-6)" << std::endl;
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