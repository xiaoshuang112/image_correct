#ifndef MY_CFG_BASE_H
#define MY_CFG_BASE_H

#include <string>
#include <cstdint>

// 禁用 MSVC 安全警告
#ifdef _MSC_VER
#pragma warning(disable : 4996)
#endif

struct CONFIG {
    std::string cam_sn = "";
    std::string cam_ip = "192.168.0.10";
    uint16_t cam_port = 8090;

    // 机器人连接参数
    std::string robot_ip = "192.168.0.6";
    int robot_port = 2323;
    std::string robot_pwd = "123";

    // 4056 x 3040
    int cam_width = 1280;
#ifdef useMVS
    int cam_height = 1024;
    int fps = 15;
#else
    int cam_height = 800;
    int fps = 30;
#endif
    int exposure_us = 30000;
    int gain = 3;
#ifdef useMVS
    bool enableDepth = false;
#else
    bool enableDepth = true;
#endif
    int grid_num_heng = 9; // # 棋盘格 横向格子数
    int grid_num_shu = 9;  // # 棋盘格 竖向格子数
    float grid_size = 0.015f;  // 15mm

    // 内参兼容开关：默认关闭。仅当 camera_params.yaml 为旧版
    // 仅含 K/D（未绑定相机身份）时，显式置 true 才允许继续标定。
    bool allow_legacy_intrinsics = false;

    // 机械臂运行速度百分比 (1~100)，眼在手上标定时通过
    // cr_set_robotSpeedPercent 下发到控制器
    int robotSpeedPercent = 100;

    CONFIG() = default;
};

#endif // MY_CFG_BASE_H
