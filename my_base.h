#ifndef MY_CFG_BASE_H
#define MY_CFG_BASE_H

#include <string>
#include <memory>

// 禁用 MSVC 安全警告
#ifdef _MSC_VER
#pragma warning(disable : 4996)
#endif

struct CONFIG {
    std::string cam_ip = "192.168.1.10";
    std::string cam_sn = "";    
    uint16_t cam_port = 8090;
    int cam_width = 1280;
    int cam_height = 800;
    int fps = 15;
    bool enableDepth = true;
    int grid_num_heng = 9; // # 棋盘格 横向格子数
    int grid_num_shu = 9;  // # 棋盘格 竖向格子数
    float grid_size = 0.015f;  // 15mm

    CONFIG() = default;
};


#endif