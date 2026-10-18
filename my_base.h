#ifndef MY_CFG_BASE_H
#define MY_CFG_BASE_H

#include <string>

// 禁用 MSVC 安全警告
#ifdef _MSC_VER
#pragma warning(disable : 4996)
#endif

struct CONFIG {
    std::string cam_sn = "";
    std::string cam_ip = "192.168.0.10";

    // 4056 x 3040
    int cam_width = 1280;
    int cam_height = 1024;
    int exposure_us = 30000;
    int gain = 3;

    CONFIG() = default;
};

#endif // MY_CFG_BASE_H
