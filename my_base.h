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

    // 内参兼容开关：默认关闭。仅当 camera_params.yaml 为旧版
    // 仅含 K/D（未绑定相机身份）时，显式置 true 才允许继续标定。
    bool allow_legacy_intrinsics = false;

    CONFIG() = default;
};

#endif // MY_CFG_BASE_H
