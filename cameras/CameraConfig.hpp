#pragma once

#include <string>

struct CameraConfig {
    std::string cam_sn;
    std::string cam_ip = "192.168.66.253";
    int cam_width = 4024;
    int cam_height = 3036;
    int exposure_us = 12000;
    int gain = 3;
};
