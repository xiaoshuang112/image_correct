#ifndef MY_CFG_BASE_H
#define MY_CFG_BASE_H

#include <string>
#include <memory>

// 禁用 MSVC 安全警告
#ifdef _MSC_VER
#pragma warning(disable : 4996)
#endif

#ifdef useMVS
struct CONFIG {
    std::string cam_sn = "";
    std::string cam_ip = "192.168.1.12";
    uint16_t cam_port = 8090;

    // 4056 x 3040
    int cam_width = 1280;
    int cam_height = 1024;
    int fps = 15;
    bool enableDepth = false;
    int grid_num_heng = 9; // # 棋盘格 横向格子数
    int grid_num_shu = 9;  // # 棋盘格 竖向格子数
    float grid_size = 0.015f;  // 15mm

    CONFIG() = default;
};

#else

struct CONFIG {
    std::string cam_sn = "";
    std::string cam_ip = "192.168.1.10";
    uint16_t cam_port = 8090;

    // 4056 x 3040
    int cam_width = 1280;
    int cam_height = 800;
    int fps = 30;
    bool enableDepth = true;
    int grid_num_heng = 9; // # 棋盘格 横向格子数
    int grid_num_shu = 9;  // # 棋盘格 竖向格子数
    float grid_size = 0.015f;  // 15mm

    CONFIG() = default;
};


#endif

typedef struct {
    float   fx;      ///< Focal length in x direction
    float   fy;      ///< Focal length in y direction
    float   cx;      ///< Optical center abscissa
    float   cy;      ///< Optical center ordinate
    int16_t width;   ///< Image width
    int16_t height;  ///< Image height
} AICameraIntrinsic;


// ============================================================
// 结合深度图与彩色图，检测图像中心处的矩形物体
// ============================================================

struct CenterRectConfig {
    double depth_tolerance_ratio = 0.15; // 深度容差比例：tol = max(50, center_depth * ratio)
    double center_ratio = 0.15;          // 中心采样区域占图像短边的比例
    double approx_eps = 0.02;            // 多边形逼近精度系数
    double min_area_ratio = 0.02;        // 最小面积占图像面积比例
    // 已知物理尺寸精化（设 <=0 则跳过），默认关闭
    double phys_width_mm = 1000;
    double phys_height_mm = 1000;
    // 相机内参（fx, fy, cx, cy），用于 3D 计算
    double cam_fx = 611.017;
    double cam_fy = 611.182;
    double cam_cx = 638.487;
    double cam_cy = 404.831;
    // 颜色备选检测（当深度分割失败时回退）
    bool use_color_fallback = true;
    int color_h_low = 90, color_h_high = 140;  // 蓝色 HSV H 范围
    int color_s_min = 50, color_v_min = 50;     // S/V 最低阈值
};



#endif