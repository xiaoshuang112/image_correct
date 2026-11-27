#ifndef CONFIG_RESOLVER_HPP
#define CONFIG_RESOLVER_HPP

#include "cameras/CameraConfig.hpp"
#include "RobotMotionLimits.hpp"
#include <opencv2/core/persistence.hpp>

#include <string>
#include <map>
#include <variant>
#include <vector>

// config.yaml 相机配置三级解析（纯 C++ + OpenCV，无 Qt，可无硬件单测）。
//
// 逐键回退顺序：
//   1. 平铺键 cam_ip/cam_width/cam_height、曝光/增益等
//   2. camera_profile 显式选择的嵌套节（如 camera_mvs_130/camera_mvs_500）
//   3. 调用方提供的默认值（通常为编译期默认）
//
// 禁止根据枚举顺序猜型号——嵌套节必须由 camera_profile 显式指定。
namespace config_resolver {

struct ResolvedCameraConfig : CameraConfig {
    explicit ResolvedCameraConfig(const CameraConfig& defaults = {}) : CameraConfig(defaults) {}

    std::string profile;        // camera_profile 原始值（空 = 未指定）
    bool profile_found = false; // 指定的嵌套节是否存在且为 map
    // 每个被解析键的来源审计："key=value(source)"，source 为 flat 或 profile:<名>
    std::vector<std::string> sources;
};

// 从已打开的 FileStorage 解析。defaults 同时提供回退值；解析结果覆盖写入
// out（未被任何配置源设置的键保持 defaults 中的值）。
ResolvedCameraConfig resolveCameraConfig(const cv::FileStorage& fs,
                                        const CameraConfig& defaults);

// Camera 2 uses independent top-level keys only (no camera_profile fallback).
// Empty cam2_ip / cam2_sn means "not configured yet" and is a valid startup state.
struct ResolvedCamera2Config {
    std::string cam_ip;
    std::string cam_sn;
    int cam_width = 4024;
    int cam_height = 3036;
    int exposure_us = 12000;
    int gain = 3;
    std::vector<int> fusion_exposures_us{5000, 8500, 14500, 25000};
    std::vector<std::string> sources;
    std::vector<std::string> warnings;
};

ResolvedCamera2Config resolveCamera2Config(const cv::FileStorage& fs,
                                           const ResolvedCamera2Config& defaults = {});

struct CollectionConfig {
    std::string robot_ip = "192.168.66.14";
    int robot_speed_mm_s = collection::defaultRobotSpeedMmS;
    std::vector<int> fusion_exposures_us{5000, 8500, 14500, 25000};
    // Keep aligned with collection::defaultNinePointYawOffsets and the 10 mm UI defaults.
    double nine_point_offset_x_mm = 10;
    double nine_point_offset_y_mm = 10;
    std::vector<double> nine_point_yaw_deg{0, 15, -15, -20, 0, 20, 15, -15, 0};
    std::vector<std::string> warnings;
};

// Missing fields retain defaults. An invalid exposure sequence falls back as
// a whole, never silently truncating or dropping individual bracket values.
CollectionConfig resolveCollectionConfig(const cv::FileStorage& fs,
                                         const CollectionConfig& defaults = {});

// 保留原文件注释与排版，只更新指定顶层键。目标文件通过同目录临时文件
// 校验后原子替换；键不存在时插入到 YAML 文档头之后。
using ConfigValue = std::variant<int, double, std::string, std::vector<int>, std::vector<double>>;
using ConfigUpdates = std::map<std::string, ConfigValue>;
// Validate and commit all edited settings in one replacement; never partially
// persist a page when a later key is invalid or duplicated in the original.
bool updateFlatConfigFile(const std::string& path, const ConfigUpdates& updates,
                          std::string& error);

}  // namespace config_resolver

#endif  // CONFIG_RESOLVER_HPP
