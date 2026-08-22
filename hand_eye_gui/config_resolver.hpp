#ifndef CONFIG_RESOLVER_HPP
#define CONFIG_RESOLVER_HPP

#include <opencv2/core/persistence.hpp>

#include <string>
#include <vector>

// config.yaml 相机配置三级解析（纯 C++ + OpenCV，无 Qt，可无硬件单测）。
//
// 逐键回退顺序：
//   1. 平铺键 cam_ip/cam_port/cam_width/cam_height/cam_sn/allow_legacy_intrinsics
//   2. camera_profile 显式选择的嵌套节（如 camera_mvs_130/camera_mvs_500）
//   3. 调用方提供的默认值（通常为编译期默认）
//
// 禁止根据枚举顺序猜型号——嵌套节必须由 camera_profile 显式指定。
namespace config_resolver {

struct CameraConfig {
    std::string cam_ip;
    int cam_port = 0;
    int cam_width = 0;
    int cam_height = 0;
    std::string cam_sn;
    bool allow_legacy_intrinsics = false;

    std::string profile;        // camera_profile 原始值（空 = 未指定）
    bool profile_found = false; // 指定的嵌套节是否存在且为 map
    // 每个被解析键的来源审计："key=value(source)"，source 为 flat 或 profile:<名>
    std::vector<std::string> sources;
};

// 从已打开的 FileStorage 解析。defaults 同时提供回退值；解析结果覆盖写入
// out（未被任何配置源设置的键保持 defaults 中的值）。
CameraConfig resolveCameraConfig(const cv::FileStorage& fs,
                                 const CameraConfig& defaults);

// 便捷入口：打开文件并解析。文件不存在或无法解析时返回 false 并填 error，
// 此时 out 不被修改。
bool resolveCameraConfigFile(const std::string& path,
                             const CameraConfig& defaults,
                             CameraConfig& out,
                             std::string& error);

}  // namespace config_resolver

#endif  // CONFIG_RESOLVER_HPP
