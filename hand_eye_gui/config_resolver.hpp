#ifndef CONFIG_RESOLVER_HPP
#define CONFIG_RESOLVER_HPP

#include "cameras/CameraConfig.hpp"
#include <opencv2/core/persistence.hpp>

#include <string>
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

struct CollectionConfig {
    std::string robot_ip = "192.168.66.14";
    std::vector<int> fusion_exposures_us{5000, 8500, 14500, 25000};
    std::vector<std::string> warnings;
};

// Missing fields retain defaults. An invalid exposure sequence falls back as
// a whole, never silently truncating or dropping individual bracket values.
CollectionConfig resolveCollectionConfig(const cv::FileStorage& fs,
                                         const CollectionConfig& defaults = {});

// 保留原文件注释与排版，只更新顶层整数键。目标文件通过同目录临时文件
// 校验后原子替换；键不存在时插入到 YAML 文档头之后。
bool updateFlatIntegerConfigFile(const std::string& path,
                                 const std::string& key,
                                 int value,
                                 std::string& error);
bool updateFlatStringConfigFile(const std::string& path,
                                const std::string& key,
                                const std::string& value,
                                std::string& error);
bool updateFlatIntegerSequenceConfigFile(const std::string& path,
                                         const std::string& key,
                                         const std::vector<int>& values,
                                         std::string& error);

}  // namespace config_resolver

#endif  // CONFIG_RESOLVER_HPP
