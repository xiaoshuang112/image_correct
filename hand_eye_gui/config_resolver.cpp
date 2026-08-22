#include "config_resolver.hpp"

namespace config_resolver {

namespace {

void resolveString(const cv::FileStorage& fs,
                   const cv::FileNode& profileNode,
                   const std::string& profile,
                   const char* key,
                   std::string& target,
                   std::vector<std::string>& sources) {
    std::string value;
    fs[key] >> value;
    std::string source = "flat";
    if (value.empty() && !profileNode.empty()) {
        profileNode[key] >> value;
        source = "profile:" + profile;
    }
    if (!value.empty()) {
        target = value;
        sources.push_back(std::string(key) + "=" + value + "(" + source + ")");
    }
}

void resolveInt(const cv::FileStorage& fs,
                const cv::FileNode& profileNode,
                const std::string& profile,
                const char* key,
                int& target,
                std::vector<std::string>& sources) {
    int value = 0;
    if (!fs[key].empty())
        fs[key] >> value;
    std::string source = "flat";
    if (value <= 0 && !profileNode.empty() && !profileNode[key].empty()) {
        profileNode[key] >> value;
        source = "profile:" + profile;
    }
    if (value > 0) {
        target = value;
        sources.push_back(std::string(key) + "=" + std::to_string(value) +
                          "(" + source + ")");
    }
}

// 布尔开关按“节点是否存在”判定：平铺键显式写 0 也视为已设置，
// 不会被嵌套节覆盖。
void resolveBool(const cv::FileStorage& fs,
                 const cv::FileNode& profileNode,
                 const char* key,
                 bool& target,
                 std::vector<std::string>& sources) {
    int value = 0;
    std::string source;
    if (!fs[key].empty()) {
        fs[key] >> value;
        source = "flat";
    } else if (!profileNode.empty() && !profileNode[key].empty()) {
        profileNode[key] >> value;
        source = "profile";
    } else {
        return;
    }
    target = (value != 0);
    sources.push_back(std::string(key) + "=" + (target ? "1" : "0") +
                      "(" + source + ")");
}

}  // namespace

CameraConfig resolveCameraConfig(const cv::FileStorage& fs,
                                 const CameraConfig& defaults) {
    CameraConfig out = defaults;
    out.sources.clear();
    out.profile.clear();
    out.profile_found = false;

    fs["camera_profile"] >> out.profile;

    cv::FileNode profileNode;
    if (!out.profile.empty()) {
        profileNode = fs[out.profile];
        if (!profileNode.empty() && profileNode.isMap()) {
            out.profile_found = true;
        } else {
            profileNode = cv::FileNode();
        }
    }

    resolveString(fs, profileNode, out.profile, "cam_ip", out.cam_ip, out.sources);
    resolveInt(fs, profileNode, out.profile, "cam_port", out.cam_port, out.sources);
    resolveInt(fs, profileNode, out.profile, "cam_width", out.cam_width, out.sources);
    resolveInt(fs, profileNode, out.profile, "cam_height", out.cam_height, out.sources);
    resolveString(fs, profileNode, out.profile, "cam_sn", out.cam_sn, out.sources);
    resolveBool(fs, profileNode, "allow_legacy_intrinsics",
                out.allow_legacy_intrinsics, out.sources);
    return out;
}

bool resolveCameraConfigFile(const std::string& path,
                             const CameraConfig& defaults,
                             CameraConfig& out,
                             std::string& error) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        error = "无法打开配置文件: " + path;
        return false;
    }
    out = resolveCameraConfig(fs, defaults);
    fs.release();
    return true;
}

}  // namespace config_resolver
