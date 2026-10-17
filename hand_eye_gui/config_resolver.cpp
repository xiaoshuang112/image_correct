#include "config_resolver.hpp"

#include <cerrno>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <variant>

#if defined(__linux__)
#include <fcntl.h>
#include <unistd.h>
#endif

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

void resolveNonNegativeInt(const cv::FileStorage& fs,
                           const cv::FileNode& profileNode,
                           const std::string& profile,
                           const char* key,
                           int& target,
                           std::vector<std::string>& sources) {
    int value = -1;
    std::string source = "flat";
    if (!fs[key].empty()) {
        fs[key] >> value;
    } else if (!profileNode.empty() && !profileNode[key].empty()) {
        profileNode[key] >> value;
        source = "profile:" + profile;
    }
    if (value >= 0) {
        target = value;
        sources.push_back(std::string(key) + "=" + std::to_string(value) +
                          "(" + source + ")");
    }
}

bool syncFile(const std::filesystem::path& path, std::string& error) {
#if defined(__linux__)
    const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0) {
        error = "cannot open temporary config for fsync: " + path.string() +
                ": " + std::strerror(errno);
        return false;
    }
    int result;
    do {
        result = ::fsync(fd);
    } while (result != 0 && errno == EINTR);
    const int savedErrno = errno;
    ::close(fd);
    if (result != 0) {
        error = "cannot fsync temporary config: " + path.string() + ": " +
                std::strerror(savedErrno);
        return false;
    }
#else
    (void)path;
    (void)error;
#endif
    return true;
}

bool syncDirectory(const std::filesystem::path& path, std::string& error) {
#if defined(__linux__)
    const int fd = ::open(path.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    if (fd < 0) {
        error = "cannot open config directory for fsync: " + path.string() +
                ": " + std::strerror(errno);
        return false;
    }
    int result;
    do {
        result = ::fsync(fd);
    } while (result != 0 && errno == EINTR);
    const int savedErrno = errno;
    ::close(fd);
    if (result != 0) {
        error = "cannot fsync config directory: " + path.string() + ": " +
                std::strerror(savedErrno);
        return false;
    }
#else
    (void)path;
    (void)error;
#endif
    return true;
}

bool isTopLevelKeyLine(const std::string& line,
                       const std::string& key,
                       size_t& colonPosition) {
    if (line.size() <= key.size() || line.compare(0, key.size(), key) != 0)
        return false;
    size_t position = key.size();
    while (position < line.size() &&
           (line[position] == ' ' || line[position] == '\t'))
        ++position;
    if (position >= line.size() || line[position] != ':')
        return false;
    colonPosition = position;
    return true;
}

size_t commentPosition(const std::string& line) {
    char quote = 0;
    for (size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (quote == '"' && c == '\\') { ++i; continue; }
        if (quote) { if (c == quote) quote = 0; }
        else if (c == '\'' || c == '"') quote = c;
        else if (c == '#') return i;
    }
    return std::string::npos;
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
    resolveInt(fs, profileNode, out.profile, "cam_width", out.cam_width, out.sources);
    resolveInt(fs, profileNode, out.profile, "cam_height", out.cam_height, out.sources);
    resolveInt(fs, profileNode, out.profile, "camera_exposure_us",
               out.exposure_us, out.sources);
    resolveNonNegativeInt(fs, profileNode, out.profile, "camera_gain",
                          out.gain, out.sources);
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

CollectionConfig resolveCollectionConfig(const cv::FileStorage& fs,
                                         const CollectionConfig& defaults) {
    CollectionConfig out = defaults;
    out.warnings.clear();
    const auto robot = fs["robot_ip"];
    if (!robot.empty()) {
        if (robot.isString() && !robot.string().empty()) out.robot_ip = robot.string();
        else out.warnings.push_back("robot_ip 无效，使用默认值");
    }
    const auto exposures = fs["fusion_exposures_us"];
    if (!exposures.empty()) {
        std::vector<int> values;
        std::set<int> distinct;
        bool valid = exposures.isSeq() && exposures.size() >= 2 && exposures.size() <= 5;
        if (valid) for (const auto& node : exposures) {
            const int value = node.isInt() ? static_cast<int>(node) : 0;
            if (value < 1 || value > 1000000 || !distinct.insert(value).second) { valid = false; break; }
            values.push_back(value);
        }
        if (valid) out.fusion_exposures_us = values;
        else out.warnings.push_back("fusion_exposures_us 应为 2～5 个不同的正整数（1～1000000 μs），使用默认序列");
    }
    return out;
}

namespace {
using ConfigValue = std::variant<int, std::string, std::vector<int>>;

std::string yamlValue(const ConfigValue& value) {
    if (const auto* number = std::get_if<int>(&value)) return std::to_string(*number);
    std::ostringstream text;
    if (const auto* string = std::get_if<std::string>(&value)) text << std::quoted(*string);
    else {
        text << '[';
        bool first = true;
        for (int number : std::get<std::vector<int>>(value)) {
            if (!first) text << ", ";
            text << number; first = false;
        }
        text << ']';
    }
    return text.str();
}

bool updateFlatConfigValueFile(const std::string& path,
                               const std::string& key,
                               const ConfigValue& value,
                               std::string& error) {
    namespace fs = std::filesystem;
    error.clear();
    if (path.empty() || key.empty() || key.find_first_not_of(
            "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_") !=
            std::string::npos) {
        error = "invalid config path or key";
        return false;
    }

    const fs::path destination(path);
    const fs::path parent = destination.parent_path().empty()
        ? fs::path(".") : destination.parent_path();
    std::error_code ec;
    if (!fs::exists(parent, ec) || ec) {
        error = "config directory does not exist: " + parent.string();
        return false;
    }

    std::string contents;
    fs::perms originalPermissions = fs::perms::unknown;
    if (fs::exists(destination, ec) && !ec) {
        std::ifstream input(destination, std::ios::binary);
        if (!input) {
            error = "cannot open config for reading: " + destination.string();
            return false;
        }
        std::ostringstream buffer;
        buffer << input.rdbuf();
        if (!input.eof() && input.fail()) {
            error = "cannot read config: " + destination.string();
            return false;
        }
        contents = buffer.str();
        originalPermissions = fs::status(destination, ec).permissions();
        ec.clear();
    } else {
        contents = "%YAML:1.0\n---\n";
    }

    const std::string newline = contents.find("\r\n") != std::string::npos
        ? "\r\n" : "\n";
    std::vector<std::string> lines;
    std::istringstream stream(contents);
    std::string line;
    while (std::getline(stream, line)) {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        lines.push_back(line);
    }
    if (lines.empty())
        lines = {"%YAML:1.0", "---"};

    size_t matchCount = 0;
    size_t matchIndex = 0;
    for (size_t index = 0; index < lines.size(); ++index) {
        size_t colonPosition = 0;
        if (isTopLevelKeyLine(lines[index], key, colonPosition)) {
            ++matchCount;
            matchIndex = index;
        }
    }
    if (matchCount > 1) {
        error = "duplicate top-level config key: " + key;
        return false;
    }

    const std::string replacement = key + ": " + yamlValue(value);
    if (matchCount == 1) {
        const std::string originalLine = lines[matchIndex];
        const size_t comment = commentPosition(originalLine);
        lines[matchIndex] = replacement;
        if (comment != std::string::npos)
            lines[matchIndex] += " " + originalLine.substr(comment);
        // Remove a block sequence or wrapped flow sequence belonging to this
        // key. Preserve comments/blank lines and all following top-level keys.
        if (std::holds_alternative<std::vector<int>>(value)) {
            size_t end = matchIndex + 1;
            while (end < lines.size()) {
                const auto& next = lines[end];
                const size_t first = next.find_first_not_of(" \t");
                if (first == std::string::npos || next[first] == '#') { ++end; continue; }
                const bool sequenceItem = next[first] == '-' && first + 1 < next.size() && next[first + 1] == ' ';
                if (first == 0 && !sequenceItem && next[first] != ']') break;
                lines.erase(lines.begin() + static_cast<std::ptrdiff_t>(end));
            }
        }
    } else {
        size_t insertIndex = 0;
        for (size_t index = 0; index < lines.size(); ++index) {
            if (lines[index] == "---") {
                insertIndex = index + 1;
                break;
            }
            if (lines[index].rfind("%YAML", 0) == 0)
                insertIndex = index + 1;
        }
        lines.insert(lines.begin() + static_cast<std::ptrdiff_t>(insertIndex),
                     replacement);
    }

    std::ostringstream updated;
    for (const std::string& updatedLine : lines)
        updated << updatedLine << newline;

    const fs::path temporary = destination.string() + ".tmp.runtime.yaml";
    {
        std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
        if (!output) {
            error = "cannot open temporary config: " + temporary.string();
            return false;
        }
        const std::string updatedContents = updated.str();
        output.write(updatedContents.data(),
                     static_cast<std::streamsize>(updatedContents.size()));
        output.flush();
        if (!output) {
            error = "cannot write temporary config: " + temporary.string();
            output.close();
            fs::remove(temporary, ec);
            return false;
        }
    }
    if (originalPermissions != fs::perms::unknown)
        fs::permissions(temporary, originalPermissions, ec);

    try {
        cv::FileStorage validation(temporary.string(), cv::FileStorage::READ);
        if (!validation.isOpened() || validation[key].empty()) {
            error = "updated config failed YAML validation";
            validation.release();
            fs::remove(temporary, ec);
            return false;
        }
        const auto node = validation[key];
        bool matches = false;
        if (const auto* number = std::get_if<int>(&value))
            matches = node.isInt() && static_cast<int>(node) == *number;
        else if (const auto* string = std::get_if<std::string>(&value))
            matches = node.isString() && node.string() == *string;
        else if (node.isSeq()) {
            std::vector<int> persisted;
            node >> persisted;
            matches = persisted == std::get<std::vector<int>>(value);
        }
        validation.release();
        if (!matches) {
            error = "updated config value failed readback";
            fs::remove(temporary, ec);
            return false;
        }
    } catch (const std::exception& exception) {
        error = exception.what();
        fs::remove(temporary, ec);
        return false;
    }

    if (!syncFile(temporary, error)) {
        fs::remove(temporary, ec);
        return false;
    }
    fs::rename(temporary, destination, ec);
    if (ec) {
        error = "atomic config replace failed: " + ec.message();
        fs::remove(temporary, ec);
        return false;
    }
    return syncDirectory(parent, error);
}
} // namespace

bool updateFlatIntegerConfigFile(const std::string& path, const std::string& key,
                                 int value, std::string& error) {
    return updateFlatConfigValueFile(path, key, value, error);
}

bool updateFlatStringConfigFile(const std::string& path, const std::string& key,
                                const std::string& value, std::string& error) {
    if (value.find_first_of("\r\n") != std::string::npos || value.find('\0') != std::string::npos) {
        error = "config string contains a line break or null character";
        return false;
    }
    return updateFlatConfigValueFile(path, key, value, error);
}

bool updateFlatIntegerSequenceConfigFile(const std::string& path, const std::string& key,
                                         const std::vector<int>& values, std::string& error) {
    return updateFlatConfigValueFile(path, key, values, error);
}

}  // namespace config_resolver
