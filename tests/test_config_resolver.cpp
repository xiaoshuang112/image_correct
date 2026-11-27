// 配置三级解析无硬件测试：平铺键 > camera_profile 嵌套节 > 默认值，
// 以及旧版纯平铺配置的兼容行为。
#include "test_check.hpp"

#include "hand_eye_gui/config_resolver.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

using config_resolver::ResolvedCameraConfig;
namespace fpath = std::filesystem;

namespace {

const char* kCfgPath = "caitu_test_config.yaml";

bool resolveCameraConfigFile(const std::string& path,
                            const CameraConfig& defaults,
                            ResolvedCameraConfig& out,
                            std::string& error) {
    // 与生产读取路径一致：UTF-8 窄路径按 Unicode 解码成原生路径后读字节，
    // 再交给 FileStorage 在内存中解析。按文件名打开时 OpenCV 用 ANSI fopen，
    // 非 ASCII 名字会被当前 ANSI 代码页误解码。
    std::ifstream input(fpath::u8path(path), std::ios::binary);
    if (!input) {
        error = "无法打开配置文件: " + path;
        return false;
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    const std::string contents = buffer.str();
    cv::FileStorage storage(contents, cv::FileStorage::READ | cv::FileStorage::MEMORY);
    if (!storage.isOpened()) {
        error = "无法打开配置文件: " + path;
        return false;
    }
    out = config_resolver::resolveCameraConfig(storage, defaults);
    return true;
}

CameraConfig makeDefaults() {
    CameraConfig d;
    d.cam_ip = "10.0.0.1";
    d.cam_width = 1280;
    d.cam_height = 800;
    d.exposure_us = 30000;
    d.gain = 3;
    return d;
}

std::string readFile(const fpath::path& path) {
    std::ifstream f(path, std::ios::binary);
    std::ostringstream contents;
    contents << f.rdbuf();
    return contents.str();
}

std::string readFile() { return readFile(fpath::u8path(kCfgPath)); }

void writeFile(const std::string& content) {
    std::ofstream f(fpath::u8path(kCfgPath), std::ios::binary);
    f << content;
}

bool hasSource(const ResolvedCameraConfig& c, const std::string& needle) {
    for (const std::string& s : c.sources)
        if (s.find(needle) != std::string::npos)
            return true;
    return false;
}

}  // namespace

namespace suite {

int test_config_resolver() {
    std::cout << "---- config_resolver ----\n";
    std::string err;
    ResolvedCameraConfig out;

    // 旧配置中的 cam_port 已废弃，忽略该字段且保留其他配置的读取。
    // ---- 旧版纯平铺配置（无 camera_profile）：兼容读取 ----
    writeFile(
        "%YAML:1.0\n---\n"
        "cam_ip: \"192.168.1.11\"\n"
        "cam_port: 8090\n"
        "cam_sn: \"ignored-flat\"\n"
        "cam_width: 2448\n"
        "cam_height: 2048\n"
        "camera_exposure_us: 28000\n"
        "camera_gain: 0\n"
        "fps: 15\n");
    TEST_CHECK(resolveCameraConfigFile(kCfgPath,
                                       makeDefaults(), out, err),
               "legacy_flat_loads");
    TEST_CHECK(out.cam_ip == "192.168.1.11" &&
                   out.cam_width == 2448 && out.cam_height == 2048,
               "legacy_flat_values");
    TEST_CHECK(out.exposure_us == 28000 && out.gain == 0,
               "legacy_flat_exposure_gain");
    TEST_CHECK(out.profile.empty() && !out.profile_found,
               "legacy_flat_no_profile");
    TEST_CHECK(hasSource(out, "cam_ip=192.168.1.11(flat)"),
               "legacy_flat_source_audit");
    TEST_CHECK(!hasSource(out, "cam_port="), "obsolete_port_ignored");
    TEST_CHECK(!hasSource(out, "cam_sn="), "obsolete_flat_serial_ignored");

    // ---- 平铺键优先于嵌套节 ----
    writeFile(
        "%YAML:1.0\n---\n"
        "camera_profile: \"cam_mvs_a\"\n"
        "cam_ip: \"192.168.3.10\"\n"
        "cam_mvs_a:\n"
        "  cam_ip: \"192.168.3.77\"\n"
        "  cam_port: 9000\n"
        "  cam_width: 2448\n"
        "  cam_height: 2048\n"
        "  camera_exposure_us: 32000\n"
        "  camera_gain: 4\n"
        "  cam_sn: \"ignored-profile\"\n");
    TEST_CHECK(resolveCameraConfigFile(kCfgPath,
                                       makeDefaults(), out, err),
               "profile_loads");
    TEST_CHECK(out.cam_ip == "192.168.3.10", "flat_beats_profile_ip");
    TEST_CHECK(out.cam_width == 2448 && out.cam_height == 2048,
               "profile_fills_missing_resolution");
    TEST_CHECK(out.exposure_us == 32000 && out.gain == 4,
               "profile_fills_exposure_gain");
    TEST_CHECK(out.profile == "cam_mvs_a" && out.profile_found,
               "profile_found_flag");
    TEST_CHECK(hasSource(out, "cam_ip=192.168.3.10(flat)") &&
                   !hasSource(out, "cam_sn="),
               "mixed_source_audit");

    // ---- 指定的嵌套节不存在：回退平铺/默认并标记 ----
    writeFile(
        "%YAML:1.0\n---\n"
        "camera_profile: \"cam_missing\"\n"
        "cam_ip: \"192.168.3.10\"\n");
    TEST_CHECK(resolveCameraConfigFile(kCfgPath,
                                       makeDefaults(), out, err),
               "missing_profile_loads");
    TEST_CHECK(out.profile == "cam_missing" && !out.profile_found,
               "missing_profile_flagged");
    TEST_CHECK(out.cam_ip == "192.168.3.10" && out.cam_width == 1280,
               "missing_profile_falls_back");

    // ---- 无平铺键时嵌套节全权提供 ----
    writeFile(
        "%YAML:1.0\n---\n"
        "camera_profile: \"cam_mvs_a\"\n"
        "cam_mvs_a:\n"
        "  cam_ip: \"192.168.3.77\"\n"
        "  cam_width: 2448\n"
        "  cam_height: 2048\n");
    TEST_CHECK(resolveCameraConfigFile(kCfgPath,
                                       makeDefaults(), out, err),
               "profile_only_loads");
    TEST_CHECK(out.cam_ip == "192.168.3.77" && out.cam_width == 2448 &&
                   out.cam_height == 2048,
               "profile_only_values");
    TEST_CHECK(out.exposure_us == 30000, "profile_only_exposure_default");

    // ---- 顶层曝光写回：保留注释和其他配置 ----
    writeFile(
        "%YAML:1.0\n---\n"
        "camera_exposure_us: 30000 # production default\n"
        "cam_ip: \"192.168.3.10\"\n");
    TEST_CHECK(config_resolver::updateFlatConfigFile(
                   kCfgPath, {{"camera_exposure_us", 45000}}, err),
               "update_existing_exposure");
    TEST_CHECK(readFile().find("camera_exposure_us: 45000 # production default") !=
                   std::string::npos &&
                   readFile().find("cam_ip: \"192.168.3.10\"") != std::string::npos,
               "update_preserves_comment_and_other_keys");
    TEST_CHECK(resolveCameraConfigFile(kCfgPath,
                                       makeDefaults(), out, err) &&
                   out.exposure_us == 45000,
               "updated_exposure_readback");

    // ---- 缺少曝光键时插入顶层键，覆盖 profile 值 ----
    writeFile(
        "%YAML:1.0\n---\n"
        "camera_profile: \"cam_mvs_a\"\n"
        "cam_mvs_a:\n"
        "  camera_exposure_us: 32000\n");
    TEST_CHECK(config_resolver::updateFlatConfigFile(
                   kCfgPath, {{"camera_exposure_us", 51000}}, err),
               "insert_missing_exposure");
    TEST_CHECK(resolveCameraConfigFile(kCfgPath,
                                       makeDefaults(), out, err) &&
                   out.exposure_us == 51000 &&
                   hasSource(out, "camera_exposure_us=51000(flat)"),
               "inserted_flat_exposure_wins");

    // ---- 重复顶层键拒绝修改，避免静默写错目标 ----
    writeFile(
        "%YAML:1.0\n---\n"
        "camera_exposure_us: 30000\n"
        "camera_exposure_us: 40000\n");
    const std::string duplicateBefore = readFile();
    TEST_CHECK(!config_resolver::updateFlatConfigFile(
                   kCfgPath, {{"camera_exposure_us", 50000}}, err),
               "duplicate_exposure_rejected");
    TEST_CHECK(readFile() == duplicateBefore, "duplicate_file_unchanged");

    // A settings page commits all keys together, preserving nested profiles,
    // comments and exposure order. Failure on any key leaves the original intact.
    writeFile("%YAML:1.0\n---\n# retained\ncam_ip: \"10.0.0.1\" # camera\n"
              "robot_ip: \"10.0.0.2\"\ncamera_profile: \"legacy\"\n"
              "legacy:\n  camera_gain: 7\nfusion_exposures_us:\n  - 5000\n  - 8500\n");
    TEST_CHECK(config_resolver::updateFlatConfigFile(kCfgPath,
        {{"cam_ip", std::string("192.168.66.253")}, {"robot_ip", std::string("192.168.66.14")},
         {"cam_width", 4024}, {"fusion_exposures_us", std::vector<int>{25000, 5000, 8500}}}, err),
        "settings_batch_commits");
    TEST_CHECK(readFile().find("# retained") != std::string::npos &&
        readFile().find("# camera") != std::string::npos &&
        readFile().find("  camera_gain: 7") != std::string::npos &&
        readFile().find("fusion_exposures_us: [25000, 5000, 8500]") != std::string::npos,
        "settings_batch_preserves_unrelated_and_order");
    writeFile("%YAML:1.0\n---\ncam_ip: \"10.0.0.1\"\nrobot_ip: \"10.0.0.2\"\nrobot_ip: \"10.0.0.3\"\n");
    const auto batchBefore = readFile();
    TEST_CHECK(!config_resolver::updateFlatConfigFile(kCfgPath,
        {{"cam_ip", std::string("192.168.66.253")}, {"robot_ip", std::string("192.168.66.14")}}, err)
        && readFile() == batchBefore, "later_duplicate_rolls_back_whole_batch");
    TEST_CHECK(!config_resolver::updateFlatConfigFile(kCfgPath,
        {{"cam_ip", std::string("192.168.66.253")}, {"robot_ip", std::string("bad\nkey: 1")}}, err)
        && readFile() == batchBefore, "batch_injected_string_rejected_before_write");

    auto collection = [&](const std::string& body) {
        cv::FileStorage config("%YAML:1.0\n---\n" + body, cv::FileStorage::READ | cv::FileStorage::MEMORY);
        return config_resolver::resolveCollectionConfig(config);
    };
    const std::vector<int> defaultExposures{5000, 8500, 14500, 25000};
    auto collect = collection("cam_ip: \"192.168.66.253\"\n");
    TEST_CHECK(collect.robot_speed_mm_s == 10, "old_config_retains_default_motion_speed");
    for (int speed : {1, 1000})
        TEST_CHECK(collection("robot_speed_mm_s: " + std::to_string(speed) + "\n").robot_speed_mm_s == speed,
                   "configured_motion_speed_loaded");
    for (const std::string& invalid : {"0", "-1", "1001", "1.5", ".Nan", ".Inf", "\"20\"", "[]"}) {
        const auto result = collection("robot_speed_mm_s: " + invalid + "\n");
        TEST_CHECK(result.robot_speed_mm_s == 10 && !result.warnings.empty(), "invalid_motion_speed_falls_back_with_diagnostic");
    }
    TEST_CHECK(collect.robot_ip == "192.168.66.14" && collect.fusion_exposures_us == defaultExposures,
               "collection_missing_fields_use_defaults");
    collect = collection("robot_ip: \"192.168.66.25\"\nfusion_exposures_us: [4000, 8000, 12000, 16000, 20000]\n");
    TEST_CHECK(collect.robot_ip == "192.168.66.25" &&
               collect.fusion_exposures_us == std::vector<int>({4000, 8000, 12000, 16000, 20000}),
               "collection_explicit_fields_loaded");
    collect = collection("fusion_exposures_us: [6000, 24000]\n");
    TEST_CHECK(collect.fusion_exposures_us == std::vector<int>({6000, 24000}) && collect.warnings.empty(),
               "collection_two_exposures_valid");
    for (const std::string& invalid : {"[]", "[5000]", "[1, 2, 3, 4, 5, 6]", "[5000, 5000]",
            "[0, 5000]", "[-1, 5000]", "[5000, 1000001]", "[5000, 8500.5]", "[5000, \"8500\"]", "\"5000,8500\""}) {
        collect = collection("fusion_exposures_us: " + invalid + "\n");
        TEST_CHECK(collect.fusion_exposures_us == defaultExposures && !collect.warnings.empty(),
                   "collection_invalid_sequence_falls_back_whole");
    }
    collect = collection("robot_ip: \"\"\nfusion_exposures_us: [7000, 21000]\n");
    TEST_CHECK(collect.robot_ip == "192.168.66.14" &&
               collect.fusion_exposures_us == std::vector<int>({7000, 21000}),
               "collection_field_fallback_is_independent");
    const std::vector<double> defaultYaw{0, 15, -15, -20, 0, 20, 15, -15, 0};
    TEST_CHECK(collect.nine_point_offset_x_mm == 10 && collect.nine_point_offset_y_mm == 10 &&
               collect.nine_point_yaw_deg == defaultYaw, "nine_point_plan_defaults_when_absent");
    collect = collection("nine_point_offset_x_mm: 7.5\nnine_point_offset_y_mm: 12.5\n"
                         "nine_point_yaw_deg: [1.25, 2.5, -3.75, -10, 0.1, 10, 7.5, -15, 0.01]\n");
    TEST_CHECK(collect.warnings.empty() && collect.nine_point_offset_x_mm == 7.5 &&
               collect.nine_point_offset_y_mm == 12.5 &&
               collect.nine_point_yaw_deg == std::vector<double>({1.25, 2.5, -3.75, -10, 0.1, 10, 7.5, -15, 0.01}),
               "nine_point_plan_loaded");
    collect = collection("nine_point_offset_x_mm: 40\nnine_point_offset_y_mm: 40\n");
    TEST_CHECK(collect.nine_point_offset_x_mm == 10 && collect.nine_point_offset_y_mm == 10 &&
               !collect.warnings.empty(), "nine_point_diagonal_falls_back");
    for (const std::string& invalid : {"0", "50.01", ".Nan", "\"10\""}) {
        collect = collection("nine_point_offset_x_mm: " + invalid + "\nnine_point_offset_y_mm: 12\n");
        TEST_CHECK(collect.nine_point_offset_x_mm == 10 && collect.nine_point_offset_y_mm == 12 &&
                   !collect.warnings.empty(), "invalid_nine_point_x_falls_back_independently");
    }
    collect = collection("nine_point_yaw_deg: [0, 15, -15, -20, 0, 20, 15, -15]\n");
    TEST_CHECK(collect.nine_point_yaw_deg == defaultYaw && !collect.warnings.empty(),
               "short_nine_point_yaw_falls_back_whole");
    collect = collection("nine_point_yaw_deg: [0, 15, -15, -20, 0.001, 20, 15, -15, 0]\n");
    TEST_CHECK(collect.nine_point_yaw_deg == defaultYaw && !collect.warnings.empty(),
               "tiny_nine_point_yaw_falls_back_whole");

    // UI writes use the same YAML file, preserve unrelated fields, and can
    // update either conventional block lists or compact inline lists.
    writeFile("%YAML:1.0\n---\ncam_ip: \"192.168.66.253\" # camera\n"
              "fusion_exposures_us: [5000, 8500, 14500, 25000] # bracket\n"
              "cam_width: 4024\ncam_height: 3036\nrobot_ip: \"192.168.66.14\"\n");
    TEST_CHECK(config_resolver::updateFlatConfigFile(kCfgPath, {{"cam_ip", "192.168.66.200"}}, err),
               "camera_ip_persisted");
    TEST_CHECK(config_resolver::updateFlatConfigFile(kCfgPath, {{"robot_ip", "192.168.66.20"}}, err),
               "robot_ip_persisted");
    TEST_CHECK(config_resolver::updateFlatConfigFile(kCfgPath, {{"cam_width", 2448}}, err) &&
               config_resolver::updateFlatConfigFile(kCfgPath, {{"cam_height", 2048}}, err),
               "resolution_persisted");
    TEST_CHECK(config_resolver::updateFlatConfigFile(
                   kCfgPath, {{"fusion_exposures_us", std::vector<int>{6000, 18000}}}, err),
               "fusion_sequence_persisted");
    TEST_CHECK(readFile().find("# camera") != std::string::npos && readFile().find("# bracket") != std::string::npos,
               "collection_updates_preserve_comments");
    {
        cv::FileStorage config(kCfgPath, cv::FileStorage::READ);
        const auto loadedCamera = config_resolver::resolveCameraConfig(config, makeDefaults());
        const auto loadedCollection = config_resolver::resolveCollectionConfig(config);
        TEST_CHECK(loadedCamera.cam_ip == "192.168.66.200" && loadedCamera.cam_width == 2448 &&
                   loadedCamera.cam_height == 2048 && loadedCollection.robot_ip == "192.168.66.20" &&
                   loadedCollection.fusion_exposures_us == std::vector<int>({6000, 18000}),
                   "all_collection_fields_survive_restart_read");
    }
    TEST_CHECK(config_resolver::updateFlatConfigFile(kCfgPath, {
        {"nine_point_offset_x_mm", 7.5}, {"nine_point_offset_y_mm", 12.5},
        {"nine_point_yaw_deg", std::vector<double>{1.25, 2.5, -3.75, -10, 0.1, 10, 7.5, -15, 0.01}}}, err),
        "nine_point_plan_persisted");
    {
        cv::FileStorage config(kCfgPath, cv::FileStorage::READ);
        const auto loaded = config_resolver::resolveCollectionConfig(config);
        TEST_CHECK(loaded.nine_point_offset_x_mm == 7.5 && loaded.nine_point_offset_y_mm == 12.5 &&
                   loaded.nine_point_yaw_deg == std::vector<double>({1.25, 2.5, -3.75, -10, 0.1, 10, 7.5, -15, 0.01}) &&
                   loaded.robot_ip == "192.168.66.20", "nine_point_plan_survives_restart_read");
    }
    for (const std::string& sequence : {std::string("\n  - 5000\n  - 8500"), std::string("[5000,\n 8500\n]")}) {
        writeFile("%YAML:1.0\n---\nfusion_exposures_us: " + sequence + "\n# keep\nrobot_ip: \"192.168.66.14\"\n");
        TEST_CHECK(config_resolver::updateFlatConfigFile(
                       kCfgPath, {{"fusion_exposures_us", std::vector<int>{7000, 22000}}}, err),
                   "multiline_sequence_updated");
        cv::FileStorage config(kCfgPath, cv::FileStorage::READ);
        TEST_CHECK(config_resolver::resolveCollectionConfig(config).fusion_exposures_us == std::vector<int>({7000, 22000}) &&
                   config["robot_ip"].string() == "192.168.66.14" && readFile().find("# keep") != std::string::npos,
                   "multiline_update_preserves_following_fields");
    }
    writeFile("%YAML:1.0\n---\ncam_ip: \"old#host\" # comment\n");
    TEST_CHECK(config_resolver::updateFlatConfigFile(kCfgPath, {{"cam_ip", "new#host"}}, err) &&
               readFile().find("\"new#host\" # comment") != std::string::npos, "hash_in_quoted_string_preserved");
    const auto beforeInvalidString = readFile();
    TEST_CHECK(!config_resolver::updateFlatConfigFile(kCfgPath, {{"cam_ip", "bad\ninjected: 1"}}, err) &&
               readFile() == beforeInvalidString, "multiline_string_write_rejected");

    // ---- Unicode 目录与文件名 ----
    // 调用方传入的是 UTF-8 窄字符串（Qt 侧 exePath.toStdString()），
    // std::filesystem::path 对窄字符串按 ANSI 代码页解码，因此 Unicode 名字
    // 只有显式走 u8path 才能指向真正的文件。
    {
        const std::string unicodeDir = "caitu_测试配置_手眼目录";
        const std::string utf8Path = unicodeDir + "/配置.yaml";
        const fpath::path unicodePath = fpath::u8path(utf8Path);
        std::error_code ec;
        fpath::remove_all(fpath::u8path(unicodeDir), ec);
        TEST_CHECK(fpath::create_directory(fpath::u8path(unicodeDir), ec),
                   "unicode_directory_created");

        // ANSI 代码页（此机器 936/GBK）解出的另一个名字：写入前先把诱饵放在那边，
        // 保存后必须仍然只有 UTF-8 名字的文件被改动。
        // 按 ANSI 代码页解码窄字符串是 OpenCV 按文件名打开时走的那条路径。
        // 这段 UTF-8 字节未必能被当前代码页解出，解不出时连文件都打不开，
        // 所以只有在真能解出并写出诱饵时才比较诱饵。
        fpath::path ansiPath;
        bool ansiDecoy = false;
        try {
            ansiPath = fpath::path(utf8Path);
            ansiDecoy = ansiPath != unicodePath;
        } catch (const std::exception&) {
            ansiDecoy = false;
        }
        const std::string decoyContents = "%YAML:1.0\n---\nrobot_speed_mm_s: 300\n";
        bool decoyWritten = false;
        if (ansiDecoy) {
            fpath::create_directory(ansiPath.parent_path(), ec);
            std::ofstream decoy(ansiPath, std::ios::binary);
            if (decoy) {
                decoy << decoyContents;
                decoy.close();
                decoyWritten = true;
            }
        }

        {
            std::ofstream fixture(unicodePath, std::ios::binary);
            fixture << "%YAML:1.0\n---\n# 中文注释\ncam_ip: \"192.168.1.11\"\nrobot_speed_mm_s: 12\n";
        }
        TEST_CHECK(config_resolver::updateFlatConfigFile(utf8Path, {{"cam_ip", "192.168.66.201"}}, err) &&
                   err.empty(), "unicode_path_save_accepts_utf8_name");
        const std::string saved = readFile(unicodePath);
        TEST_CHECK(saved.find("# 中文注释") != std::string::npos &&
                   saved.find("cam_ip: \"192.168.66.201\"") != std::string::npos &&
                   saved.find("robot_speed_mm_s: 12") != std::string::npos,
                   "unicode_path_readback_matches_utf8_name");
        TEST_CHECK(!fpath::exists(unicodePath.parent_path() / fpath::u8path("配置.yaml.tmp.runtime.yaml"), ec),
                   "unicode_temporary_removed_after_save");
        if (decoyWritten)
            TEST_CHECK(readFile(ansiPath) == decoyContents,
                       "unicode_save_does_not_write_ansi_decoded_name");
        TEST_CHECK(resolveCameraConfigFile(utf8Path, makeDefaults(), out, err) &&
                   out.cam_ip == "192.168.66.201", "unicode_path_readback_resolves");

        // 报告错误时使用 UTF-8 名字，而不是 ANSI 代码页解出的乱码。
        const std::string missingUnicodePath = "caitu_测试配置_手眼目录_不存在/配置.yaml";
        err.clear();
        TEST_CHECK(!config_resolver::updateFlatConfigFile(missingUnicodePath, {{"cam_ip", "10.0.0.1"}}, err) &&
                   err.find("测试配置") != std::string::npos,
                   "unicode_error_message_keeps_utf8_name");

        // 无法编码成 UTF-8 的名字只能报错，不能在报告路径时抛出。
        const std::string invalidUtf8 = std::string("caitu_") + char('\xFF') + char('\xFE') + "_bad/配置.yaml";
        bool rejectedWithoutThrow = false;
        err.clear();
        try {
            rejectedWithoutThrow =
                !config_resolver::updateFlatConfigFile(invalidUtf8, {{"cam_ip", "10.0.0.1"}}, err);
        } catch (const std::exception&) {
            rejectedWithoutThrow = false;
        }
        TEST_CHECK(rejectedWithoutThrow && !err.empty(), "invalid_utf8_path_reports_error_without_throw");

        fpath::remove_all(fpath::u8path(unicodeDir), ec);
    }

    std::remove(kCfgPath);
    TEST_CHECK(config_resolver::updateFlatConfigFile(
                   kCfgPath, {{"fusion_exposures_us", defaultExposures}}, err),
               "missing_config_created_with_exposures");

    writeFile(
        "%YAML:1.0\n---\n"
        "cam2_ip: \"10.0.0.88\"\n"
        "cam2_sn: \"SN-CAM2\"\n"
        "cam2_width: 2000\n"
        "cam2_height: 1500\n"
        "camera2_exposure_us: 9000\n"
        "camera2_gain: 2\n"
        "camera2_fusion_exposures_us: [3000, 6000, 12000]\n");
    {
        cv::FileStorage storage(kCfgPath, cv::FileStorage::READ);
        config_resolver::ResolvedCamera2Config defaults;
        const auto cam2 = config_resolver::resolveCamera2Config(storage, defaults);
        TEST_CHECK(cam2.cam_ip == "10.0.0.88" && cam2.cam_sn == "SN-CAM2" &&
                       cam2.cam_width == 2000 && cam2.cam_height == 1500 &&
                       cam2.exposure_us == 9000 && cam2.gain == 2 &&
                       cam2.fusion_exposures_us == std::vector<int>({3000, 6000, 12000}) &&
                       cam2.warnings.empty(),
                   "camera2_flat_resolve");
    }

    writeFile("%YAML:1.0\n---\ncam2_ip: \"\"\ncam2_sn: \"\"\n");
    {
        cv::FileStorage storage(kCfgPath, cv::FileStorage::READ);
        const auto cam2 = config_resolver::resolveCamera2Config(storage);
        TEST_CHECK(cam2.cam_ip.empty() && cam2.cam_sn.empty() && cam2.warnings.empty(),
                   "camera2_empty_identity_allowed");
    }

    writeFile("%YAML:1.0\n---\ncam2_width: -10\ncamera2_gain: -1\ncamera2_fusion_exposures_us: [1,1,2]\n");
    {
        cv::FileStorage storage(kCfgPath, cv::FileStorage::READ);
        config_resolver::ResolvedCamera2Config defaults;
        const auto cam2 = config_resolver::resolveCamera2Config(storage, defaults);
        TEST_CHECK(cam2.cam_width == defaults.cam_width &&
                       cam2.gain == defaults.gain &&
                       cam2.fusion_exposures_us == defaults.fusion_exposures_us &&
                       cam2.warnings.size() == 3,
                   "camera2_invalid_fields_fallback");
    }

    // ---- 非法/缺失文件 ----
    TEST_CHECK(!resolveCameraConfigFile(
                   "nonexistent_config.yaml", makeDefaults(), out, err),
               "missing_file_fails");

    std::remove(kCfgPath);
    return testutil::takeFailures();
}

}  // namespace suite
