// 配置三级解析无硬件测试：平铺键 > camera_profile 嵌套节 > 默认值，
// 以及旧版纯平铺配置的兼容行为。
#include "test_check.hpp"

#include "hand_eye_gui/config_resolver.hpp"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

using config_resolver::CameraConfig;

namespace {

const char* kCfgPath = "caitu_test_config.yaml";

CameraConfig makeDefaults() {
    CameraConfig d;
    d.cam_ip = "10.0.0.1";
    d.cam_width = 1280;
    d.cam_height = 800;
    d.exposure_us = 30000;
    d.gain = 3;
    d.cam_sn = "";
    return d;
}

std::string readFile() {
    std::ifstream f(kCfgPath, std::ios::binary);
    std::ostringstream contents;
    contents << f.rdbuf();
    return contents.str();
}

void writeFile(const std::string& content) {
    std::ofstream f(kCfgPath);
    f << content;
}

bool hasSource(const CameraConfig& c, const std::string& needle) {
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
    CameraConfig out;

    // 旧配置中的 cam_port 已废弃，忽略该字段且保留其他配置的读取。
    // ---- 旧版纯平铺配置（无 camera_profile）：兼容读取 ----
    writeFile(
        "%YAML:1.0\n---\n"
        "cam_ip: \"192.168.1.11\"\n"
        "cam_port: 8090\n"
        "cam_width: 2448\n"
        "cam_height: 2048\n"
        "camera_exposure_us: 28000\n"
        "camera_gain: 0\n"
        "fps: 15\n");
    TEST_CHECK(config_resolver::resolveCameraConfigFile(kCfgPath,
                                                        makeDefaults(), out,
                                                        err),
               "legacy_flat_loads");
    TEST_CHECK(out.cam_ip == "192.168.1.11" &&
                   out.cam_width == 2448 && out.cam_height == 2048,
               "legacy_flat_values");
    TEST_CHECK(out.exposure_us == 28000 && out.gain == 0,
               "legacy_flat_exposure_gain");
    TEST_CHECK(out.cam_sn.empty(),
               "legacy_flat_new_keys_default");
    TEST_CHECK(out.profile.empty() && !out.profile_found,
               "legacy_flat_no_profile");
    TEST_CHECK(hasSource(out, "cam_ip=192.168.1.11(flat)"),
               "legacy_flat_source_audit");
    TEST_CHECK(!hasSource(out, "cam_port="), "obsolete_port_ignored");

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
        "  cam_sn: \"DA1234567\"\n");
    TEST_CHECK(config_resolver::resolveCameraConfigFile(kCfgPath,
                                                        makeDefaults(), out,
                                                        err),
               "profile_loads");
    TEST_CHECK(out.cam_ip == "192.168.3.10", "flat_beats_profile_ip");
    TEST_CHECK(out.cam_width == 2448 && out.cam_height == 2048,
               "profile_fills_missing_resolution");
    TEST_CHECK(out.cam_sn == "DA1234567", "profile_fills_missing_sn");
    TEST_CHECK(out.exposure_us == 32000 && out.gain == 4,
               "profile_fills_exposure_gain");
    TEST_CHECK(out.profile == "cam_mvs_a" && out.profile_found,
               "profile_found_flag");
    TEST_CHECK(hasSource(out, "cam_ip=192.168.3.10(flat)") &&
                   hasSource(out, "cam_sn=DA1234567(profile:cam_mvs_a)"),
               "mixed_source_audit");

    // ---- 指定的嵌套节不存在：回退平铺/默认并标记 ----
    writeFile(
        "%YAML:1.0\n---\n"
        "camera_profile: \"cam_missing\"\n"
        "cam_ip: \"192.168.3.10\"\n");
    TEST_CHECK(config_resolver::resolveCameraConfigFile(kCfgPath,
                                                        makeDefaults(), out,
                                                        err),
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
    TEST_CHECK(config_resolver::resolveCameraConfigFile(kCfgPath,
                                                        makeDefaults(), out,
                                                        err),
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
    TEST_CHECK(config_resolver::updateFlatIntegerConfigFile(
                   kCfgPath, "camera_exposure_us", 45000, err),
               "update_existing_exposure");
    TEST_CHECK(readFile().find("camera_exposure_us: 45000 # production default") !=
                   std::string::npos &&
                   readFile().find("cam_ip: \"192.168.3.10\"") != std::string::npos,
               "update_preserves_comment_and_other_keys");
    TEST_CHECK(config_resolver::resolveCameraConfigFile(kCfgPath,
                                                        makeDefaults(), out,
                                                        err) &&
                   out.exposure_us == 45000,
               "updated_exposure_readback");

    // ---- 缺少曝光键时插入顶层键，覆盖 profile 值 ----
    writeFile(
        "%YAML:1.0\n---\n"
        "camera_profile: \"cam_mvs_a\"\n"
        "cam_mvs_a:\n"
        "  camera_exposure_us: 32000\n");
    TEST_CHECK(config_resolver::updateFlatIntegerConfigFile(
                   kCfgPath, "camera_exposure_us", 51000, err),
               "insert_missing_exposure");
    TEST_CHECK(config_resolver::resolveCameraConfigFile(kCfgPath,
                                                        makeDefaults(), out,
                                                        err) &&
                   out.exposure_us == 51000 &&
                   hasSource(out, "camera_exposure_us=51000(flat)"),
               "inserted_flat_exposure_wins");

    // ---- 重复顶层键拒绝修改，避免静默写错目标 ----
    writeFile(
        "%YAML:1.0\n---\n"
        "camera_exposure_us: 30000\n"
        "camera_exposure_us: 40000\n");
    const std::string duplicateBefore = readFile();
    TEST_CHECK(!config_resolver::updateFlatIntegerConfigFile(
                   kCfgPath, "camera_exposure_us", 50000, err),
               "duplicate_exposure_rejected");
    TEST_CHECK(readFile() == duplicateBefore, "duplicate_file_unchanged");

    auto collection = [&](const std::string& body) {
        cv::FileStorage config("%YAML:1.0\n---\n" + body, cv::FileStorage::READ | cv::FileStorage::MEMORY);
        return config_resolver::resolveCollectionConfig(config);
    };
    const std::vector<int> defaultExposures{5000, 8500, 14500, 25000};
    auto collect = collection("cam_ip: \"192.168.66.253\"\n");
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

    // UI writes use the same YAML file, preserve unrelated fields, and can
    // update either conventional block lists or compact inline lists.
    writeFile("%YAML:1.0\n---\ncam_ip: \"192.168.66.253\" # camera\n"
              "fusion_exposures_us: [5000, 8500, 14500, 25000] # bracket\n"
              "cam_width: 4024\ncam_height: 3036\nrobot_ip: \"192.168.66.14\"\n");
    TEST_CHECK(config_resolver::updateFlatStringConfigFile(kCfgPath, "cam_ip", "192.168.66.200", err),
               "camera_ip_persisted");
    TEST_CHECK(config_resolver::updateFlatStringConfigFile(kCfgPath, "robot_ip", "192.168.66.20", err),
               "robot_ip_persisted");
    TEST_CHECK(config_resolver::updateFlatIntegerConfigFile(kCfgPath, "cam_width", 2448, err) &&
               config_resolver::updateFlatIntegerConfigFile(kCfgPath, "cam_height", 2048, err),
               "resolution_persisted");
    TEST_CHECK(config_resolver::updateFlatIntegerSequenceConfigFile(kCfgPath, "fusion_exposures_us", {6000, 18000}, err),
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
    for (const std::string& sequence : {std::string("\n  - 5000\n  - 8500"), std::string("[5000,\n 8500\n]")}) {
        writeFile("%YAML:1.0\n---\nfusion_exposures_us: " + sequence + "\n# keep\nrobot_ip: \"192.168.66.14\"\n");
        TEST_CHECK(config_resolver::updateFlatIntegerSequenceConfigFile(kCfgPath, "fusion_exposures_us", {7000, 22000}, err),
                   "multiline_sequence_updated");
        cv::FileStorage config(kCfgPath, cv::FileStorage::READ);
        TEST_CHECK(config_resolver::resolveCollectionConfig(config).fusion_exposures_us == std::vector<int>({7000, 22000}) &&
                   config["robot_ip"].string() == "192.168.66.14" && readFile().find("# keep") != std::string::npos,
                   "multiline_update_preserves_following_fields");
    }
    writeFile("%YAML:1.0\n---\ncam_ip: \"old#host\" # comment\n");
    TEST_CHECK(config_resolver::updateFlatStringConfigFile(kCfgPath, "cam_ip", "new#host", err) &&
               readFile().find("\"new#host\" # comment") != std::string::npos, "hash_in_quoted_string_preserved");
    const auto beforeInvalidString = readFile();
    TEST_CHECK(!config_resolver::updateFlatStringConfigFile(kCfgPath, "cam_ip", "bad\ninjected: 1", err) &&
               readFile() == beforeInvalidString, "multiline_string_write_rejected");
    std::remove(kCfgPath);
    TEST_CHECK(config_resolver::updateFlatIntegerSequenceConfigFile(kCfgPath, "fusion_exposures_us", defaultExposures, err),
               "missing_config_created_with_exposures");

    // ---- 非法/缺失文件 ----
    TEST_CHECK(!config_resolver::resolveCameraConfigFile(
                   "nonexistent_config.yaml", makeDefaults(), out, err),
               "missing_file_fails");

    std::remove(kCfgPath);
    return testutil::takeFailures();
}

}  // namespace suite
