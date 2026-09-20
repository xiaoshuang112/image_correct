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
    d.cam_port = 8090;
    d.cam_width = 1280;
    d.cam_height = 800;
    d.exposure_us = 30000;
    d.gain = 3;
    d.cam_sn = "";
    d.allow_legacy_intrinsics = false;
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

    // ---- 旧版纯平铺配置（无 camera_profile）：完全兼容 ----
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
    TEST_CHECK(out.cam_ip == "192.168.1.11" && out.cam_port == 8090 &&
                   out.cam_width == 2448 && out.cam_height == 2048,
               "legacy_flat_values");
    TEST_CHECK(out.exposure_us == 28000 && out.gain == 0,
               "legacy_flat_exposure_gain");
    TEST_CHECK(out.cam_sn.empty() && !out.allow_legacy_intrinsics,
               "legacy_flat_new_keys_default");
    TEST_CHECK(out.profile.empty() && !out.profile_found,
               "legacy_flat_no_profile");
    TEST_CHECK(hasSource(out, "cam_ip=192.168.1.11(flat)"),
               "legacy_flat_source_audit");

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
    TEST_CHECK(out.cam_port == 9000, "profile_fills_missing_port");
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
    TEST_CHECK(out.cam_ip == "192.168.3.10" && out.cam_port == 8090,
               "missing_profile_falls_back");

    // ---- 无平铺键时嵌套节全权提供 ----
    writeFile(
        "%YAML:1.0\n---\n"
        "camera_profile: \"cam_mvs_a\"\n"
        "cam_mvs_a:\n"
        "  cam_ip: \"192.168.3.77\"\n"
        "  cam_width: 2448\n"
        "  cam_height: 2048\n"
        "  allow_legacy_intrinsics: 1\n");
    TEST_CHECK(config_resolver::resolveCameraConfigFile(kCfgPath,
                                                        makeDefaults(), out,
                                                        err),
               "profile_only_loads");
    TEST_CHECK(out.cam_ip == "192.168.3.77" && out.cam_width == 2448 &&
                   out.cam_height == 2048,
               "profile_only_values");
    TEST_CHECK(out.allow_legacy_intrinsics, "profile_bool_set");
    TEST_CHECK(out.cam_port == 8090, "profile_only_port_default");

    // ---- 布尔开关：平铺显式 0 不被嵌套节 1 覆盖 ----
    writeFile(
        "%YAML:1.0\n---\n"
        "camera_profile: \"cam_mvs_a\"\n"
        "allow_legacy_intrinsics: 0\n"
        "cam_mvs_a:\n"
        "  allow_legacy_intrinsics: 1\n");
    TEST_CHECK(config_resolver::resolveCameraConfigFile(kCfgPath,
                                                        makeDefaults(), out,
                                                        err),
               "bool_flat_zero_loads");
    TEST_CHECK(!out.allow_legacy_intrinsics, "bool_flat_zero_wins");

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

    // ---- 非法/缺失文件 ----
    TEST_CHECK(!config_resolver::resolveCameraConfigFile(
                   "nonexistent_config.yaml", makeDefaults(), out, err),
               "missing_file_fails");

    std::remove(kCfgPath);
    return testutil::takeFailures();
}

}  // namespace suite
