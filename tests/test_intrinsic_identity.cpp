// 内参安全门禁无硬件测试：形状校验、身份字段加载、身份绑定与旧版兼容开关。
#include "test_check.hpp"

#include "calibration/intrinsic_identity.hpp"

#include <opencv2/core.hpp>

#include <cmath>
#include <cstdio>
#include <string>

using namespace intrinsic_gate;

namespace {

const char* kYamlPath = "caitu_test_camera_params.yaml";
const char* kLegacyYamlPath = "caitu_test_camera_params_legacy.yaml";

cv::Mat makeK() {
    return (cv::Mat_<double>(3, 3) << 2400, 0, 1224, 0, 2400, 1024, 0, 0, 1);
}
cv::Mat makeD() {
    return (cv::Mat_<double>(1, 5) << -0.1, 0.05, 0, 0, 0);
}

}  // namespace

namespace suite {

int test_intrinsic_identity() {
    std::cout << "---- intrinsic_identity ----\n";
    std::string reason;

    // ---- 形状校验 ----
    TEST_CHECK(validateIntrinsicsShape(makeK(), makeD(), 2448, 2048, reason),
               "shape_valid");
    {
        cv::Mat badK = makeK();
        badK.at<double>(0, 0) = -1.0;
        TEST_CHECK(!validateIntrinsicsShape(badK, makeD(), 2448, 2048, reason),
                   "shape_negative_focal");
    }
    {
        cv::Mat nanK = makeK();
        nanK.at<double>(1, 1) = std::nan("");
        TEST_CHECK(!validateIntrinsicsShape(nanK, makeD(), 2448, 2048, reason),
                   "shape_nan_focal");
    }
    {
        cv::Mat badD(1, 7, CV_64F, 0.0);
        TEST_CHECK(!validateIntrinsicsShape(makeK(), badD, 2448, 2048, reason),
                   "shape_bad_dist_length");
    }
    {
        cv::Mat wrongShape(4, 4, CV_64F, 0.0);
        TEST_CHECK(!validateIntrinsicsShape(wrongShape, makeD(), 2448, 2048,
                                            reason),
                   "shape_not_3x3");
    }
    {
        // 焦距相对分辨率离谱（fx=2400 对 640 宽 = 3.75 倍，仍合理；
        // fx=240000 则不合理）
        cv::Mat hugeK = makeK();
        hugeK.at<double>(0, 0) = 240000.0;
        TEST_CHECK(!validateIntrinsicsShape(hugeK, makeD(), 2448, 2048,
                                            reason),
                   "shape_focal_resolution_mismatch");
    }
    {
        // 主点离谱偏出
        cv::Mat offK = makeK();
        offK.at<double>(0, 2) = 50000.0;
        TEST_CHECK(!validateIntrinsicsShape(offK, makeD(), 2448, 2048, reason),
                   "shape_principal_point_outlier");
    }
    // 4 元素畸变（鱼眼/简化模型）也应接受
    TEST_CHECK(validateIntrinsicsShape(
                   makeK(), cv::Mat(1, 4, CV_64F, 0.0), 2448, 2048, reason),
               "shape_dist_len4_ok");

    // ---- 带身份 YAML 加载 ----
    {
        cv::FileStorage fs(kYamlPath, cv::FileStorage::WRITE);
        fs << "camera_matrix" << makeK()
           << "distortion_coefficients" << makeD()
           << "camera_serial" << "DA1234567"
           << "image_width" << 2448 << "image_height" << 2048
           << "pixel_format" << "Mono8";
        fs.release();
    }
    cv::Mat K2, D2;
    IntrinsicIdentity id;
    std::string err;
    TEST_CHECK(loadCameraParamsWithIdentity(kYamlPath, K2, D2, id, err),
               "load_with_identity");
    TEST_CHECK(id.serial == "DA1234567" && id.width == 2448 &&
                   id.height == 2048 && id.pixel_format == "Mono8",
               "identity_fields_loaded");
    TEST_CHECK(id.hasAny() && id.hasSerial() && id.hasResolution(),
               "identity_flags");

    // ---- 身份绑定 ----
    TEST_CHECK(checkIntrinsicBinding(id, "DA1234567", 2448, 2048, false,
                                     reason),
               "bind_exact_match");
    TEST_CHECK(!checkIntrinsicBinding(id, "DA9999999", 2448, 2048, false,
                                      reason),
               "bind_serial_mismatch");
    TEST_CHECK(!checkIntrinsicBinding(id, "DA1234567", 1280, 800, false,
                                      reason),
               "bind_resolution_mismatch");
    TEST_CHECK(!checkIntrinsicBinding(id, "", 2448, 2048, false, reason),
               "bind_empty_device_serial");

    // ---- 旧版仅 K/D（无身份字段）----
    {
        cv::FileStorage fs(kLegacyYamlPath, cv::FileStorage::WRITE);
        fs << "camera_matrix" << makeK()
           << "distortion_coefficients" << makeD();
        fs.release();
    }
    IntrinsicIdentity legacyId;
    TEST_CHECK(loadCameraParamsWithIdentity(kLegacyYamlPath, K2, D2, legacyId,
                                            err),
               "load_legacy");
    TEST_CHECK(!legacyId.hasAny(), "legacy_no_identity");
    TEST_CHECK(!checkIntrinsicBinding(legacyId, "DA1234567", 2448, 2048,
                                      false, reason),
               "legacy_rejected_by_default");
    TEST_CHECK(checkIntrinsicBinding(legacyId, "DA1234567", 2448, 2048, true,
                                     reason),
               "legacy_allowed_with_switch");
    TEST_CHECK(reason.find("警告") != std::string::npos,
               "legacy_warning_returned");

    // ---- 缺失文件 ----
    TEST_CHECK(!loadCameraParamsWithIdentity("nonexistent_file.yaml", K2, D2,
                                             id, err),
               "load_missing_file_fails");

    std::remove(kYamlPath);
    std::remove(kLegacyYamlPath);
    return testutil::takeFailures();
}

}  // namespace suite
