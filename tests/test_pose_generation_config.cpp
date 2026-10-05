#include "test_check.hpp"

#include "calibration/eye_to_hand_calibration.hpp"

#include <cstdio>
#include <fstream>
#include <string>

namespace suite {

int test_pose_generation_config()
{
    {
        PoseGenerationConfig config;
        std::string error;
        TEST_CHECK(!tryLoadPoseGenerationConfig(
                       "caitu_missing_pose_generation_config.yaml",
                       config, error) && !error.empty(),
                   "pose config rejects missing file");
        // 未配置时必须保持历史行为：参考比对默认启用
        TEST_CHECK(config.reference_check_enabled && config.require_reference_result,
                   "pose config defaults to enabling the reference check");
    }

    const char* path = "caitu_test_pose_generation_config.yaml";
    {
        std::ofstream output(path);
        output << "%YAML:1.0\n---\n"
               << "offset_x: [1, 0, -1]\n"
               << "offset_y: [2, 0, -2]\n"
               << "offset_z: [3, 0, -3]\n"
               << "offset_rx: [4, 0, -4]\n"
               << "offset_ry: [5, 0, -5]\n"
               << "offset_rz: [6, 0, -6]\n"
               << "min_corners: 16\n"
               << "min_valid_frames: 20\n"
               << "maximum_candidates: 90\n"
               << "maximum_joint_step_degrees: 60.0\n"
               << "maximum_capture_translation_drift_mm: 0.2\n"
               << "maximum_capture_rotation_drift_degrees: 0.1\n"
               << "reference_result_file: reference.yaml\n"
               << "require_reference_result: 0\n"
               << "reference_check_enabled: 0\n"
               << "minimum_publish_frames: 30\n"
               << "maximum_publish_translation_rms_mm: 2.5\n"
               << "maximum_publish_translation_p95_mm: 4.5\n"
               << "maximum_publish_rotation_rms_degrees: 0.4\n"
               << "maximum_publish_translation_drift_mm: 15.0\n"
               << "maximum_publish_rotation_drift_degrees: 1.5\n";
    }
    {
        PoseGenerationConfig config;
        std::string error;
        const bool ok = tryLoadPoseGenerationConfig(path, config, error);
        TEST_CHECK(ok, "pose config loads valid file");
        TEST_CHECK(config.offset_z.size() == 3 && config.offset_z[0] == 3.0,
                   "pose config loads offsets");
        TEST_CHECK(config.min_corners == 16 &&
                   config.maximum_candidates == 90 &&
                   config.maximum_joint_step_degrees == 60.0,
                   "pose config loads planning gates");
        TEST_CHECK(config.source_file == path,
                   "pose config records source file");
        TEST_CHECK(config.maximum_capture_translation_drift_mm == 0.2 &&
                   config.maximum_capture_rotation_drift_degrees == 0.1 &&
                   config.reference_result_file == "reference.yaml" &&
                   !config.require_reference_result &&
                   !config.reference_check_enabled &&
                   config.minimum_publish_frames == 30 &&
                   config.maximum_publish_translation_drift_mm == 15.0,
                   "pose config loads capture and publication gates");
    }
    std::remove(path);

    return testutil::takeFailures();
}

} // namespace suite
