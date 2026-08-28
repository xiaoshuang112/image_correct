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
               << "maximum_joint_step_degrees: 60.0\n";
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
    }
    std::remove(path);

    return testutil::takeFailures();
}

} // namespace suite
