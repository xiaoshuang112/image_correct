#include "calibration/eye_to_hand_calibration.hpp"
#include "calibration/calibration_utils.hpp"

#include <cmath>
#include <functional>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace fs = std::filesystem;

void require(bool condition, const std::string& message) {
    if (!condition)
        throw std::runtime_error(message);
}

struct TempDirectory {
    fs::path path;
    TempDirectory() {
        std::random_device random;
        for (int attempt = 0; attempt < 100; ++attempt) {
            const auto candidate = fs::temp_directory_path() /
                ("caitu_eye_to_hand_test_" + std::to_string(random()));
            if (fs::create_directory(candidate)) {
                path = candidate;
                return;
            }
        }
        throw std::runtime_error("cannot create unique test directory");
    }
    ~TempDirectory() {
        std::error_code ignored;
        fs::remove_all(path, ignored);
    }
};

void writeFile(const fs::path& path, const std::string& contents) {
    std::ofstream output(path, std::ios::binary);
    output.exceptions(std::ios::failbit | std::ios::badbit);
    output << contents;
    output.close();
}

void makeDiskFixture(const fs::path& directory, const std::string& row = "450,120,550,0,0,0") {
    std::ostringstream csv;
    for (int i = 0; i < 12; ++i) {
        csv << (i == 7 ? row : "450,120,550,0,0,0") << '\n';
        std::ostringstream filename;
        filename << std::setfill('0') << std::setw(3) << i << ".jpg";
        writeFile(directory / filename.str(), "");
    }
    writeFile(directory / "rob_pos.csv", csv.str());
}

cv::Mat pose(const cv::Vec3d& rotation, const cv::Vec3d& translation) {
    cv::Mat transform = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat r;
    cv::Rodrigues(rotation, r);
    r.copyTo(transform(cv::Rect(0, 0, 3, 3)));
    for (int axis = 0; axis < 3; ++axis)
        transform.at<double>(axis, 3) = translation[axis];
    return transform;
}

struct Fixture {
    // Translations are metres, Rodrigues rotation vectors are radians.
    const cv::Mat X = pose({0.38, -0.51, 0.24}, {0.72, -0.34, 0.91});
    const cv::Mat Z = pose({-0.22, 0.31, 0.17}, {0.08, -0.035, 0.16});
    std::vector<cv::Mat> robot;
    std::vector<cv::Mat> camera;

    explicit Fixture(double rotation_scale = 1.0) {
        for (int i = 0; i < 32; ++i) {
            const double t = static_cast<double>(i);
            const cv::Mat G = pose(
                {rotation_scale * 0.75 * std::sin(0.71 * t),
                 rotation_scale * 0.65 * std::cos(1.13 * t),
                 rotation_scale * 0.8 * std::sin(0.43 * t + 0.2)},
                {0.45 + 0.22 * std::sin(0.53 * t),
                 0.12 + 0.25 * std::cos(0.83 * t),
                 0.55 + 0.18 * std::sin(1.31 * t)});
            robot.push_back(G);
            // G=T_base_flange, C=T_camera_board: X*C = G*Z.
            camera.push_back(X.inv() * G * Z);
        }
    }
};

void checkAccurate(const EyeToHandResult& result, const Fixture& fixture,
                   double translation_tolerance = 1e-5,
                   double rotation_tolerance = 1e-5,
                   double consistency_tolerance = 1e-5) {
    require(result.success, "success flag missing: " + result.message);
    const cv::Mat& actual = result.T_base_camera;
    const cv::Mat expected = fixture.X.inv();
    require(actual.rows == 4 && actual.cols == 4 && actual.type() == CV_64F,
            "T_base_camera must be a 4x4 double transform");
    require(cv::checkRange(actual), "non-finite output transform");
    const cv::Rect rotation(0, 0, 3, 3);
    cv::Mat rotation_error;
    cv::Rodrigues(actual(rotation) * expected(rotation).t(), rotation_error);
    const double radians = cv::norm(rotation_error);
    const double metres = cv::norm(actual(cv::Rect(3, 0, 1, 3)) -
                                  expected(cv::Rect(3, 0, 1, 3)));
    // Exact synthetic inliers should recover well within 10 um / 0.0006 deg.
    require(metres < translation_tolerance,
            "translation error (m): " + std::to_string(metres));
    require(radians < rotation_tolerance,
            "rotation error (rad): " + std::to_string(radians));
    require(cv::norm(actual.row(3) - expected.row(3)) < 1e-12,
            "invalid homogeneous output row");
    require(cv::norm(actual - fixture.X) > 1.0,
            "returned internal solver direction instead of production convention");
    require(std::isfinite(result.reproj_error) &&
                result.reproj_error < consistency_tolerance,
            "consistency error (m): " + std::to_string(result.reproj_error));
    std::cout << "  translation=" << metres << " m, rotation=" << radians
              << " rad, consistency=" << result.reproj_error << " m\n";
}

void checkRejected(const std::vector<cv::Mat>& robot,
                   const std::vector<cv::Mat>& camera,
                   const EyeToHandResult& prior_success) {
    EyeToHandResult result = prior_success;
    result.T_base_camera = prior_success.T_base_camera.clone();
    require(!solveEyeToHandPoses(robot, camera, result),
            "invalid input unexpectedly succeeded");
    require(!result.success, "failure retained prior success flag");
    require(result.T_base_camera.empty(), "failure retained prior transform");
    require(result.valid_frames == 0, "failure retained prior valid frame count");
    require(result.reproj_error == 0.0, "failure retained prior consistency error");
    require(!result.message.empty(), "failure lacks diagnostic message");
}

} // namespace

int main() {
    cv::setNumThreads(1);
    const Fixture fixture;
    EyeToHandResult baseline;
    int failures = 0;
    auto run = [&](const std::string& name, const std::function<void()>& test) {
        try {
            test();
            std::cout << "[PASS] " << name << '\n';
        } catch (const std::exception& error) {
            ++failures;
            std::cerr << "[FAIL] " << name << ": " << error.what() << '\n';
        }
    };

    run("exact diverse poses recover production inverse convention", [&] {
        require(solveEyeToHandPoses(fixture.robot, fixture.camera, baseline),
                "solve failed: " + baseline.message);
        checkAccurate(baseline, fixture);
        require(baseline.valid_frames == static_cast<int>(fixture.robot.size()),
                "discarded exact diverse inliers");
    });
    // Failure reset checks must start from an actual successful solve.
    if (!baseline.success) {
        std::cerr << "Cannot check stale-success reset: baseline solve failed\n";
        return 1;
    }

    run("publication gate accepts reference-consistent result", [&] {
        TempDirectory temporary;
        const auto reference = temporary.path / "reference.yaml";
        cv::FileStorage out(reference.string(), cv::FileStorage::WRITE);
        out << "T_base_camera" << baseline.T_base_camera;
        out.release();
        PoseGenerationConfig config;
        config.minimum_publish_frames = 12;
        const EyeToHandPublicationCheck check = checkEyeToHandPublication(
            baseline, reference.string(), config);
        require(check.passed && check.reference_loaded,
                "reference-consistent result was blocked: " + check.message);
        require(check.translation_drift_mm < 1e-9 && check.rotation_drift_deg < 1e-9,
                "identical reference reported drift");
    });
    run("publication gate blocks excessive reference drift", [&] {
        TempDirectory temporary;
        const auto reference = temporary.path / "reference.yaml";
        cv::FileStorage out(reference.string(), cv::FileStorage::WRITE);
        out << "T_base_camera" << baseline.T_base_camera;
        out.release();
        PoseGenerationConfig config;
        config.minimum_publish_frames = 12;
        EyeToHandResult shifted = baseline;
        shifted.T_base_camera = baseline.T_base_camera.clone();
        shifted.T_base_camera.at<double>(0, 3) += 0.021;
        const EyeToHandPublicationCheck check = checkEyeToHandPublication(
            shifted, reference.string(), config);
        require(!check.passed && check.translation_drift_mm > 20.0,
                "21 mm reference drift was not blocked");
    });
    run("blocked publication preserves existing production bytes", [&] {
        TempDirectory temporary;
        const auto reference = temporary.path / "reference.yaml";
        const auto production = temporary.path / "production.yaml";
        cv::FileStorage out(reference.string(), cv::FileStorage::WRITE);
        out << "T_base_camera" << baseline.T_base_camera;
        out.release();
        writeFile(production, "known-good-production\n");
        PoseGenerationConfig config;
        config.minimum_publish_frames = 12;
        EyeToHandResult shifted = baseline;
        shifted.T_base_camera = baseline.T_base_camera.clone();
        shifted.T_base_camera.at<double>(1, 3) += 0.025;
        EyeToHandPublicationCheck check;
        std::string error;
        require(!publishEyeToHandResult(
                    shifted, reference.string(), production.string(), config,
                    check, error),
                "excessive drift unexpectedly published");
        std::ifstream input(production, std::ios::binary);
        const std::string bytes((std::istreambuf_iterator<char>(input)),
                                std::istreambuf_iterator<char>());
        require(bytes == "known-good-production\n",
                "blocked publication modified production result");
    });
    run("publication gate requires protected reference", [&] {
        TempDirectory temporary;
        PoseGenerationConfig config;
        config.minimum_publish_frames = 12;
        const EyeToHandPublicationCheck check = checkEyeToHandPublication(
            baseline, (temporary.path / "missing.yaml").string(), config);
        require(!check.passed && check.message.find("缺少受保护参考结果") != std::string::npos,
                "missing protected reference was not blocked");
    });
    run("publication gate blocks all September field drifts from June reference", [&] {
        const cv::Mat june = (cv::Mat_<double>(4, 4) <<
            0.99986113048138547, -0.011420704821709197, -0.012136196021096583, 0.2294065352798664,
            -0.010909309238003425, -0.99908309208772517, 0.041400025077042046, 1.0394282948566971,
            -0.012597885712956178, -0.041261878360121775, -0.99906894190028639, 0.97401898922113117,
            0, 0, 0, 1);
        const std::vector<cv::Mat> september{
            (cv::Mat_<double>(4, 4) <<
                0.99976425974715022, 0.019259799900111282, 0.010024222665158096, 0.18103239407758903,
                0.019334652929527994, -0.99978549991061816, -0.0074246457538376066, 1.1650557416967207,
                0.0098790752769523128, 0.0076167103320894164, -0.99992219177063446, 0.90934362945690173,
                0, 0, 0, 1),
            (cv::Mat_<double>(4, 4) <<
                0.99977897855553355, 0.019200422202143262, 0.0085637506802915143, 0.18743505085038614,
                0.019232296840059504, -0.99980836024746933, -0.0036553436941572123, 1.164080670944619,
                0.0084919253830086482, 0.0038192363799615351, -0.99995664937874351, 0.90660297943195156,
                0, 0, 0, 1),
            (cv::Mat_<double>(4, 4) <<
                0.99979890150703465, 0.018295808933456285, 0.008210963451210444, 0.19077413961114018,
                0.018340727242780488, -0.999817055227048, -0.0054289779259130665, 1.1639668271209793,
                0.0081101337555296877, 0.0055784812076929147, -0.99995155196533547, 0.8974886663655115,
                0, 0, 0, 1)};
        TempDirectory temporary;
        const auto reference = temporary.path / "reference.yaml";
        cv::FileStorage out(reference.string(), cv::FileStorage::WRITE);
        out << "T_base_camera" << june;
        out.release();
        PoseGenerationConfig config;
        for (const cv::Mat& transform : september) {
            EyeToHandResult candidate;
            candidate.success = true;
            candidate.T_base_camera = transform;
            candidate.valid_frames = 69;
            candidate.reproj_error = 0.0025;
            candidate.quality.translation_p95_mm = 4.5;
            candidate.quality.rotation_rms_deg = 0.35;
            candidate.quality.consensus_available = true;
            candidate.quality.consensus_passed = true;
            const EyeToHandPublicationCheck check = checkEyeToHandPublication(
                candidate, reference.string(), config);
            require(!check.passed && check.reference_loaded &&
                        check.translation_drift_mm > 140.0,
                    "September field drift was not blocked");
        }
    });
    run("publication gate does not trade rotation quality for lower translation RMS", [&] {
        TempDirectory temporary;
        const auto reference = temporary.path / "reference.yaml";
        cv::FileStorage out(reference.string(), cv::FileStorage::WRITE);
        out << "T_base_camera" << baseline.T_base_camera;
        out.release();
        PoseGenerationConfig config;
        config.minimum_publish_frames = 12;
        EyeToHandResult candidate = baseline;
        candidate.reproj_error = 0.001079184;
        candidate.quality.translation_p95_mm = 1.81872;
        candidate.quality.rotation_rms_deg = 0.574243;
        const EyeToHandPublicationCheck check = checkEyeToHandPublication(
            candidate, reference.string(), config);
        require(!check.passed && check.message.find("旋转 RMS") != std::string::npos,
                "low-translation/high-rotation candidate was not blocked");
    });

    run("minimum 12 diverse poses succeed", [&] {
        EyeToHandResult result;
        const bool ok = solveEyeToHandPoses(
            {fixture.robot.begin(), fixture.robot.begin() + 12},
            {fixture.camera.begin(), fixture.camera.begin() + 12}, result);
        require(ok, "minimum-frame solve failed: " + result.message);
        checkAccurate(result, fixture);
        require(result.valid_frames == 12, "incorrect minimum-frame inlier count");
    });

    run("production small-angle poses recover exact X", [&] {
        // Each Rodrigues component is bounded by 10 degrees.
        const Fixture small_angles((10.0 * CV_PI / 180.0) / 0.8);
        EyeToHandResult result;
        const bool ok = solveEyeToHandPoses(small_angles.robot, small_angles.camera, result);
        require(ok, "small-angle solve failed: " + result.message);
        checkAccurate(result, small_angles);
        require(result.valid_frames == 32, "discarded exact small-angle inliers");
    });
    run("production small-angle poses with small measurement noise", [&] {
        Fixture noisy((10.0 * CV_PI / 180.0) / 0.8);
        std::mt19937 random(0xE1E70u);
        std::normal_distribution<double> normal(0.0, 1.0);
        auto perturb = [&](const cv::Mat& transform, double metres, double radians) {
            const cv::Vec3d rotation(normal(random) * radians,
                                     normal(random) * radians,
                                     normal(random) * radians);
            const cv::Vec3d translation(normal(random) * metres,
                                        normal(random) * metres,
                                        normal(random) * metres);
            return cv::Mat(transform * pose(rotation, translation));
        };
        for (size_t i = 0; i < noisy.robot.size(); ++i) {
            // Independent local pose perturbations: robot 20 um / 50 urad,
            // camera 50 um / 100 urad, per-component standard deviations.
            noisy.robot[i] = perturb(noisy.robot[i], 2e-5, 5e-5);
            noisy.camera[i] = perturb(noisy.camera[i], 5e-5, 1e-4);
        }
        EyeToHandResult result;
        const bool ok = solveEyeToHandPoses(noisy.robot, noisy.camera, result);
        require(ok, "noisy small-angle solve failed: " + result.message);
        // Fixed measurement-noise budget: 1 mm, 0.1 degree, 0.5 mm residual.
        checkAccurate(result, noisy, 1e-3, 0.1 * CV_PI / 180.0, 5e-4);
        require(result.valid_frames == 32, "discarded small-noise inliers");
    });

    run("empty input clears prior success", [&] {
        checkRejected({}, {}, baseline);
    });
    for (int count : {1, 2, 11}) {
        run("too few poses: " + std::to_string(count), [&] {
            checkRejected({fixture.robot.begin(), fixture.robot.begin() + count},
                          {fixture.camera.begin(), fixture.camera.begin() + count},
                          baseline);
        });
    }
    for (bool robot_side : {false, true}) {
        const std::string side = robot_side ? "robot" : "camera";
        run(side + " count mismatch clears prior success", [&] {
            auto robot = fixture.robot;
            auto camera = fixture.camera;
            (robot_side ? robot : camera).pop_back();
            checkRejected(robot, camera, baseline);
        });
        const std::vector<std::pair<std::string, cv::Mat>> invalid{
            {"empty matrix", cv::Mat()},
            {"wrong shape", cv::Mat::eye(3, 3, CV_64F)},
            {"wrong scalar type", cv::Mat::eye(4, 4, CV_32F)},
            {"multiple channels", cv::Mat::zeros(4, 4, CV_64FC2)}};
        for (const auto& entry : invalid) {
            run(side + " " + entry.first, [&] {
                auto robot = fixture.robot;
                auto camera = fixture.camera;
                (robot_side ? robot : camera)[7] = entry.second;
                checkRejected(robot, camera, baseline);
            });
        }
        for (const std::string defect : {"scaled rotation", "reflection", "bottom row"}) {
            run(side + " " + defect, [&] {
                auto robot = fixture.robot;
                auto camera = fixture.camera;
                cv::Mat& bad = (robot_side ? robot : camera)[7];
                // Allocate independently; MatExpr assignment could reuse the
                // shallow-copied fixture's storage and corrupt later cases.
                bad = pose({0, 0, 0}, {0, 0, 0});
                if (defect == "bottom row")
                    bad.at<double>(3, 0) = 0.1;
                else
                    bad.at<double>(0, 0) = defect == "reflection" ? -1.0 : 1.1;
                checkRejected(robot, camera, baseline);
            });
        }
        for (double value : {std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::infinity()}) {
            run(side + (std::isnan(value) ? " NaN" : " infinity"), [&] {
                auto robot = fixture.robot;
                auto camera = fixture.camera;
                cv::Mat& bad = (robot_side ? robot : camera)[7];
                bad = bad.clone();
                bad.at<double>(0, 3) = value;
                checkRejected(robot, camera, baseline);
            });
        }
    }
    run("identical motion is unobservable and clears prior success", [&] {
        checkRejected(std::vector<cv::Mat>(32, fixture.robot.front()),
                      std::vector<cv::Mat>(32, fixture.camera.front()), baseline);
    });
    for (bool single_axis : {false, true}) {
        run(single_axis ? "single-axis rotation is unobservable" :
                          "translation-only motion is unobservable", [&] {
            std::vector<cv::Mat> robot, camera;
            for (int i = 0; i < 32; ++i) {
                const cv::Mat G = pose({0, 0, single_axis ? 0.04 * i : 0.0},
                                      {0.3 + 0.01 * i, 0.1 * std::sin(i), 0.5});
                robot.push_back(G);
                camera.push_back(fixture.X.inv() * G * fixture.Z);
            }
            checkRejected(robot, camera, baseline);
        });
    }
    run("one gross camera translation outlier is rejected", [&] {
        auto camera = fixture.camera;
        camera[11] = camera[11].clone();
        camera[11].at<double>(0, 3) += 0.35;
        camera[11].at<double>(1, 3) -= 0.25;
        camera[11].at<double>(2, 3) += 0.30;
        EyeToHandResult result;
        require(solveEyeToHandPoses(fixture.robot, camera, result),
                "outlier solve failed: " + result.message);
        checkAccurate(result, fixture);
        require(result.valid_frames == 31, "expected exactly 31 retained inliers");
    });

    auto rejectDisk = [&](const fs::path& directory, const std::string& diagnostic) {
        EyeToHandResult result = baseline;
        result.T_base_camera = baseline.T_base_camera.clone();
        require(!calibrateEyeToHandFromDisk(directory.string(), result),
                "invalid disk input unexpectedly succeeded");
        require(!result.success && result.T_base_camera.empty() &&
                    result.valid_frames == 0 && result.reproj_error == 0.0,
                "disk failure retained prior successful result");
        // Zero-byte images cannot calibrate: require the specific earlier gate,
        // so falling through to intrinsic calibration cannot produce a false pass.
        require(result.message.find(diagnostic) != std::string::npos,
                "wrong rejection stage: " + result.message);
        require(result.camera_matrix.empty() && result.dist_coeffs.empty(),
                "preflight rejection should not produce intrinsics");
    };
    for (const std::string row : {"450,120,550,0,0", "450,120,550,0,0,0,7",
                                  "450,120,NaN,0,0,0", "", "not,a,robot,pose"}) {
        run("disk malformed CSV: " + row, [&] {
            TempDirectory temporary;
            makeDiskFixture(temporary.path, row);
            rejectDisk(temporary.path, "CSV");
        });
    }
    run("disk missing image rejects before intrinsic calibration", [&] {
        TempDirectory temporary;
        makeDiskFixture(temporary.path);
        fs::remove(temporary.path / "005.jpg");
        rejectDisk(temporary.path, "缺少匹配图像");
    });
    for (const std::string extra : {"012.jpg", "5.jpg", "005.png"}) {
        run("disk extra or nonstandard image: " + extra, [&] {
            TempDirectory temporary;
            makeDiskFixture(temporary.path);
            writeFile(temporary.path / extra, "");
            rejectDisk(temporary.path, "多余或非标准编号图像");
        });
    }
    run("disk incomplete collection rejects before intrinsic calibration", [&] {
        TempDirectory temporary;
        makeDiskFixture(temporary.path);
        writeFile(temporary.path / ".collection_incomplete", "incomplete\n");
        rejectDisk(temporary.path, "采集未完整写入");
    });
    const cv::Mat intrinsics = (cv::Mat_<double>(3, 3) <<
        1000, 0, 640, 0, 1000, 480, 0, 0, 1);
    const cv::Mat distortion = cv::Mat::zeros(1, 5, CV_64F);
    auto save = [&](const fs::path& destination) {
        return saveCalibrationResult(destination.string(), "T_base_camera",
                                     baseline.T_base_camera, intrinsics, distortion,
                                     baseline.reproj_error * 1000.0, baseline.valid_frames);
    };
    run("save fails for unwritable destination path", [&] {
        TempDirectory temporary;
        // A regular file as parent fails even when tests run as root.
        const auto blocker = temporary.path / "not_a_directory";
        writeFile(blocker, "keep");
        require(!save(blocker / "result.yaml"), "save accepted impossible parent path");
    });
    run("failed save preserves previous destination bytes", [&] {
        TempDirectory temporary;
        const auto destination = temporary.path / "result.yaml";
        require(save(destination), "initial result save failed");
        auto read = [&] {
            std::ifstream input(destination, std::ios::binary);
            require(input.good(), "cannot read saved result");
            return std::string(std::istreambuf_iterator<char>(input),
                               std::istreambuf_iterator<char>());
        };
        const std::string original = read();
        require(!original.empty(), "initial result file empty");
        // Block staging without relying on OS user permission semantics.
        const fs::path staging(destination.string() + ".tmp.yaml");
        fs::create_directory(staging);
        writeFile(staging / "blocker", "keep");
        require(!save(destination), "save unexpectedly accepted blocked staging path");
        require(read() == original, "failed save changed previous destination");
    });
#ifdef __linux__
    run("disk write failure preserves published YAML", [&] {
        TempDirectory temporary;
        const auto destination = temporary.path / "result.yaml";
        require(save(destination), "initial result save failed");
        const fs::path staging(destination.string() + ".tmp.yaml");
        // /dev/full deterministically fails write/flush with ENOSPC, including as root.
        fs::create_symlink("/dev/full", staging);
        require(!save(destination), "disk-full write unexpectedly succeeded");
        cv::FileStorage stored(destination.string(), cv::FileStorage::READ);
        cv::Mat transform;
        stored["T_base_camera"] >> transform;
        require(cv::norm(transform - baseline.T_base_camera) < 1e-12,
                "disk-full write damaged the previously published transform");
        require(!fs::exists(staging), "failed staging symlink was not cleaned up");
    });
#endif
    run("unreadable images cannot return a successful calibration", [&] {
        TempDirectory temporary;
        makeDiskFixture(temporary.path);
        EyeToHandResult result = baseline;
        result.T_base_camera = baseline.T_base_camera.clone();
        require(!calibrateEyeToHandFromDisk(temporary.path.string(), result),
                "zero-byte images unexpectedly calibrated");
        require(!result.success && result.T_base_camera.empty(),
                "unreadable image failure kept stale success");
    });
    run("disk replay rejects missing fixed intrinsics", [&] {
        TempDirectory temporary;
        makeDiskFixture(temporary.path);
        EyeToHandResult result = baseline;
        result.T_base_camera = baseline.T_base_camera.clone();
        const auto missing = temporary.path / "missing_camera_params.yaml";
        require(!calibrateEyeToHandFromDisk(temporary.path.string(), result,
                                             missing.string()),
                "missing fixed intrinsics unexpectedly accepted");
        require(!result.success && result.T_base_camera.empty(),
                "fixed-intrinsics failure kept stale success");
        require(result.message.find("无法打开相机内参") != std::string::npos,
                "missing fixed intrinsics reported wrong failure");
    });
    std::cout << failures << " failed test(s)\n";
    return failures == 0 ? 0 : 1;
}
