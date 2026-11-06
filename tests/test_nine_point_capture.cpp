#include "hand_eye_gui/NinePointCapture.hpp"
#include "utils/calib_transform.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
template<class F> std::string rejects(F action) {
    try { action(); } catch (const std::exception& e) { return e.what(); }
    throw std::runtime_error("unsafe operation accepted");
}
using collection::FlangePose;
struct Robot {
    FlangePose actual{100, 200, 300, 180, 0, 0};
    PointControlPara command{};
    double expectedSpeed = 10;
    CRresult modeCode = sucess, poseCode = sucess, lineCode = sucess, stopCode = sucess;
    bool issued = false, neverReach = false, badInverse = false, badJoint = false, protective = false;
    bool lostPoseDuringMove = false, invalidPoseDuringMove = false, faultDuringMove = false;
    CRresult inverseCode = sucess;
    int sends = 0, stops = 0, polls = 0, inverseReads = 0, cancelOnPoll = 0;
    std::atomic<bool> cancelled{false};
    void reset() {
        actual = {100, 200, 300, 180, 0, 0}; command = {};
        expectedSpeed = 10;
        modeCode = poseCode = lineCode = stopCode = sucess;
        issued = neverReach = badInverse = badJoint = protective = false;
        lostPoseDuringMove = invalidPoseDuringMove = faultDuringMove = false; inverseCode = sucess;
        sends = stops = polls = inverseReads = cancelOnPoll = 0; cancelled = false;
    }
} robot;
}

extern "C" CRresult cr_get_robotMode(RobotHandle, RobotModes* mode) {
    *mode = robot.protective || (robot.issued && robot.faultDuringMove) ? ProtectiveStop
        : robot.issued && robot.polls < 2 ? SDK_Moving : Enable;
    return robot.modeCode;
}
extern "C" CRresult cr_get_robotMoveStatus(RobotHandle, BOOL* moving) {
    if (!robot.issued) { *moving = FALSE; return sucess; }
    ++robot.polls;
    if (robot.cancelOnPoll && robot.polls >= robot.cancelOnPoll) robot.cancelled = true;
    *moving = robot.polls < 2;
    if (!*moving && !robot.neverReach)
        std::copy_n(robot.command.pose, ROB_AXIS_NUM, robot.actual.begin());
    return sucess;
}
extern "C" CRresult cr_get_AssignTCP_Pose(RobotHandle, double* tool, double* pose) {
    require(std::all_of(tool, tool + ROB_AXIS_NUM, [](double v) { return v == 0; }), "inherited active TCP");
    std::copy(robot.actual.begin(), robot.actual.end(), pose);
    if (robot.issued && robot.invalidPoseDuringMove) pose[0] = std::numeric_limits<double>::quiet_NaN();
    if (robot.issued && robot.lostPoseDuringMove) return static_cast<CRresult>(1);
    return robot.poseCode;
}
extern "C" CRresult cr_get_jointActualPos(RobotHandle, double* joints) {
    std::fill_n(joints, ROB_AXIS_NUM, 0);
    if (robot.badJoint) joints[0] = std::numeric_limits<double>::quiet_NaN();
    return sucess;
}
extern "C" CRresult cr_kineInverse(RobotHandle, double*, double*, double* joints) {
    ++robot.inverseReads;
    std::fill_n(joints, ROB_AXIS_NUM, robot.badInverse ? 90 : 0);
    return robot.inverseCode;
}
extern "C" CRresult cr_move_line(RobotHandle, PointControlPara command, BOOL block) {
    require(!block, "motion blocks cancellation");
    require(command.coordinateType == baseCoordinate && command.tcpID < 0, "wrong coordinate frame or tool");
    require(command.pointTransType == pointTransStop && command.pointTransRadius == 0, "motion blends past target");
    for (int axis = 0; axis < ROB_AXIS_NUM; ++axis) {
        require(command.tcpOffset[axis] == 0 && command.coordinatePose[axis] == 0, "tool/frame offset changed");
        require(command.speed[axis] == robot.expectedSpeed && command.acc[axis] == 20, "motion limits changed");
        require(std::isfinite(command.jointpos[axis]), "invalid joint reference sent");
    }
    robot.command = command; robot.issued = true; ++robot.sends;
    return robot.lineCode;
}
extern "C" CRresult cr_stop(RobotHandle) { ++robot.stops; return robot.stopCode; }

int main() {
    try {
        const FlangePose origin{100, 200, 300, 180, 0, 0};
        const auto targets = collection::ninePointTargets(origin, 10, 12.5);
        require(targets[4] == origin, "center is not the baseline");
        const std::array<double, 9> expectedAngles{0, 15, -15, -20, 0, 20, 15, -15, 0};
        for (int point = 0; point < 9; ++point) {
            const auto& p = targets[point];
            require(p[0] == origin[0] + (point % 3 - 1) * 10 &&
                    p[1] == origin[1] + (1 - point / 3) * 12.5 && p[2] == origin[2],
                    "incorrect grid offsets or changed Z height");
            const double angle = expectedAngles[point] * CV_PI / 180;
            const cv::Mat expected = (cv::Mat_<double>(3,3) << std::cos(angle), std::sin(angle), 0,
                std::sin(angle), -std::cos(angle), 0, 0, 0, -1);
            const auto actual = calib_transform::axisAnglePoseToTransform(p.data());
            require(cv::norm(actual(cv::Rect(0,0,3,3)) - expected) < 1e-9,
                    "Euler Rz increment incorrect for downward-facing flange");
        }
        // Independent Euler construction checks arbitrary roll/pitch, yaw wrap,
        // and both gimbal-lock directions, without adding to the SDK's rvec.z.
        auto eulerRotation = [](double roll, double pitch, double yaw) -> cv::Mat {
            const double x = roll * CV_PI / 180, y = pitch * CV_PI / 180, z = yaw * CV_PI / 180;
            const double cx = std::cos(x), sx = std::sin(x), cy = std::cos(y), sy = std::sin(y),
                         cz = std::cos(z), sz = std::sin(z);
            return (cv::Mat_<double>(3,3) << cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx,
                sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx, -sy, cy*sx, cy*cx);
        };
        for (double pitch : {-90.0, -89.999, -32.0, 0.0, 47.0, 89.999, 90.0}) {
            cv::Mat transform = cv::Mat::eye(4,4,CV_64F);
            const cv::Mat rotation = eulerRotation(38, pitch, 179);
            rotation.copyTo(transform(cv::Rect(0,0,3,3)));
            transform.at<double>(0,3) = 0.123;
            transform.at<double>(1,3) = -0.456;
            transform.at<double>(2,3) = 0.789;
            FlangePose baseline{};
            calib_transform::transformToAxisAnglePose(transform, baseline.data());
            const auto grid = collection::ninePointTargets(baseline, 10, 10);
            for (int point = 0; point < 9; ++point) {
                const auto actual = calib_transform::axisAnglePoseToTransform(grid[point].data());
                const cv::Mat expected = eulerRotation(38, pitch, 179 + expectedAngles[point]);
                require(cv::norm(actual(cv::Rect(0,0,3,3)) - expected) < 1e-9,
                        "Euler Rx/Ry changed, wrong rotation order or cumulative yaw");
                require(grid[point][2] == baseline[2], "rotation changed translation height");
            }
        }
        for (double bad : {0.0, -10.0, 0.09, 50.01, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
            rejects([&] { collection::ninePointTargets(origin, bad, 10); });
            rejects([&] { collection::ninePointTargets(origin, 10, bad); });
        }
        const collection::NinePointAngles customAngles{1.25, 2.5, -3.75, -10, 0.1, 10, 7.5, -15, 0.01};
        const auto custom = collection::ninePointTargets(origin, 10, 12.5, customAngles);
        require(custom[4] != origin && custom[4][0] == origin[0] && custom[4][1] == origin[1],
                "center angle was ignored or moved the center position");
        for (int point = 0; point < 9; ++point) {
            const auto actual = calib_transform::axisAnglePoseToTransform(custom[point].data());
            require(cv::norm(actual(cv::Rect(0,0,3,3)) - eulerRotation(180, 0, customAngles[point])) < 1e-7,
                    "custom point used a default or accumulated angle");
            for (double invalid : {-20.01, 20.01, 0.001, -0.001,
                                   std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
                auto badAngles = customAngles; badAngles[point] = invalid;
                rejects([&] { collection::ninePointTargets(origin, 10, 10, badAngles); });
            }
        }
        collection::ninePointTargets(origin, 30, 40);
        rejects([&] { collection::ninePointTargets(origin, 40, 40); });
        rejects([&] { collection::ninePointTargets(origin, 0.1, 50); });
        auto invalid = origin; invalid[2] = std::numeric_limits<double>::quiet_NaN();
        rejects([&] { collection::ninePointTargets(invalid, 10, 10); });
        auto equivalent = origin; equivalent[3] = -180;
        require(collection::flangeAtTarget(equivalent, origin), "axis-angle wrap incorrectly rejected");
        equivalent = origin; equivalent[3] += 1;
        require(!collection::flangeAtTarget(equivalent, origin), "attitude mismatch accepted");
        equivalent = origin; equivalent[0] += 0.3;
        require(!collection::flangeAtTarget(equivalent, origin), "position mismatch accepted");

        std::atomic<bool> cancelled{false};
        std::vector<std::string> events;
        FlangePose current = origin;
        bool saving = false;
        auto move = [&](const FlangePose& p, int index) {
            require(!saving, "robot moved before file saving finished");
            current = p; events.push_back("move" + std::to_string(index));
        };
        auto capture = [&](int point) {
            require(current == targets[point], "capture ran at wrong position");
            saving = true; events.push_back("save" + std::to_string(point)); saving = false;
        };
        collection::runNinePointCapture(origin, targets, move, capture, cancelled);
        require(events == std::vector<std::string>{"move0", "save0", "move-1", "move1", "save1", "move-1", "move2", "save2",
                "move-1", "move3", "save3", "move-1", "save4", "move5", "save5", "move-1",
                "move6", "save6", "move-1", "move7", "save7", "move-1", "move8", "save8", "move-1"}, "route or save/move order incorrect");
        require(current == origin, "normal completion did not return to origin");
        int customCaptures = 0, customReturns = 0;
        collection::runNinePointCapture(origin, custom,
            [&](const FlangePose& pose, int index) {
                if (index == -1) {
                    require(pose == origin, "return used the rotated center instead of the original baseline");
                    ++customReturns;
                }
                current = pose;
            }, [&](int point) {
                require(current == custom[point], "custom capture ran at the wrong point or angle");
                ++customCaptures;
            }, cancelled);
        require(customCaptures == 9 && customReturns == 9 && current == origin,
                "custom center was skipped or the original baseline was not restored");
        rejects([&] { collection::runNinePointCapture(origin, custom, move, [&](int point) {
            if (point == 4) throw std::runtime_error("center save failed");
        }, cancelled); });
        require(current == custom[4], "moved after a rotated center save failure");
        for (int failed = 0; failed < 9; ++failed) {
            events.clear(); current = origin;
            rejects([&] { collection::runNinePointCapture(origin, targets, move, [&](int point) {
                capture(point);
                if (point == failed) throw std::runtime_error("save failed");
            }, cancelled); });
            require(events.back() == "save" + std::to_string(failed) && current == targets[failed],
                    "returned or advanced after an incomplete save");
        }
        events.clear();
        current = origin;
        rejects([&] { collection::runNinePointCapture(origin, targets, move, [&](int point) {
            capture(point); if (point == 2) throw std::runtime_error("disk full");
        }, cancelled); });
        require(events.back() == "save2" && current == targets[2], "moved after save failure");
        events.clear(); current = origin;
        rejects([&] { collection::runNinePointCapture(origin, targets, move, [&](int point) {
            capture(point); cancelled = true;
        }, cancelled); });
        require(events == std::vector<std::string>{"move0", "save0"}, "cancel still issued motion");
        events.clear(); cancelled = false;
        rejects([&] { collection::runNinePointCapture(origin, targets, [&](const FlangePose&, int) {
            events.push_back("failed_move"); throw std::runtime_error("motion failed");
        }, capture, cancelled); });
        require(events == std::vector<std::string>{"failed_move"}, "capture or return followed failed move");

        collection::checkNinePointReachability(1, targets);
        require(robot.inverseReads == 9 && robot.sends == 0, "preflight moved robot or missed a target");
        robot.badInverse = true;
        rejects([&] { collection::checkNinePointReachability(1, targets); });
        robot.reset(); robot.badJoint = true;
        rejects([&] { collection::checkNinePointReachability(1, targets); });
        robot.reset(); robot.inverseCode = static_cast<CRresult>(1);
        rejects([&] { collection::checkNinePointReachability(1, targets); });
        require(robot.sends == 0, "unreachable target sent motion");
        robot.reset();
        collection::moveFlangeLinear(1, origin, targets[1], robot.cancelled, {0, 300});
        require(robot.sends == 1 && robot.stops == 0 && robot.actual == targets[1] && robot.polls >= 3,
                "movement did not confirm stopped actual pose");
        robot.reset(); int telemetry = 0;
        for (double speed : {1.0, 1000.0}) {
            robot.reset(); robot.expectedSpeed = speed;
            collection::moveFlangeLinear(1, origin, targets[1], robot.cancelled, {0, 300}, {}, speed);
            require(robot.sends == 1 && robot.command.speed[0] == speed, "nine-point speed not sent to SDK");
        }
        for (double bad : {0.0, -1.0, 1001.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
            robot.reset();
            rejects([&] { collection::moveFlangeLinear(1, origin, targets[1], robot.cancelled, {0, 300}, {}, bad); });
            require(robot.sends == 0, "invalid nine-point speed dispatched motion");
        }
        robot.reset();
        collection::moveFlangeLinear(1, origin, targets[1], robot.cancelled, {0, 300}, [&] { ++telemetry; });
        require(telemetry >= 3, "worker telemetry callback was not sampled throughout motion");
        robot.reset();
        rejects([&] { collection::moveFlangeLinear(1, origin, targets[1], robot.cancelled, {0, 300},
            [] { throw std::runtime_error("telemetry read failed"); }); });
        require(robot.stops == 1, "telemetry failure during motion did not stop");
        robot.reset(); int delayedTicks = 0;
        const auto timeoutError = rejects([&] {
            collection::waitForStableFlange(1, origin, robot.cancelled, {0, 30}, [&] {
                ++delayedTicks;
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            });
        });
        require(delayedTicks == 1 && robot.actual == origin && robot.sends == 0 &&
                timeoutError.find("超时") != std::string::npos,
                "telemetry callback overrun returned arrival success beyond timeout");
        robot.reset(); robot.cancelled = true;
        rejects([&] { collection::moveFlangeLinear(1, origin, targets[1], robot.cancelled, {0, 100}); });
        require(robot.sends == 0, "pre-cancelled motion was sent");
        robot.reset(); robot.cancelOnPoll = 1;
        rejects([&] { collection::moveFlangeLinear(1, origin, targets[1], robot.cancelled, {0, 100}); });
        require(robot.sends == 1 && robot.stops == 1, "motion cancellation did not request stop");
        robot.reset(); robot.neverReach = true;
        rejects([&] { collection::moveFlangeLinear(1, origin, targets[1], robot.cancelled, {0, 25}); });
        require(robot.stops == 1, "reported idle without reaching target passed or timed out without stop");
        for (int point = 0; point < 9; ++point) {
            if (point == 4) continue;
            robot.reset(); robot.neverReach = true;
            const auto tiny = collection::ninePointTargets(origin, 0.1, 0.1);
            rejects([&] { collection::moveFlangeLinear(1, origin, tiny[point], robot.cancelled, {0, 200}); });
            require(robot.stops == 1 && robot.actual == origin, "0.1 mm move succeeded while still at origin");
            robot.reset();
            collection::moveFlangeLinear(1, origin, tiny[point], robot.cancelled, {0, 300});
            require(collection::flangeAtTarget(robot.actual, tiny[point]), "valid 0.1 mm move rejected");
        }
        auto tinyActual = origin; tinyActual[0] += 0.049;
        auto tinyTarget = origin; tinyTarget[0] += 0.1;
        require(!collection::flangeAtTarget(tinyActual, tinyTarget), "small-offset capture gate is too loose");
        tinyActual[0] = tinyTarget[0] - 0.05;
        require(collection::flangeAtTarget(tinyActual, tinyTarget), "0.05 mm tolerance boundary rejected");
        robot.reset(); robot.lineCode = static_cast<CRresult>(1);
        rejects([&] { collection::moveFlangeLinear(1, origin, targets[1], robot.cancelled, {0, 25}); });
        require(robot.stops == 1, "uncertain command failure did not request stop");
        robot.reset(); robot.neverReach = true; robot.stopCode = static_cast<CRresult>(1);
        require(rejects([&] { collection::moveFlangeLinear(1, origin, targets[1], robot.cancelled, {0, 25}); })
                    .find("停止指令失败") != std::string::npos, "stop failure was concealed");
        robot.reset(); robot.poseCode = static_cast<CRresult>(1);
        rejects([&] { collection::moveFlangeLinear(1, origin, targets[1], robot.cancelled); });
        require(robot.sends == 0, "motion sent after pose read failure");
        robot.reset(); robot.lostPoseDuringMove = true;
        rejects([&] { collection::moveFlangeLinear(1, origin, targets[1], robot.cancelled); });
        require(robot.sends == 1 && robot.stops == 1, "communication loss during motion did not stop");
        robot.reset(); robot.invalidPoseDuringMove = true;
        rejects([&] { collection::moveFlangeLinear(1, origin, targets[1], robot.cancelled); });
        require(robot.stops == 1, "invalid pose during motion did not stop");
        robot.reset(); robot.faultDuringMove = true;
        rejects([&] { collection::moveFlangeLinear(1, origin, targets[1], robot.cancelled); });
        require(robot.stops == 1, "protective stop during motion did not terminate command");
        robot.reset(); robot.badJoint = true;
        rejects([&] { collection::moveFlangeLinear(1, origin, targets[1], robot.cancelled); });
        require(robot.sends == 0, "nonfinite joint reference sent");
        robot.reset(); robot.protective = true;
        rejects([&] { collection::moveFlangeLinear(1, origin, targets[1], robot.cancelled); });
        require(robot.sends == 0, "motion sent in protective stop");
        robot.reset(); auto far = origin; far[0] += 60;
        rejects([&] { collection::moveFlangeLinear(1, origin, far, robot.cancelled); });
        require(robot.sends == 0, "oversized movement sent");
        robot.reset(); auto rotated = targets[0]; rotated[3] += 5;
        rejects([&] { collection::moveFlangeLinear(1, origin, rotated, robot.cancelled); });
        require(robot.sends == 0, "roll-changing movement sent");
        robot.reset(); auto changedHeight = targets[0]; changedHeight[2] += 1;
        rejects([&] { collection::moveFlangeLinear(1, origin, changedHeight, robot.cancelled); });
        require(robot.sends == 0, "height-changing movement sent");
        robot.reset(); auto excessiveYaw = origin;
        excessiveYaw[3] = 0; excessiveYaw[5] = 25;
        auto upright = origin; upright[3] = 0; robot.actual = upright;
        rejects([&] { collection::moveFlangeLinear(1, upright, excessiveYaw, robot.cancelled); });
        require(robot.sends == 0, "excessive Euler yaw sent");
        collection::NinePointAngles smallAngles{}; smallAngles[4] = 0.1;
        const auto smallRotation = collection::ninePointTargets(upright, 10, 10, smallAngles)[4];
        robot.reset(); robot.actual = upright; robot.neverReach = true;
        rejects([&] { collection::moveFlangeLinear(1, upright, smallRotation, robot.cancelled, {0, 100, 0.025}); });
        require(robot.stops == 1 && robot.actual == upright, "0.1 degree center succeeded without rotating");
        robot.reset(); robot.actual = upright;
        collection::moveFlangeLinear(1, upright, smallRotation, robot.cancelled, {0, 300, 0.025});
        require(collection::flangeAtTarget(robot.actual, smallRotation, 0.025), "valid small center rotation rejected");
        // Every rotated leg and its return must pass the same motion guards.
        for (int point = 0; point < 9; ++point) {
            if (point == 4) continue;
            robot.reset();
            collection::moveFlangeLinear(1, origin, targets[point], robot.cancelled, {0, 300});
            collection::moveFlangeLinear(1, origin, origin, robot.cancelled, {0, 300});
            require(robot.sends == 2 && robot.stops == 0 && collection::flangeAtTarget(robot.actual, origin),
                    "rotated point or return to baseline was rejected");
        }
        std::cout << "Nine-point grid, Euler yaw, ordering, save barrier, cancellation, base/zero-tool motion and failures passed\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
