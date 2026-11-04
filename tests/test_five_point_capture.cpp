#include "hand_eye_gui/FivePointCapture.hpp"
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
        const auto targets = collection::fivePointTargets(origin, 10, 12.5);
        require(targets[0] == origin && targets[1][0] == 110 && targets[2][0] == 90 &&
                targets[3][1] == 212.5 && targets[4][1] == 187.5, "incorrect base offsets");
        for (const auto& p : targets)
            require(std::equal(p.begin() + 2, p.end(), origin.begin() + 2), "height or attitude changed");
        for (double bad : {0.0, -10.0, 0.09, 50.01, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
            rejects([&] { collection::fivePointTargets(origin, bad, 10); });
            rejects([&] { collection::fivePointTargets(origin, 10, bad); });
        }
        collection::fivePointTargets(origin, 0.1, 50);
        auto invalid = origin; invalid[2] = std::numeric_limits<double>::quiet_NaN();
        rejects([&] { collection::fivePointTargets(invalid, 10, 10); });
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
        collection::runFivePointCapture(targets, move, capture, cancelled);
        require(events == std::vector<std::string>{"save0", "move1", "save1", "move0", "move2", "save2",
                "move0", "move3", "save3", "move0", "move4", "save4", "move0"}, "route or save/move order incorrect");
        require(current == origin, "normal completion did not return to origin");
        events.clear();
        rejects([&] { collection::runFivePointCapture(targets, move, [&](int point) {
            capture(point); if (point == 2) throw std::runtime_error("disk full");
        }, cancelled); });
        require(events.back() == "save2" && current == targets[2], "moved after save failure");
        events.clear(); current = origin;
        rejects([&] { collection::runFivePointCapture(targets, move, [&](int point) {
            capture(point); cancelled = true;
        }, cancelled); });
        require(events == std::vector<std::string>{"save0"}, "cancel still issued motion");
        events.clear(); cancelled = false;
        rejects([&] { collection::runFivePointCapture(targets, [&](const FlangePose&, int) {
            events.push_back("failed_move"); throw std::runtime_error("motion failed");
        }, capture, cancelled); });
        require(events == std::vector<std::string>{"save0", "failed_move"}, "capture or return followed failed move");

        collection::checkFivePointReachability(1, targets);
        require(robot.inverseReads == 5 && robot.sends == 0, "preflight moved robot or missed a target");
        robot.badInverse = true;
        rejects([&] { collection::checkFivePointReachability(1, targets); });
        robot.reset(); robot.badJoint = true;
        rejects([&] { collection::checkFivePointReachability(1, targets); });
        robot.reset(); robot.inverseCode = static_cast<CRresult>(1);
        rejects([&] { collection::checkFivePointReachability(1, targets); });
        require(robot.sends == 0, "unreachable target sent motion");
        robot.reset();
        collection::moveFlangeLinear(1, targets[1], robot.cancelled, {0, 300});
        require(robot.sends == 1 && robot.stops == 0 && robot.actual == targets[1] && robot.polls >= 3,
                "movement did not confirm stopped actual pose");
        robot.reset(); int telemetry = 0;
        for (double speed : {1.0, 1000.0}) {
            robot.reset(); robot.expectedSpeed = speed;
            collection::moveFlangeLinear(1, targets[1], robot.cancelled, {0, 300}, {}, speed);
            require(robot.sends == 1 && robot.command.speed[0] == speed, "five-point speed not sent to SDK");
        }
        for (double bad : {0.0, -1.0, 1001.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
            robot.reset();
            rejects([&] { collection::moveFlangeLinear(1, targets[1], robot.cancelled, {0, 300}, {}, bad); });
            require(robot.sends == 0, "invalid five-point speed dispatched motion");
        }
        robot.reset();
        collection::moveFlangeLinear(1, targets[1], robot.cancelled, {0, 300}, [&] { ++telemetry; });
        require(telemetry >= 3, "worker telemetry callback was not sampled throughout motion");
        robot.reset();
        rejects([&] { collection::moveFlangeLinear(1, targets[1], robot.cancelled, {0, 300},
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
        rejects([&] { collection::moveFlangeLinear(1, targets[1], robot.cancelled, {0, 100}); });
        require(robot.sends == 0, "pre-cancelled motion was sent");
        robot.reset(); robot.cancelOnPoll = 1;
        rejects([&] { collection::moveFlangeLinear(1, targets[1], robot.cancelled, {0, 100}); });
        require(robot.sends == 1 && robot.stops == 1, "motion cancellation did not request stop");
        robot.reset(); robot.neverReach = true;
        rejects([&] { collection::moveFlangeLinear(1, targets[1], robot.cancelled, {0, 25}); });
        require(robot.stops == 1, "reported idle without reaching target passed or timed out without stop");
        for (int point = 1; point < 5; ++point) {
            robot.reset(); robot.neverReach = true;
            const auto tiny = collection::fivePointTargets(origin, 0.1, 0.1);
            rejects([&] { collection::moveFlangeLinear(1, tiny[point], robot.cancelled, {0, 200}); });
            require(robot.stops == 1 && robot.actual == origin, "0.1 mm move succeeded while still at origin");
            robot.reset();
            collection::moveFlangeLinear(1, tiny[point], robot.cancelled, {0, 300});
            require(collection::flangeAtTarget(robot.actual, tiny[point]), "valid 0.1 mm move rejected");
        }
        auto tinyActual = origin; tinyActual[0] += 0.049;
        auto tinyTarget = origin; tinyTarget[0] += 0.1;
        require(!collection::flangeAtTarget(tinyActual, tinyTarget), "small-offset capture gate is too loose");
        tinyActual[0] = tinyTarget[0] - 0.05;
        require(collection::flangeAtTarget(tinyActual, tinyTarget), "0.05 mm tolerance boundary rejected");
        robot.reset(); robot.lineCode = static_cast<CRresult>(1);
        rejects([&] { collection::moveFlangeLinear(1, targets[1], robot.cancelled, {0, 25}); });
        require(robot.stops == 1, "uncertain command failure did not request stop");
        robot.reset(); robot.neverReach = true; robot.stopCode = static_cast<CRresult>(1);
        require(rejects([&] { collection::moveFlangeLinear(1, targets[1], robot.cancelled, {0, 25}); })
                    .find("停止指令失败") != std::string::npos, "stop failure was concealed");
        robot.reset(); robot.poseCode = static_cast<CRresult>(1);
        rejects([&] { collection::moveFlangeLinear(1, targets[1], robot.cancelled); });
        require(robot.sends == 0, "motion sent after pose read failure");
        robot.reset(); robot.lostPoseDuringMove = true;
        rejects([&] { collection::moveFlangeLinear(1, targets[1], robot.cancelled); });
        require(robot.sends == 1 && robot.stops == 1, "communication loss during motion did not stop");
        robot.reset(); robot.invalidPoseDuringMove = true;
        rejects([&] { collection::moveFlangeLinear(1, targets[1], robot.cancelled); });
        require(robot.stops == 1, "invalid pose during motion did not stop");
        robot.reset(); robot.faultDuringMove = true;
        rejects([&] { collection::moveFlangeLinear(1, targets[1], robot.cancelled); });
        require(robot.stops == 1, "protective stop during motion did not terminate command");
        robot.reset(); robot.badJoint = true;
        rejects([&] { collection::moveFlangeLinear(1, targets[1], robot.cancelled); });
        require(robot.sends == 0, "nonfinite joint reference sent");
        robot.reset(); robot.protective = true;
        rejects([&] { collection::moveFlangeLinear(1, targets[1], robot.cancelled); });
        require(robot.sends == 0, "motion sent in protective stop");
        robot.reset(); auto far = origin; far[0] += 60;
        rejects([&] { collection::moveFlangeLinear(1, far, robot.cancelled); });
        require(robot.sends == 0, "oversized movement sent");
        robot.reset(); auto rotated = targets[1]; rotated[3] += 5;
        rejects([&] { collection::moveFlangeLinear(1, rotated, robot.cancelled); });
        require(robot.sends == 0, "attitude-changing movement sent");
        std::cout << "Five-point ordering, save barrier, cancellation, base/zero-tool line motion and failures passed\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
