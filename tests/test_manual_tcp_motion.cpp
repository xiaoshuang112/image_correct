#include "hand_eye_gui/ManualTcpMotion.hpp"
#include "utils/calib_transform.hpp"
#include <QCoreApplication>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

namespace {
using collection::FlangePose;
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
template<class F> std::string rejects(F operation) {
    try { operation(); } catch (const std::exception& e) { return e.what(); }
    throw std::runtime_error("unsafe manual motion accepted");
}
constexpr auto sdkError = static_cast<CRresult>(1);
struct Robot {
    FlangePose actual{100, 200, 300, 0, 0, 0};
    FlangePose offset{0, 0, 100, 0, 0, 0};
    FlangePose ikPose{}, ikJoints{};
    PointControlPara command{};
    double expectedSpeed = 10;
    std::atomic<bool> cancelled{false};
    bool issued = false, neverReach = false, alwaysMoving = false, fault = false;
    bool badPose = false, badJoint = false, badInverse = false, badLimit = false;
    bool wrongIdentity = false, changeBeforeSend = false, changeDuringMove = false;
    double limit = 360;
    CRresult lineCode = sucess, stopCode = sucess, inverseCode = sucess, limitCode = sucess, poseCode = sucess;
    int sends = 0, stops = 0, polls = 0, lookups = 0, cancelOnPoll = 0, limitReads = 0;
    void reset() {
        actual = {100, 200, 300, 0, 0, 0}; offset = {0, 0, 100, 0, 0, 0};
        ikPose = {}; ikJoints = {}; command = {}; cancelled = false;
        expectedSpeed = 10;
        issued = neverReach = alwaysMoving = fault = false;
        badPose = badJoint = badInverse = badLimit = false;
        wrongIdentity = changeBeforeSend = changeDuringMove = false; limit = 360;
        lineCode = stopCode = inverseCode = limitCode = poseCode = sucess;
        sends = stops = polls = lookups = cancelOnPoll = limitReads = 0;
    }
} robot;
const FlangePose target{500, -100, 450, 0, 90, 0};
void move(collection::LineMotionWait wait = {0, 300}, collection::MotionTick tick = {}) {
    collection::moveSelectedTcpLinear(1, "selected-probe", target, robot.cancelled, wait, tick);
}
}

extern "C" CRresult cr_cfg_tcp_accord_name_get(RobotHandle, char* name, TCPMsg* tool) {
    require(std::strcmp(name, "selected-probe") == 0, "active tool used instead of selected tool");
    ++robot.lookups;
    *tool = {};
    std::strcpy(tool->tcpName, robot.wrongIdentity ? "other-probe" : "selected-probe");
    if ((robot.changeBeforeSend && robot.lookups == 2) || (robot.changeDuringMove && robot.issued))
        robot.offset[2] += 1;
    std::copy(robot.offset.begin(), robot.offset.end(), tool->tcpOffset);
    return sucess;
}
extern "C" CRresult cr_get_allTCPmsg(RobotHandle, TCPMsg*, int*) {
    throw std::runtime_error("manual move read unrelated TCP list");
}
extern "C" CRresult cr_get_currentTCPmsg(RobotHandle, TCPMsg*) {
    throw std::runtime_error("manual move depended on active TCP");
}
extern "C" CRresult cr_get_robotMode(RobotHandle, RobotModes* mode) {
    *mode = robot.fault ? ProtectiveStop : robot.issued && (robot.polls < 2 || robot.alwaysMoving) ? SDK_Moving : Enable;
    return sucess;
}
extern "C" CRresult cr_get_robotMoveStatus(RobotHandle, BOOL* moving) {
    if (!robot.issued) { *moving = FALSE; return sucess; }
    ++robot.polls;
    if (robot.cancelOnPoll && robot.polls >= robot.cancelOnPoll) robot.cancelled = true;
    *moving = robot.alwaysMoving || robot.polls < 2;
    if (robot.polls >= 2 && !robot.neverReach)
        std::copy_n(robot.command.pose, ROB_AXIS_NUM, robot.actual.begin());
    return sucess;
}
extern "C" CRresult cr_get_AssignTCP_Pose(RobotHandle, double* offset, double* pose) {
    require(std::equal(offset, offset + ROB_AXIS_NUM, robot.offset.begin()), "selected offset not used for telemetry");
    std::copy(robot.actual.begin(), robot.actual.end(), pose);
    if (robot.badPose) pose[2] = std::numeric_limits<double>::quiet_NaN();
    return robot.poseCode;
}
extern "C" CRresult cr_get_jointActualPos(RobotHandle, double* joints) {
    std::fill_n(joints, ROB_AXIS_NUM, 0);
    if (robot.badJoint) joints[0] = std::numeric_limits<double>::quiet_NaN();
    return sucess;
}
extern "C" CRresult cr_kineInverse(RobotHandle, double* pose, double*, double* joints) {
    std::copy_n(pose, ROB_AXIS_NUM, robot.ikPose.begin());
    std::copy(robot.ikJoints.begin(), robot.ikJoints.end(), joints);
    if (robot.badInverse) joints[0] = std::numeric_limits<double>::quiet_NaN();
    return robot.inverseCode;
}
extern "C" CRresult cr_cfg_safety_joint_limit_get(RobotHandle, SafetyLimitsMode mode, int axis, SafetyLimitsJointAngle* limit) {
    require(axis >= 0 && axis < ROB_AXIS_NUM, "SDK safety index must be zero-based");
    require(mode == mode_Normal || mode == mode_Reduced, "invalid safety envelope");
    ++robot.limitReads;
    *limit = {180, -robot.limit, robot.limit};
    if (robot.badLimit) limit->minJointPosition = std::numeric_limits<double>::quiet_NaN();
    return robot.limitCode;
}
extern "C" CRresult cr_move_line(RobotHandle, PointControlPara command, BOOL block) {
    require(!block, "SDK call blocked cancellation");
    require(command.tcpID < 0, "motion inherited active TCP");
    require(std::equal(robot.offset.begin(), robot.offset.end(), command.tcpOffset), "fresh selected offset missing from motion");
    require(std::equal(target.begin(), target.end(), command.pose), "flange target substituted for TCP trajectory");
    require(command.coordinateType == baseCoordinate && command.pointTransType == pointTransStop &&
            command.pointTransRadius == 0 && command.motiontriggerMode == MovetriggerbyOnlyRpc,
            "invalid frame, blending, or deferred trigger");
    for (int axis = 0; axis < ROB_AXIS_NUM; ++axis)
        require(command.speed[axis] == robot.expectedSpeed && command.acc[axis] == 20 && command.coordinatePose[axis] == 0,
                "manual linear speed/frame changed");
    robot.command = command; robot.issued = true; ++robot.sends;
    return robot.lineCode;
}
extern "C" CRresult cr_stop(RobotHandle) { ++robot.stops; return robot.stopCode; }

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);
    try {
        // A 90-degree TCP orientation moves the flange in X rather than Z.
        const auto flange = collection::tcpTargetToFlange(target, robot.offset);
        require(std::abs(flange[0] - 400) < 1e-9 && std::abs(flange[1] + 100) < 1e-9 &&
                std::abs(flange[2] - 450) < 1e-9, "TCP translation ignored target orientation");
        const FlangePose rotatedTool{30, -20, 100, 40, 20, -10};
        const auto converted = collection::tcpTargetToFlange(target, rotatedTool);
        const auto recomposed = calib_transform::axisAnglePoseToTransform(converted.data()) *
            calib_transform::axisAnglePoseToTransform(rotatedTool.data());
        require(cv::norm(recomposed - calib_transform::axisAnglePoseToTransform(target.data())) < 1e-9,
                "tool rotation/translation composition is incorrect");
        int ticks = 0;
        robot.offset = rotatedTool;
        move({0, 300}, [&] { ++ticks; });
        require(robot.sends == 1 && robot.stops == 0 && ticks >= 3 && robot.limitReads == 12 && robot.lookups >= 5,
                "manual move failed to confirm actual selected TCP and stationary state");
        require(collection::flangeAtTarget(robot.ikPose, converted), "IK preflight used TCP instead of flange");
        // Far translations and arbitrary orientations are intentionally allowed.
        require(robot.actual == target, "manual move inherited nine-point distance/orientation restriction");
        robot.reset(); robot.offset[2] = 250;
        move();
        require(robot.command.tcpOffset[2] == 250, "cached tool offset sent to motion");
        for (double speed : {1.0, 1000.0}) {
            robot.reset(); robot.expectedSpeed = speed;
            collection::moveSelectedTcpLinear(1, "selected-probe", target, robot.cancelled, {0, 300}, {}, speed);
            require(robot.sends == 1 && robot.command.speed[0] == speed, "manual speed not sent to SDK");
        }
        for (double bad : {0.0, -1.0, 1001.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
            robot.reset();
            rejects([&] { collection::moveSelectedTcpLinear(1, "selected-probe", target, robot.cancelled, {0, 300}, {}, bad); });
            require(robot.sends == 0, "invalid manual speed dispatched motion");
        }
        for (int scenario = 0; scenario < 12; ++scenario) {
            robot.reset();
            switch (scenario) {
            case 0: robot.cancelled = true; break;
            case 1: robot.wrongIdentity = true; break;
            case 2: robot.offset[0] = std::numeric_limits<double>::infinity(); break;
            case 3: robot.badPose = true; break;
            case 4: robot.badJoint = true; break;
            case 5: robot.inverseCode = sdkError; break;
            case 6: robot.badInverse = true; break;
            case 7: robot.badLimit = true; break;
            case 8: robot.limitCode = sdkError; break;
            case 9: robot.ikJoints[0] = 361; break;
            case 10: robot.changeBeforeSend = true; break;
            case 11: robot.fault = true; break;
            }
            rejects([] { move(); });
            require(robot.sends == 0, "failed preflight dispatched motion");
        }
        robot.reset(); auto invalidTarget = target; invalidTarget[0] = std::numeric_limits<double>::quiet_NaN();
        rejects([&] { collection::moveSelectedTcpLinear(1, "selected-probe", invalidTarget, robot.cancelled); });
        require(robot.sends == 0, "invalid target sent motion");
        robot.reset(); rejects([] { move({-1, 100}); });
        require(robot.sends == 0, "invalid timeout dispatched motion");
        // Failures during an uncertain dispatch or while polling always request stop.
        robot.reset(); robot.lineCode = sdkError;
        rejects([] { move(); }); require(robot.stops == 1, "failed command did not stop");
        robot.reset(); robot.cancelOnPoll = 1;
        rejects([] { move(); }); require(robot.stops == 1, "cancellation did not stop");
        robot.reset(); robot.neverReach = true;
        rejects([] { move({0, 80}); }); require(robot.stops == 1, "idle away from target passed");
        robot.reset(); robot.alwaysMoving = true;
        rejects([] { move({0, 80}); }); require(robot.stops == 1, "moving at target passed");
        robot.reset(); robot.changeDuringMove = true;
        rejects([] { move(); }); require(robot.stops == 1, "tool change during motion did not stop");
        robot.reset(); rejects([] { move({0, 300}, [] { throw std::runtime_error("telemetry failure"); }); });
        require(robot.stops == 1, "callback failure did not stop");
        robot.reset(); rejects([] { move({0, 300}, [] { robot.poseCode = sdkError; }); });
        require(robot.stops == 1, "pose RPC loss during motion did not stop");
        robot.reset(); rejects([] { move({0, 300}, [] { robot.badPose = true; }); });
        require(robot.stops == 1, "nonfinite pose during motion did not stop");
        robot.reset(); rejects([] { move({0, 300}, [] { robot.fault = true; }); });
        require(robot.stops == 1, "protective stop during motion did not fail command");
        robot.reset();
        bool unknownFailure = false;
        try { move({0, 300}, [] { throw 7; }); } catch (int) { unknownFailure = true; }
        require(unknownFailure && robot.stops == 1, "nonstandard callback exception bypassed stop");
        robot.reset(); int delayedTicks = 0;
        rejects([&] { move({0, 130}, [&] {
            if (++delayedTicks == 3) std::this_thread::sleep_for(std::chrono::milliseconds(70));
        }); });
        require(robot.stops == 1, "callback overrun returned success beyond timeout");
        robot.reset(); robot.neverReach = true; robot.stopCode = sdkError;
        require(rejects([] { move({0, 60}); }).find("停止指令失败") != std::string::npos,
                "manual stop failure concealed");
        robot.reset(); move({60, 400});
        require(robot.polls >= 5, "settling interval was not confirmed");
        std::cout << "Manual selected-TCP trajectory, transform, preflight, stop and stationary checks passed\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
