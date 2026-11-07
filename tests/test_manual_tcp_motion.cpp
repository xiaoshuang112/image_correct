#include "hand_eye_gui/ManualTcpMotion.hpp"
#include "hand_eye_gui/TeachMotion.hpp"
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
#include <vector>

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
    std::vector<PointControlPara> commands;
    double expectedSpeed = 10;
    std::atomic<bool> cancelled{false};
    bool issued = false, neverReach = false, alwaysMoving = false, fault = false;
    bool badPose = false, badJoint = false, badInverse = false, badLimit = false;
    bool wrongIdentity = false, changeBeforeSend = false, changeDuringMove = false;
    bool strictTarget = true;
    double limit = 360, reducedLimit = 360;
    double inverseMaxX = std::numeric_limits<double>::infinity();
    double jointLimitFromX = std::numeric_limits<double>::infinity();
    double arrivalPositionError = 0, arrivalAngleError = 0;
    std::vector<FlangePose> inverseTargets;
    CRresult lineCode = sucess, stopCode = sucess, inverseCode = sucess, limitCode = sucess, poseCode = sucess;
    int sends = 0, stops = 0, polls = 0, lookups = 0, cancelOnPoll = 0, limitReads = 0;
    int slowPoseLookup = 0, poseDelayMs = 270, limitDelayMs = 0, cancelOnLimitRead = 0;
    std::function<void()> poseReadAction;
    void reset() {
        actual = {100, 200, 300, 0, 0, 0}; offset = {0, 0, 100, 0, 0, 0};
        ikPose = {}; ikJoints = {}; command = {}; cancelled = false;
        commands.clear(); slowPoseLookup = 0; poseDelayMs = 270;
        limitDelayMs = cancelOnLimitRead = 0; poseReadAction = {};
        inverseTargets.clear();
        inverseMaxX = jointLimitFromX = std::numeric_limits<double>::infinity();
        arrivalPositionError = arrivalAngleError = 0;
        expectedSpeed = 10;
        issued = neverReach = alwaysMoving = fault = false;
        badPose = badJoint = badInverse = badLimit = false;
        wrongIdentity = changeBeforeSend = changeDuringMove = false; limit = reducedLimit = 360;
        strictTarget = true;
        lineCode = stopCode = inverseCode = limitCode = poseCode = sucess;
        sends = stops = polls = lookups = cancelOnPoll = limitReads = 0;
    }
} robot;
// The real UI renews the press independently of the worker's blocking SDK calls.
class LeaseHeartbeat {
public:
    explicit LeaseHeartbeat(collection::TeachJogLease& lease) : worker_([this, &lease] {
        while (!done_.load()) {
            lease.heartbeat();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }) {}
    ~LeaseHeartbeat() { done_.store(true); worker_.join(); }
private:
    std::atomic<bool> done_{false};
    std::thread worker_;
};
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
    if (robot.polls >= 2 && !robot.neverReach) {
        std::copy_n(robot.command.pose, ROB_AXIS_NUM, robot.actual.begin());
        robot.actual[0] += robot.arrivalPositionError;
        robot.actual[3] += robot.arrivalAngleError;
    }
    return sucess;
}
extern "C" CRresult cr_get_AssignTCP_Pose(RobotHandle, double* offset, double* pose) {
    if (robot.poseReadAction) robot.poseReadAction();
    if (robot.slowPoseLookup && robot.lookups >= robot.slowPoseLookup)
        std::this_thread::sleep_for(std::chrono::milliseconds(robot.poseDelayMs));
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
    robot.inverseTargets.push_back(robot.ikPose);
    std::copy(robot.ikJoints.begin(), robot.ikJoints.end(), joints);
    if (pose[0] >= robot.jointLimitFromX) joints[0] = std::min(robot.limit, robot.reducedLimit) + 1;
    if (robot.badInverse) joints[0] = std::numeric_limits<double>::quiet_NaN();
    return pose[0] > robot.inverseMaxX ? sdkError : robot.inverseCode;
}
extern "C" CRresult cr_cfg_safety_joint_limit_get(RobotHandle, SafetyLimitsMode mode, int axis, SafetyLimitsJointAngle* limit) {
    require(axis >= 0 && axis < ROB_AXIS_NUM, "SDK safety index must be zero-based");
    require(mode == mode_Normal || mode == mode_Reduced, "invalid safety envelope");
    ++robot.limitReads;
    if (robot.limitDelayMs) std::this_thread::sleep_for(std::chrono::milliseconds(robot.limitDelayMs));
    if (robot.cancelOnLimitRead && robot.limitReads >= robot.cancelOnLimitRead) robot.cancelled = true;
    const double bound = mode == mode_Reduced ? robot.reducedLimit : robot.limit;
    *limit = {180, -bound, bound};
    if (robot.badLimit) limit->minJointPosition = std::numeric_limits<double>::quiet_NaN();
    return robot.limitCode;
}
extern "C" CRresult cr_move_line(RobotHandle, PointControlPara command, BOOL block) {
    require(!block, "SDK call blocked cancellation");
    require(command.tcpID < 0, "motion inherited active TCP");
    require(std::equal(robot.offset.begin(), robot.offset.end(), command.tcpOffset), "fresh selected offset missing from motion");
    if (robot.strictTarget)
        require(std::equal(target.begin(), target.end(), command.pose), "flange target substituted for TCP trajectory");
    require(command.coordinateType == baseCoordinate && command.pointTransType == pointTransStop &&
            command.pointTransRadius == 0 && command.motiontriggerMode == MovetriggerbyOnlyRpc,
            "invalid frame, blending, or deferred trigger");
    for (int axis = 0; axis < ROB_AXIS_NUM; ++axis)
        require(command.speed[axis] == robot.expectedSpeed && command.acc[axis] == 20 && command.coordinatePose[axis] == 0,
                "manual linear speed/frame changed");
    robot.command = command; robot.issued = true; ++robot.sends;
    robot.commands.push_back(command);
    return robot.lineCode;
}
extern "C" CRresult cr_moveControl(RobotHandle, MoveType type) {
    require(type == DecStop, "SDK motion must use deceleration stop");
    ++robot.stops;
    return robot.stopCode;
}

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
        require(robot.lookups == 2 + robot.polls,
                "manual motion read extra TCP snapshots instead of reusing its own samples");
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
        for (bool changePose : {false, true}) {
            robot.reset(); int tickCount = 0;
            rejects([&] { move({0, 220}, [&] {
                if (++tickCount == 3) {
                    if (changePose) { robot.neverReach = true; robot.actual[0] = target[0] + 1; }
                    else robot.alwaysMoving = true;
                }
            }); });
            require(robot.stops == 1,
                    "arrival used robot state or TCP pose captured before the tick");
        }
        // The deadline includes preparation. A late successful read or limit
        // response must not be followed by a movement command.
        for (int stage = 0; stage < 3; ++stage) {
            robot.reset();
            if (stage < 2) { robot.slowPoseLookup = stage + 1; robot.poseDelayMs = 80; }
            else robot.limitDelayMs = 80;
            const auto failure = rejects([&] { move({0, 60}); });
            require(robot.sends == 0 && failure == "TCP 单段运动超时，请检查机器人连接和状态" &&
                    robot.lookups == (stage == 1 ? 2 : 1),
                    "late preparation response bypassed the deadline or exposed internal stage details");
            if (stage == 2) require(robot.limitReads == 1,
                    "expired limit read continued through the remaining safety limits");
        }
        robot.reset(); robot.cancelOnLimitRead = 1;
        rejects([] { move(); });
        require(robot.sends == 0 && robot.limitReads == 1,
                "cancelled limit read continued toward dispatch");
        robot.reset(); robot.neverReach = true; robot.stopCode = sdkError;
        require(rejects([] { move({0, 60}); }).find("停止指令失败") != std::string::npos,
                "manual stop failure concealed");
        robot.reset(); move({60, 400});
        require(robot.polls >= 5, "settling interval was not confirmed");
        using collection::TeachOrientation;
        // Each representation edits only its selected displayed component; XYZ
        // is held at the selected TCP center, even for an offset/rotated tool.
        const FlangePose euler{125, -45, 320, 25, -30, 40};
        const auto vector = collection::tcpPoseFromDisplay(euler, TeachOrientation::EulerZYX);
        const auto roundtrip = collection::tcpPoseForDisplay(vector, TeachOrientation::EulerZYX);
        for (int axis = 0; axis < 6; ++axis)
            require(std::abs(roundtrip[axis] - euler[axis]) < 1e-9, "ZYX Euler roundtrip failed");
        for (int axis = 0; axis < 6; ++axis) {
            const auto next = collection::teachJogTarget(euler, TeachOrientation::EulerZYX, axis, 0.5);
            const auto displayed = collection::tcpPoseForDisplay(next, TeachOrientation::EulerZYX);
            for (int i = 0; i < 6; ++i)
                require(std::abs(displayed[i] - euler[i] - (i == axis ? 0.5 : 0)) < 1e-8,
                    "Euler jog changed another component or TCP center");
            const auto rv = collection::teachJogTarget(vector, TeachOrientation::RotationVector, axis, 0.5);
            for (int i = 0; i < 6; ++i)
                require(std::abs(rv[i] - vector[i] - (i == axis ? 0.5 : 0)) < 1e-9,
                    "rotation-vector jog changed another component or TCP center");
        }
        auto singular = euler; singular[4] = 89;
        rejects([&] { collection::tcpPoseFromDisplay(singular, TeachOrientation::EulerZYX); });
        singular[4] = 88.9;
        rejects([&] { collection::teachJogTarget(singular, TeachOrientation::EulerZYX, 4, .2); });
        const FlangePose branch{0, 0, 0, 358.9, 0, 0};
        rejects([&] { collection::teachJogTarget(branch, TeachOrientation::RotationVector, 3, .2); });
        const FlangePose commonOrientation{0, 0, 0, 179.95, 0, 0};
        const auto crossed = collection::teachJogTarget(commonOrientation, TeachOrientation::RotationVector, 3, .1);
        require(crossed[3] > 180 && crossed[3] < 180.1, "normal 180-degree rotation branch was blocked");
        rejects([&] { collection::teachJogTarget(vector, TeachOrientation::RotationVector, 6, .2); });
        require(collection::validTeachSpeeds({10, 5}) && collection::validTeachSpeeds({1000, 180}) &&
                !collection::validTeachSpeeds({1001, 5}) && !collection::validTeachSpeeds({10, 0}) &&
                !collection::validTeachSpeeds({10, std::numeric_limits<double>::quiet_NaN()}),
                "independent teaching speeds invalid");
        collection::TeachJogRequest jog;
        robot.reset(); robot.strictTarget = false;
        collection::TeachJogLease released; released.release(); released.heartbeat();
        collection::jogSelectedTcp(1, "selected-probe", jog, released, robot.cancelled);
        require(robot.sends == 0 && !released.valid(), "released press was restarted by heartbeat");
        collection::TeachJogLease expired;
        std::this_thread::sleep_for(std::chrono::milliseconds(collection::TeachJogLease::expiryMs + 5));
        collection::jogSelectedTcp(1, "selected-probe", jog, expired, robot.cancelled);
        require(robot.sends == 0, "expired GUI heartbeat dispatched motion");
        expired.heartbeat();
        require(!expired.valid(), "late GUI heartbeat resurrected an expired press");
        robot.reset(); robot.strictTarget = false;
        collection::TeachJogLease held;
        rejects([&] { collection::jogSelectedTcp(1, "selected-probe", jog, held, robot.cancelled,
            [&] { held.release(); }); });
        require(robot.sends == 1 && robot.stops == 1 && robot.command.pose[0] == 101 &&
                robot.command.pose[1] == 200 && robot.command.pose[2] == 300,
                "release failed to stop a single bounded selected-TCP segment");
        // Translation must not use the 0.005-degree angular-jog arrival test:
        // a small orientation residual used to abort after the first X step.
        for (double residual : {-0.2, -0.006, 0.006, 0.19, 0.2}) {
            robot.reset(); robot.strictTarget = false; robot.expectedSpeed = 100;
            robot.arrivalAngleError = residual;
            collection::TeachJogRequest translation;
            translation.direction = -1; translation.speeds.linearMmS = 100;
            collection::TeachJogLease press;
            const auto reason = rejects([&] {
                collection::jogSelectedTcp(1, "selected-probe", translation, press, robot.cancelled, [&] {
                    press.heartbeat();
                    if (robot.sends >= 3) press.release();
                });
            });
            require(robot.sends == 3 && robot.stops == 1 && robot.command.pose[0] == 97 &&
                    reason.find("意外变化") == std::string::npos,
                    "translation jog rejected an allowed orientation residual");
            for (const auto& command : robot.commands)
                require(command.pose[3] == 0 && command.pose[4] == 0 && command.pose[5] == 0,
                        "translation jog accumulated orientation residual in its targets");
        }
        for (double residual : {-0.201, 0.201}) {
            robot.reset(); robot.strictTarget = false; robot.arrivalAngleError = residual;
            collection::TeachJogLease excessiveAngle;
            const auto jogDeviation = rejects([&] {
                collection::jogSelectedTcp(1, "selected-probe", jog, excessiveAngle, robot.cancelled,
                    [&] { excessiveAngle.heartbeat(); });
            });
            require(robot.sends == 1 && robot.stops == 1 && jogDeviation.find("意外变化") != std::string::npos,
                    "translation jog ignored excessive orientation deviation");
        }
        robot.reset(); robot.strictTarget = false; robot.arrivalAngleError = 0.19;
        collection::TeachJogLease cancelledTranslation;
        rejects([&] {
            collection::jogSelectedTcp(1, "selected-probe", jog, cancelledTranslation, robot.cancelled, [&] {
                cancelledTranslation.heartbeat();
                if (robot.sends >= 2) robot.cancelled = true;
            });
        });
        require(robot.sends == 2 && robot.stops == 1,
                "translation jog ignored cancellation with an allowed orientation residual");
        for (bool rotating : {false, true}) {
            robot.reset(); robot.strictTarget = false;
            collection::TeachJogRequest precision;
            precision.component = rotating ? 3 : 0;
            robot.expectedSpeed = rotating ? 1 : 10;
            if (rotating) robot.arrivalAngleError = 0.006;
            else robot.arrivalPositionError = collection::positionToleranceMm + 0.001;
            collection::TeachJogLease press;
            rejects([&] {
                collection::jogSelectedTcp(1, "selected-probe", precision, press, robot.cancelled, [&] {
                    press.heartbeat();
                    if (robot.polls >= 4) press.release();
                });
            });
            require(robot.sends == 1 && robot.stops == 1,
                    "jog advanced before satisfying position or angular arrival precision");
        }
        robot.reset(); robot.strictTarget = false; robot.neverReach = true;
        collection::TeachJogLease noMovement;
        rejects([&] { collection::jogSelectedTcp(1, "selected-probe", jog, noMovement, robot.cancelled); });
        require(robot.sends == 1 && robot.stops == 1, "idle robot away from jog target accepted repeated motion");
        robot.reset(); robot.strictTarget = false; robot.changeBeforeSend = true;
        collection::TeachJogLease toolChanged;
        rejects([&] { collection::jogSelectedTcp(1, "selected-probe", jog, toolChanged, robot.cancelled); });
        require(robot.sends == 0, "changed tool accepted by a later jog segment");
        // A compound SDK read can take over 100ms while the independent UI
        // heartbeat remains healthy. It must not be mistaken for a stale pose.
        robot.reset(); robot.strictTarget = false; robot.slowPoseLookup = 1; robot.poseDelayMs = 120;
        collection::TeachJogLease slowHeld;
        {
            LeaseHeartbeat heartbeat(slowHeld);
            const auto failure = rejects([&] {
                collection::jogSelectedTcp(1, "selected-probe", jog, slowHeld, robot.cancelled,
                    [&] { if (robot.sends >= 2) slowHeld.release(); });
            });
            require(robot.sends == 2 && robot.stops == 1 && failure.find("读取过慢") == std::string::npos &&
                    robot.lookups == 8,
                    "healthy held jog rejected slow successful reads or duplicated snapshots");
        }
        // Stop or expiry during origin, preflight, final refresh and arrival
        // reads must be observed immediately when the blocking read returns.
        for (int lookup : {1, 2, 3, 4}) {
            for (int action = 0; action < 3; ++action) {
                robot.reset(); robot.strictTarget = false; robot.slowPoseLookup = lookup;
                robot.poseDelayMs = action == 2 ? collection::TeachJogLease::expiryMs + 20 : 120;
                collection::TeachJogLease reading;
                robot.poseReadAction = [&] {
                    if (robot.lookups == lookup) {
                        if (action == 0) reading.release();
                        else if (action == 1) robot.cancelled = true;
                    }
                };
                rejects([&] { collection::jogSelectedTcp(1, "selected-probe", jog, reading, robot.cancelled); });
                require(robot.sends == (lookup == 4 ? 1 : 0) && robot.stops == (lookup == 4 ? 1 : 0) &&
                        robot.lookups == lookup,
                        "control lost during SDK read was ignored before dispatch or stop");
            }
        }
        robot.poseReadAction = {};
        for (int lookup : {1, 2}) {
            robot.reset(); robot.strictTarget = false; robot.slowPoseLookup = lookup; robot.poseDelayMs = 100;
            auto shortTarget = robot.actual; shortTarget[0] += 1;
            const auto failure = rejects([&] {
                collection::moveSelectedTcpTarget(1, "selected-probe", shortTarget, {100, 5},
                    robot.cancelled, {0, 80});
            });
            require(robot.sends == 0 && robot.lookups == lookup && failure.find("总时间超时") != std::string::npos,
                    "origin or complete endpoint preflight escaped the total target deadline");
        }
        robot.reset(); robot.strictTarget = false; robot.expectedSpeed = 1;
        collection::TeachJogLease angular;
        jog.component = 3;
        rejects([&] { collection::jogSelectedTcp(1, "selected-probe", jog, angular, robot.cancelled,
            [&] { if (robot.polls >= 2) angular.release(); }); });
        require(robot.sends == 1 && robot.stops == 1 && robot.command.pose[0] == 100 &&
            robot.command.pose[1] == 200 && robot.command.pose[2] == 300 &&
            std::abs(robot.command.pose[3] - .1) < 1e-9,
            "rotation jog failed bounded angle and fixed TCP center");
        robot.reset(); robot.strictTarget = false; robot.expectedSpeed = 1; robot.neverReach = true;
        collection::TeachJogLease noRotation;
        rejects([&] { collection::jogSelectedTcp(1, "selected-probe", jog, noRotation, robot.cancelled); });
        require(robot.sends == 1 && robot.stops == 1, "angular tolerance swallowed an unmoved jog step");
        robot.reset(); robot.strictTarget = false; robot.expectedSpeed = 1;
        auto angularTarget = robot.actual; angularTarget[3] = .2;
        const auto angularBegan = std::chrono::steady_clock::now();
        collection::moveSelectedTcpTarget(1, "selected-probe", angularTarget, {10, .5}, robot.cancelled, {0, 2000});
        require(robot.sends == 2 && std::chrono::steady_clock::now() - angularBegan >= std::chrono::milliseconds(395) &&
            collection::flangeAtTarget(robot.actual, angularTarget, .00001), "angular progression did not pace serial bounded targets");
        robot.reset(); robot.strictTarget = false; robot.expectedSpeed = 1;
        robot.actual[3] = 179.95;
        auto across = robot.actual; across[3] = 180.05;
        collection::moveSelectedTcpTarget(1, "selected-probe", across, {10, 5}, robot.cancelled, {0, 2000}, {}, TeachOrientation::EulerZYX);
        require(robot.sends <= 2 && robot.command.pose[3] > 180 && robot.command.pose[3] < 181,
            "Euler roll crossing 180 degrees caused a rotation-vector jump");
        robot.reset(); robot.strictTarget = false; robot.expectedSpeed = 1;
        const FlangePose multiStart{100, 200, 300, 10, 80, 30};
        const FlangePose multiFinish{100, 200, 300, 10.2, 80, 29.8};
        robot.actual = collection::tcpPoseFromDisplay(multiStart, TeachOrientation::EulerZYX);
        auto previous = robot.actual;
        const auto multiTarget = collection::tcpPoseFromDisplay(multiFinish, TeachOrientation::EulerZYX);
        collection::moveSelectedTcpTarget(1, "selected-probe", multiTarget, {10, 5}, robot.cancelled,
            {0, 2000}, {}, TeachOrientation::EulerZYX);
        require(robot.sends >= 4, "multi-component Euler target was not divided conservatively");
        for (const auto& command : robot.commands) {
            FlangePose issued{}; std::copy_n(command.pose, 6, issued.begin());
            require(collection::flangeAtTarget(issued, previous, .100001),
                "multi-component step exceeded .1 degree or moved TCP center");
            previous = issued;
        }
        // Reject the complete requested destination before moving through any
        // otherwise reachable intermediate positions.
        for (int failure = 0; failure < 3; ++failure) {
            robot.reset(); robot.strictTarget = false;
            if (failure == 0) robot.inverseMaxX = 102;
            else robot.jointLimitFromX = 103;
            if (failure == 2) robot.reducedLimit = 180;
            auto unreachable = robot.actual; unreachable[0] = 104;
            rejects([&] { collection::moveSelectedTcpTarget(1, "selected-probe", unreachable,
                {10, 5}, robot.cancelled, {0, 2000}); });
            require(robot.sends == 0, "unreachable complete target moved through partial targets first");
        }
        for (bool rotating : {false, true}) {
            robot.reset(); robot.strictTarget = false; robot.expectedSpeed = 1;
            auto tooSlow = robot.actual; tooSlow[rotating ? 3 : 0] += 1;
            rejects([&] { collection::moveSelectedTcpTarget(1, "selected-probe", tooSlow,
                {1, .1}, robot.cancelled, {0, 200}); });
            require(robot.sends == 0, "impossible total motion duration was rejected only after moving");
        }
        robot.reset(); robot.strictTarget = false; robot.expectedSpeed = 1;
        auto mixed = robot.actual; mixed[0] += 1; mixed[3] += .1;
        rejects([&] { collection::moveSelectedTcpTarget(1, "selected-probe", mixed,
            {100, 180}, robot.cancelled, {0, 200}); });
        require(robot.sends == 0, "duration check ignored the 1 mm/s SDK limit for rotating segments");
        // A pure translation can settle with an angular residual allowed by
        // the configured arrival tolerance, including between serial segments.
        robot.reset(); robot.strictTarget = false; robot.arrivalAngleError = .1;
        auto translated = robot.actual; translated[0] += 2;
        collection::moveSelectedTcpTarget(1, "selected-probe", translated, {10, 5},
            robot.cancelled, {0, 2000, .2});
        require(robot.sends == 2 && robot.stops == 0 && collection::flangeAtTarget(robot.actual, translated, .2),
            "valid arrival residual was treated as unexpected orientation movement");
        require(!robot.inverseTargets.empty() && collection::flangeAtTarget(robot.inverseTargets.front(),
            collection::tcpTargetToFlange(translated, robot.offset)), "complete endpoint was not preflighted first");
        robot.reset(); robot.strictTarget = false; robot.arrivalAngleError = .21;
        const auto deviation = rejects([&] { collection::moveSelectedTcpTarget(1, "selected-probe", translated,
            {10, 5}, robot.cancelled, {0, 2000, .2}); });
        require(robot.sends == 1 && robot.stops == 1 && deviation.find("意外变化") != std::string::npos,
            "orientation outside arrival tolerance bypassed unexpected-motion guard");
        for (double residual : {-collection::positionToleranceMm, collection::positionToleranceMm}) {
            robot.reset(); robot.strictTarget = false; robot.arrivalPositionError = residual;
            collection::moveSelectedTcpTarget(1, "selected-probe", translated, {10, 5},
                robot.cancelled, {0, 2000});
            require(robot.sends == 2 && robot.stops == 0 && collection::flangeAtTarget(robot.actual, translated),
                "valid position residual blocked the next serial segment");
        }
        robot.reset(); robot.strictTarget = false;
        robot.arrivalPositionError = collection::positionToleranceMm + .001;
        rejects([&] { collection::moveSelectedTcpTarget(1, "selected-probe", translated,
            {10, 5}, robot.cancelled, {0, 500}); });
        require(robot.sends == 1 && robot.stops == 1,
            "position outside arrival tolerance was accepted by the next serial segment");
        std::cout << "Manual selected-TCP trajectory, transform, preflight, stop and stationary checks passed\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
