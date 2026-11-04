#include "FivePointCapture.hpp"
#include "DiagnosticLogger.hpp"
#include "utils/calib_transform.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <thread>

namespace collection {
namespace {
void check(CRresult result, const char* operation) {
    if (result != sucess)
        throw std::runtime_error(std::string(operation) + "（错误码 " + std::to_string(result) + "）");
}
void checkCancelled(const std::atomic<bool>& cancelled) {
    if (cancelled.load()) throw std::runtime_error("五点采图已停止");
}
void checkFinite(const FlangePose& pose) {
    if (!std::all_of(pose.begin(), pose.end(), [](double v) { return std::isfinite(v); }))
        throw std::runtime_error("机器人位姿包含无效数值");
}
RobotModes checkEnabled(RobotHandle handle, bool allowMoving = false) {
    RobotModes mode{};
    check(cr_get_robotMode(handle, &mode), "读取机器人状态失败");
    if (mode != ProgramStop && mode != Enable && !(allowMoving && mode == SDK_Moving))
        throw std::runtime_error("机器人须处于使能且停止状态");
    return mode;
}
void stopAndRethrow(RobotHandle handle, const std::exception& error) {
    const auto result = cr_stop(handle);
    APP_LOG_EVENT_ERROR("robot.motion", "five_point_stop", "requested",
        {{"reason", QString::fromUtf8(error.what())}, {"stop_result", QString::number(result)}});
    if (result != sucess)
        throw std::runtime_error(std::string(error.what()) + "；停止指令失败（错误码 " +
            std::to_string(result) + "），请在示教器确认机器人状态");
    throw std::runtime_error(error.what());
}
}

FivePointTargets fivePointTargets(const FlangePose& origin, double xMm, double yMm) {
    checkFinite(origin);
    for (double offset : {xMm, yMm})
        if (!std::isfinite(offset) || offset < 0.1 || offset > maximumFivePointOffsetMm)
            throw std::runtime_error("X/Y 偏移须为 0.1～50 mm");
    FivePointTargets points{origin, origin, origin, origin, origin};
    points[1][0] += xMm;
    points[2][0] -= xMm;
    points[3][1] += yMm;
    points[4][1] -= yMm;
    return points;
}

void runFivePointCapture(const FivePointTargets& targets,
    const std::function<void(const FlangePose&, int)>& move,
    const std::function<void(int)>& capture, const std::atomic<bool>& cancelled) {
    checkCancelled(cancelled);
    capture(0);
    for (int point = 1; point < 5; ++point) {
        checkCancelled(cancelled);
        move(targets[point], point);
        checkCancelled(cancelled);
        capture(point);
        checkCancelled(cancelled);
        move(targets[0], 0);
    }
    checkCancelled(cancelled);
}

FlangePose readStationaryFlange(RobotHandle handle) {
    checkEnabled(handle);
    BOOL moving = TRUE;
    check(cr_get_robotMoveStatus(handle, &moving), "读取机器人运动状态失败");
    if (moving) throw std::runtime_error("机器人尚未停止");
    FlangePose zero{}, pose{};
    check(cr_get_AssignTCP_Pose(handle, zero.data(), pose.data()), "读取法兰位姿失败");
    checkFinite(pose);
    return pose;
}

bool flangeAtTarget(const FlangePose& actual, const FlangePose& target) {
    checkFinite(actual);
    checkFinite(target);
    double distanceSquared = 0;
    for (int axis = 0; axis < 3; ++axis)
        distanceSquared += std::pow(actual[axis] - target[axis], 2);
    const cv::Mat a = calib_transform::axisAnglePoseToTransform(actual.data());
    const cv::Mat b = calib_transform::axisAnglePoseToTransform(target.data());
    const cv::Mat rotation = a(cv::Rect(0, 0, 3, 3)).t() * b(cv::Rect(0, 0, 3, 3));
    const double angle = std::acos(std::clamp((cv::trace(rotation)[0] - 1) / 2, -1.0, 1.0)) * 180 / CV_PI;
    constexpr double positionToleranceMm = 0.05;
    return distanceSquared <= positionToleranceMm * positionToleranceMm + 1e-12 && angle <= 0.2;
}

void confirmRobotStopped(RobotHandle handle) {
    const auto started = std::chrono::steady_clock::now();
    int stable = 0;
    do {
        RobotModes mode{};
        BOOL moving = TRUE;
        check(cr_get_robotMode(handle, &mode), "无法确认机器人停止状态");
        check(cr_get_robotMoveStatus(handle, &moving), "无法确认机器人运动状态");
        const bool stoppedMode = mode == Enable || mode == ProgramStop || mode == ProtectiveStop ||
            mode == Imdstop || mode == JointIdle || mode == JointPowerOff;
        stable = !moving && stoppedMode ? stable + 1 : 0;
        if (stable >= 3) return;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } while (std::chrono::steady_clock::now() - started < std::chrono::seconds(2));
    throw std::runtime_error("尚未确认机器人已停止");
}

void checkFivePointReachability(RobotHandle handle, const FivePointTargets& targets) {
    FlangePose reference{};
    check(cr_get_jointActualPos(handle, reference.data()), "读取基准关节角失败");
    checkFinite(reference);
    for (auto pose : targets) {
        FlangePose joints{};
        checkFinite(pose);
        check(cr_kineInverse(handle, pose.data(), reference.data(), joints.data()), "五点目标逆解失败");
        checkFinite(joints);
        for (int axis = 0; axis < ROB_AXIS_NUM; ++axis) {
            double delta = joints[axis] - reference[axis];
            if (axis == 5) delta = std::remainder(delta, 360.0);
            if (std::abs(delta) > 30)
                throw std::runtime_error("五点目标关节变化超过 30°，请调整基准位置或减小偏移");
        }
    }
}

void waitForStableFlange(RobotHandle handle, const FlangePose& target,
    const std::atomic<bool>& cancelled, LineMotionWait wait, MotionTick tick) {
    using Clock = std::chrono::steady_clock;
    if (wait.settlingMs < 0 || wait.timeoutMs <= wait.settlingMs)
        throw std::runtime_error("机器人等待参数无效");
    const auto started = Clock::now();
    auto stableSince = started;
    bool stable = false;
    do {
        checkCancelled(cancelled);
        const auto mode = checkEnabled(handle, true);
        BOOL moving = TRUE;
        check(cr_get_robotMoveStatus(handle, &moving), "读取机器人运动状态失败");
        FlangePose zero{}, actual{};
        check(cr_get_AssignTCP_Pose(handle, zero.data(), actual.data()), "读取实际法兰位姿失败");
        checkFinite(actual);
        if (tick) tick();
        checkCancelled(cancelled);
        const auto now = Clock::now();
        if (now - started >= std::chrono::milliseconds(wait.timeoutMs))
            throw std::runtime_error("机器人到位或稳定等待超时");
        if (!moving && mode != SDK_Moving && flangeAtTarget(actual, target)) {
            if (!stable) stableSince = now;
            stable = true;
            if (now - stableSince >= std::chrono::milliseconds(wait.settlingMs)) return;
        } else {
            stable = false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } while (Clock::now() - started < std::chrono::milliseconds(wait.timeoutMs));
    throw std::runtime_error("机器人到位或稳定等待超时");
}

void moveFlangeLinear(RobotHandle handle, const FlangePose& target,
    const std::atomic<bool>& cancelled, LineMotionWait wait, MotionTick tick) {
    checkCancelled(cancelled);
    const auto initial = readStationaryFlange(handle);
    checkFinite(target);
    double distance = 0;
    for (int axis = 0; axis < 3; ++axis) distance += std::pow(target[axis] - initial[axis], 2);
    distance = std::sqrt(distance);
    if (distance > maximumFivePointOffsetMm + 0.2)
        throw std::runtime_error("单段直线运动超过 50 mm，未下发运动");
    auto positionOnly = initial;
    std::copy(target.begin(), target.begin() + 3, positionOnly.begin());
    if (!flangeAtTarget(positionOnly, target))
        throw std::runtime_error("五点采图不允许改变机器人姿态");
    PointControlPara command{};
    check(cr_get_jointActualPos(handle, command.jointpos), "读取参考关节角失败");
    if (!std::all_of(command.jointpos, command.jointpos + ROB_AXIS_NUM, [](double v) { return std::isfinite(v); }))
        throw std::runtime_error("参考关节角包含无效数值，未下发运动");
    std::copy(target.begin(), target.end(), command.pose);
    command.tcpID = -1; // Explicit zero tool: never inherit the active TCP.
    command.coordinateType = baseCoordinate;
    command.pointTransType = pointTransStop;
    command.poseTranType = poseTranMoveToTargetPose;
    command.motiontriggerMode = MovetriggerbyOnlyRpc;
    std::fill_n(command.speed, ROB_AXIS_NUM, 10.0); // mm/s
    std::fill_n(command.acc, ROB_AXIS_NUM, 20.0);   // mm/s²
    checkCancelled(cancelled);
    APP_LOG_EVENT_INFO("robot.motion", "five_point_line", "start",
        {{"x", QString::number(target[0], 'f', 3)}, {"y", QString::number(target[1], 'f', 3)},
         {"z", QString::number(target[2], 'f', 3)}, {"speed_mm_s", "10"}});
    try {
        check(cr_move_line(handle, command, FALSE), "直线运动指令失败");
        waitForStableFlange(handle, target, cancelled, wait, tick);
    } catch (const std::exception& error) {
        stopAndRethrow(handle, error);
    } catch (...) {
        const auto result = cr_stop(handle);
        if (result != sucess)
            throw std::runtime_error("五点采图运动异常；停止指令失败，请在示教器确认机器人状态");
        throw;
    }
}
} // namespace collection
