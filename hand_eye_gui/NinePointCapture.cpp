#include "NinePointCapture.hpp"
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
    if (cancelled.load()) throw std::runtime_error("九点采图已停止");
}
void checkFinite(const FlangePose& pose) {
    if (!std::all_of(pose.begin(), pose.end(), [](double v) { return std::isfinite(v); }))
        throw std::runtime_error("机器人位姿包含无效数值");
}
FlangePose withEulerYawOffset(const FlangePose& origin, double degrees) {
    if (degrees == 0) return origin;
    const double radians = degrees * CV_PI / 180.0;
    const double c = std::cos(radians), s = std::sin(radians);
    auto transform = calib_transform::axisAnglePoseToTransform(origin.data());
    // ZYX Euler R = Rz(yaw) Ry(pitch) Rx(roll). Pre-multiplying Rz(delta)
    // changes Euler yaw only, without extracting singular Euler angles or
    // rotating the XYZ translation about the base origin.
    const cv::Mat rotation = (cv::Mat_<double>(3, 3) << c, -s, 0, s, c, 0, 0, 0, 1) *
        transform(cv::Rect(0, 0, 3, 3));
    rotation.copyTo(transform(cv::Rect(0, 0, 3, 3)));
    FlangePose result = origin;
    calib_transform::transformToAxisAnglePose(transform, result.data());
    std::copy_n(origin.begin(), 3, result.begin());
    checkFinite(result);
    return result;
}
void checkGridPose(const FlangePose& origin, const FlangePose& pose) {
    checkFinite(origin);
    checkFinite(pose);
    if (std::abs(pose[2] - origin[2]) > 0.05 ||
        std::hypot(pose[0] - origin[0], pose[1] - origin[1]) > maximumNinePointOffsetMm + 0.05)
        throw std::runtime_error("九点目标须保持基准 Z 高度，距基准不超过 50 mm");
    const auto base = calib_transform::axisAnglePoseToTransform(origin.data());
    const auto actual = calib_transform::axisAnglePoseToTransform(pose.data());
    const cv::Mat relative = actual(cv::Rect(0, 0, 3, 3)) * base(cv::Rect(0, 0, 3, 3)).t();
    const double yaw = std::atan2(relative.at<double>(1, 0), relative.at<double>(0, 0)) * 180 / CV_PI;
    auto expected = withEulerYawOffset(origin, yaw);
    std::copy_n(pose.begin(), 3, expected.begin());
    if (std::abs(yaw) > maximumNinePointYawDegrees + 0.2 || !flangeAtTarget(pose, expected))
        throw std::runtime_error("九点采图仅允许欧拉 Rz 相对基准变化 ±20°，Rx/Ry 保持不变");
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
    APP_LOG_EVENT_ERROR("robot.motion", "nine_point_stop", "requested",
        {{"reason", QString::fromUtf8(error.what())}, {"stop_result", QString::number(result)}});
    if (result != sucess)
        throw std::runtime_error(std::string(error.what()) + "；停止指令失败（错误码 " +
            std::to_string(result) + "），请在示教器确认机器人状态");
    throw std::runtime_error(error.what());
}
}

NinePointTargets ninePointTargets(const FlangePose& origin, double xMm, double yMm,
    const NinePointAngles& yawOffsets) {
    checkFinite(origin);
    for (double offset : {xMm, yMm})
        if (!std::isfinite(offset) || offset < 0.1 || offset > maximumNinePointOffsetMm)
            throw std::runtime_error("X/Y 偏移须为 0.1～50 mm");
    if (std::hypot(xMm, yMm) > maximumNinePointOffsetMm)
        throw std::runtime_error("对角点距基准不得超过 50 mm，请减小 X/Y 偏移");
    for (double angle : yawOffsets)
        if (!std::isfinite(angle) || std::abs(angle) > maximumNinePointYawDegrees ||
            (angle != 0 && std::abs(angle) < 0.01))
            throw std::runtime_error("九个 Rz 角度须为 −20°～+20° 的有效数值，非零角度至少 0.01°");
    NinePointTargets points{};
    for (int point = 0; point < static_cast<int>(points.size()); ++point) {
        points[point] = withEulerYawOffset(origin, yawOffsets[point]);
        points[point][0] += (point % 3 - 1) * xMm;
        points[point][1] += (1 - point / 3) * yMm;
        checkFinite(points[point]);
    }
    return points;
}

void runNinePointCapture(const FlangePose& origin, const NinePointTargets& targets,
    const std::function<void(const FlangePose&, int)>& move,
    const std::function<void(int)>& capture, const std::atomic<bool>& cancelled) {
    checkCancelled(cancelled);
    for (int point = 0; point < static_cast<int>(targets.size()); ++point) {
        checkCancelled(cancelled);
        const bool displaced = targets[point] != origin;
        if (displaced) move(targets[point], point);
        checkCancelled(cancelled);
        capture(point);
        checkCancelled(cancelled);
        if (displaced) move(origin, -1);
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

bool flangeAtTarget(const FlangePose& actual, const FlangePose& target, double angleToleranceDeg) {
    checkFinite(actual);
    checkFinite(target);
    double distanceSquared = 0;
    for (int axis = 0; axis < 3; ++axis)
        distanceSquared += std::pow(actual[axis] - target[axis], 2);
    const cv::Mat a = calib_transform::axisAnglePoseToTransform(actual.data());
    const cv::Mat b = calib_transform::axisAnglePoseToTransform(target.data());
    const cv::Mat rotation = a(cv::Rect(0, 0, 3, 3)).t() * b(cv::Rect(0, 0, 3, 3));
    const double angle = std::acos(std::clamp((cv::trace(rotation)[0] - 1) / 2, -1.0, 1.0)) * 180 / CV_PI;
    return distanceSquared <= positionToleranceMm * positionToleranceMm + 1e-12 && angle <= angleToleranceDeg;
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

void checkNinePointReachability(RobotHandle handle, const NinePointTargets& targets) {
    FlangePose reference{};
    check(cr_get_jointActualPos(handle, reference.data()), "读取基准关节角失败");
    checkFinite(reference);
    for (auto pose : targets) {
        FlangePose joints{};
        checkFinite(pose);
        check(cr_kineInverse(handle, pose.data(), reference.data(), joints.data()), "九点目标逆解失败");
        checkFinite(joints);
        for (int axis = 0; axis < ROB_AXIS_NUM; ++axis) {
            double delta = joints[axis] - reference[axis];
            if (axis == 5) delta = std::remainder(delta, 360.0);
            if (std::abs(delta) > 30)
                throw std::runtime_error("九点目标关节变化超过 30°，请调整基准位置或减小偏移");
        }
    }
}

void waitForStableFlange(RobotHandle handle, const FlangePose& target,
    const std::atomic<bool>& cancelled, LineMotionWait wait, MotionTick tick) {
    using Clock = std::chrono::steady_clock;
    if (wait.settlingMs < 0 || wait.timeoutMs <= wait.settlingMs ||
        !std::isfinite(wait.angleToleranceDeg) || wait.angleToleranceDeg <= 0 || wait.angleToleranceDeg > 0.2)
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
        if (!moving && mode != SDK_Moving && flangeAtTarget(actual, target, wait.angleToleranceDeg)) {
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

void moveFlangeLinear(RobotHandle handle, const FlangePose& origin, const FlangePose& target,
    const std::atomic<bool>& cancelled, LineMotionWait wait, MotionTick tick, double speedMmS) {
    checkCancelled(cancelled);
    if (!validRobotSpeed(speedMmS)) throw std::runtime_error("机器人移动速度超出 1～1000 mm/s，未下发运动");
    const auto initial = readStationaryFlange(handle);
    checkGridPose(origin, initial);
    checkGridPose(origin, target);
    double distance = 0;
    for (int axis = 0; axis < 3; ++axis) distance += std::pow(target[axis] - initial[axis], 2);
    distance = std::sqrt(distance);
    if (distance > maximumNinePointOffsetMm + 0.2)
        throw std::runtime_error("单段直线运动超过 50 mm，未下发运动");
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
    std::fill_n(command.speed, ROB_AXIS_NUM, speedMmS); // mm/s
    std::fill_n(command.acc, ROB_AXIS_NUM, 20.0);   // mm/s²
    checkCancelled(cancelled);
    APP_LOG_EVENT_INFO("robot.motion", "nine_point_line", "start",
        {{"x", QString::number(target[0], 'f', 3)}, {"y", QString::number(target[1], 'f', 3)},
         {"z", QString::number(target[2], 'f', 3)}, {"rx", QString::number(target[3], 'f', 6)},
         {"ry", QString::number(target[4], 'f', 6)}, {"rz", QString::number(target[5], 'f', 6)},
         {"speed_mm_s", QString::number(speedMmS)}});
    try {
        check(cr_move_line(handle, command, FALSE), "直线运动指令失败");
        waitForStableFlange(handle, target, cancelled, wait, tick);
    } catch (const std::exception& error) {
        stopAndRethrow(handle, error);
    } catch (...) {
        const auto result = cr_stop(handle);
        if (result != sucess)
            throw std::runtime_error("九点采图运动异常；停止指令失败，请在示教器确认机器人状态");
        throw;
    }
}
} // namespace collection
