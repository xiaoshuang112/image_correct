#include "TeachMotion.hpp"
#include "utils/calib_transform.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>

namespace collection {
namespace {
constexpr double eulerPitchLimit = 89.0;
std::int64_t nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
void finite(const FlangePose& pose) {
    if (!std::all_of(pose.begin(), pose.end(), [](double v) { return std::isfinite(v); }))
        throw std::runtime_error("TCP 位姿包含无效数值");
}
void representation(TeachOrientation orientation) {
    if (orientation != TeachOrientation::RotationVector && orientation != TeachOrientation::EulerZYX)
        throw std::runtime_error("TCP 姿态表示无效");
}
void eulerSafe(const FlangePose& pose) {
    if (std::abs(pose[4]) >= eulerPitchLimit)
        throw std::runtime_error("ZYX 欧拉俯仰角接近 ±90° 奇异点，请停止后使用旋转向量模式");
}
void rotationVectorSafe(const FlangePose& pose) {
    const double length = std::hypot(pose[3], pose[4], pose[5]);
    // Continue through pi on the original numeric branch. At 2*pi the rotation
    // vector chart itself becomes singular for transverse component changes.
    if (length >= 359.0)
        throw std::runtime_error("旋转向量接近 360° 表示奇异点，请停止后重新确认姿态");
}
void speedsChecked(const TeachSpeeds& speeds) {
    if (!validTeachSpeeds(speeds))
        throw std::runtime_error("示教速度无效：线速度须为 1～1000 mm/s，角速度须为 0.1～180 °/s");
}
} // namespace

bool validTeachSpeeds(const TeachSpeeds& speeds) {
    return std::isfinite(speeds.linearMmS) && validRobotSpeed(speeds.linearMmS) &&
        std::isfinite(speeds.angularDegS) && speeds.angularDegS >= 0.1 &&
        speeds.angularDegS <= maximumTeachAngularDegS;
}

FlangePose tcpPoseForDisplay(const FlangePose& axisAngle, TeachOrientation orientation) {
    finite(axisAngle);
    representation(orientation);
    if (orientation == TeachOrientation::RotationVector) return axisAngle;
    const cv::Mat transform = calib_transform::axisAnglePoseToTransform(axisAngle.data());
    FlangePose result = axisAngle;
    result[4] = std::asin(std::clamp(-transform.at<double>(2, 0), -1.0, 1.0)) * 180 / CV_PI;
    // Do not return a made-up roll/yaw pair when the decomposition is unstable.
    eulerSafe({0, 0, 0, 0, result[4], 0});
    result[3] = std::atan2(transform.at<double>(2, 1), transform.at<double>(2, 2)) * 180 / CV_PI;
    result[5] = std::atan2(transform.at<double>(1, 0), transform.at<double>(0, 0)) * 180 / CV_PI;
    return result;
}

FlangePose tcpPoseFromDisplay(const FlangePose& displayed, TeachOrientation orientation) {
    finite(displayed);
    representation(orientation);
    if (orientation == TeachOrientation::RotationVector) return displayed;
    eulerSafe(displayed);
    const double roll = displayed[3] * CV_PI / 180;
    const double pitch = displayed[4] * CV_PI / 180;
    const double yaw = displayed[5] * CV_PI / 180;
    const double cr = std::cos(roll), sr = std::sin(roll);
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    cv::Mat transform = cv::Mat::eye(4, 4, CV_64F);
    const cv::Mat rotation = (cv::Mat_<double>(3, 3) <<
        cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr,
        sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr,
        -sp, cp*sr, cp*cr);
    rotation.copyTo(transform(cv::Rect(0, 0, 3, 3)));
    for (int i = 0; i < 3; ++i) transform.at<double>(i, 3) = displayed[i] / 1000;
    FlangePose result{};
    calib_transform::transformToAxisAnglePose(transform, result.data());
    finite(result);
    return result;
}

FlangePose teachJogTarget(const FlangePose& displayed, TeachOrientation orientation,
    int component, double increment) {
    finite(displayed);
    representation(orientation);
    if (component < 0 || component >= ROB_AXIS_NUM || !std::isfinite(increment))
        throw std::runtime_error("示教分量或增量无效");
    auto next = displayed;
    next[component] += increment;
    finite(next);
    if (component >= 3) {
        if (orientation == TeachOrientation::EulerZYX) { eulerSafe(displayed); eulerSafe(next); }
        else { rotationVectorSafe(displayed); rotationVectorSafe(next); }
    }
    return tcpPoseFromDisplay(next, orientation);
}

TeachJogLease::TeachJogLease() : heartbeatMs_(nowMs()) {}
void TeachJogLease::heartbeat() {
    if (released_.load()) return;
    const auto now = nowMs();
    if (now - heartbeatMs_.load() >= expiryMs) {
        released_.store(true);
        return;
    }
    heartbeatMs_.store(now);
}
void TeachJogLease::release() { released_.store(true); }
bool TeachJogLease::valid() const {
    return !released_.load() && nowMs() - heartbeatMs_.load() < expiryMs;
}

namespace {
constexpr double angularStepDeg = 0.1;
constexpr double rotatingSdkSpeedMmS = 1.0;
FlangePose continuousVector(FlangePose pose, const FlangePose& reference) {
    const double length = std::hypot(pose[3], pose[4], pose[5]);
    if (length < 1e-9) return pose;
    auto best = pose;
    double bestDistance = std::numeric_limits<double>::infinity();
    for (int turns = -2; turns <= 2; ++turns) {
        auto candidate = pose;
        for (int i = 3; i < 6; ++i) candidate[i] = pose[i] * (length + 360 * turns) / length;
        double squared = 0;
        for (int i = 3; i < 6; ++i) squared += std::pow(candidate[i] - reference[i], 2);
        if (squared < bestDistance) { best = candidate; bestDistance = squared; }
    }
    return best;
}
void demandControl(const std::atomic<bool>& cancelled, const TeachJogLease* lease) {
    if (cancelled.load() || (lease && !lease->valid()))
        throw std::runtime_error("示教已停止、按压已释放或心跳已过期");
}
double orientationDistance(const FlangePose& a, const FlangePose& b) {
    const auto ta = calib_transform::axisAnglePoseToTransform(a.data());
    const auto tb = calib_transform::axisAnglePoseToTransform(b.data());
    const double chord = cv::norm(ta(cv::Rect(0, 0, 3, 3)) - tb(cv::Rect(0, 0, 3, 3)));
    return 2 * std::asin(std::clamp(chord / (2 * std::sqrt(2.0)), 0.0, 1.0)) * 180 / CV_PI;
}
double distance(const FlangePose& a, const FlangePose& b, int first) {
    return std::hypot(a[first] - b[first], a[first + 1] - b[first + 1], a[first + 2] - b[first + 2]);
}
TcpPose freshPose(RobotHandle handle, const QString& name, const std::atomic<bool>& cancelled,
    const TeachJogLease* lease) {
    demandControl(cancelled, lease);
    const auto started = nowMs();
    auto actual = readTcpPose(handle, name);
    demandControl(cancelled, lease);
    if (nowMs() - started > 100) throw std::runtime_error("TCP 状态读取过慢，请检查机器人连接后重试");
    return actual;
}
void paceUntil(std::chrono::steady_clock::time_point due, const std::atomic<bool>& cancelled,
    const TeachJogLease* lease, const MotionTick& tick) {
    // Wait after each completed segment, never accumulate an overdue backlog.
    while (std::chrono::steady_clock::now() < due) {
        demandControl(cancelled, lease);
        if (tick) tick();
        demandControl(cancelled, lease);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    demandControl(cancelled, lease);
}
void segment(RobotHandle handle, const QString& name, const TcpPose& origin,
    const FlangePose& before, const FlangePose& target, const TeachSpeeds& speeds,
    const std::atomic<bool>& cancelled, const TeachJogLease* lease,
    LineMotionWait wait, const MotionTick& tick) {
    const double angularTravel = orientationDistance(before, target);
    if (angularTravel > 0.100001 || distance(before, target, 0) > 1.000001 ||
        distance(before, target, 3) > 1.0)
        throw std::runtime_error("示教目标超出单步连续运动范围");
    if (angularTravel > 1e-7) wait.angleToleranceDeg = std::min(wait.angleToleranceDeg,
        std::max(0.00001, angularTravel / 4));
    auto guard = [&] {
        const auto actual = freshPose(handle, name, cancelled, lease);
        if (actual.offset != origin.offset) throw std::runtime_error("所选 TCP 工具偏移已变化，请重新开始示教");
        // Reject an external move rather than turn a nominal small segment into
        // a distant target. Pose tolerances are the shared completion tolerances.
        if (distance(actual.pose, target, 0) > 1.1 ||
            orientationDistance(actual.pose, target) > angularTravel + 0.02)
            throw std::runtime_error("TCP 位置或姿态发生意外变化，请停止后重新开始示教");
    };
    moveSelectedTcpLinear(handle, name, target, cancelled, wait, tick,
        angularTravel > 1e-9 ? std::min(speeds.linearMmS, rotatingSdkSpeedMmS) : speeds.linearMmS, guard);
}
} // namespace

void moveSelectedTcpTarget(RobotHandle handle, const QString& name, const FlangePose& target,
    const TeachSpeeds& speeds, const std::atomic<bool>& cancelled, LineMotionWait wait,
    MotionTick tick, TeachOrientation orientation) {
    speedsChecked(speeds);
    finite(target);
    representation(orientation);
    if (wait.settlingMs < 0 || wait.timeoutMs <= wait.settlingMs ||
        !std::isfinite(wait.angleToleranceDeg) || wait.angleToleranceDeg <= 0 || wait.angleToleranceDeg > 0.2)
        throw std::runtime_error("机器人等待参数无效");
    const auto origin = freshPose(handle, name, cancelled, nullptr);
    const auto start = tcpPoseForDisplay(origin.pose, orientation);
    auto finish = tcpPoseForDisplay(target, orientation);
    if (orientation == TeachOrientation::EulerZYX) {
        for (int i : {3, 5}) finish[i] += 360 * std::round((start[i] - finish[i]) / 360);
    } else finish = continuousVector(finish, start);
    const double linear = distance(start, finish, 0), angular = distance(start, finish, 3);
    if (angular > 1e-9) {
        if (orientation == TeachOrientation::RotationVector) { rotationVectorSafe(start); rotationVectorSafe(finish); }
        else {
            eulerSafe(start); eulerSafe(finish);
        }
    }
    // The sum bounds physical rotation for a simultaneous change in three
    // Euler or rotation-vector parameters, including near Euler gimbal lock.
    double angularBound = 0;
    for (int i = 3; i < 6; ++i) angularBound += std::abs(finish[i] - start[i]);
    const double countValue = std::max({1.0, std::ceil(linear - 1e-9), std::ceil(angularBound / angularStepDeg - 1e-9)});
    if (countValue > 1000000) throw std::runtime_error("TCP 目标距离过大");
    const int count = static_cast<int>(countValue);
    const auto operationStarted = std::chrono::steady_clock::now();
    const auto deadline = operationStarted + std::chrono::milliseconds(wait.timeoutMs);
    auto before = origin.pose;
    for (int index = 1; index <= count; ++index) {
        demandControl(cancelled, nullptr);
        const auto began = std::chrono::steady_clock::now();
        if (began >= deadline) throw std::runtime_error("TCP 目标移动总时间超时");
        FlangePose displayed{};
        for (int i = 0; i < 6; ++i) displayed[i] = start[i] + (finish[i] - start[i]) * index / count;
        const auto next = continuousVector(tcpPoseFromDisplay(displayed, orientation), before);
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - began).count();
        LineMotionWait segmentWait{index == count ? wait.settlingMs : 0,
            static_cast<int>(std::min<std::int64_t>(remaining, 3000 + wait.settlingMs)), wait.angleToleranceDeg};
        if (segmentWait.timeoutMs <= segmentWait.settlingMs) throw std::runtime_error("TCP 目标移动总时间超时");
        segment(handle, name, origin, before, next, speeds, cancelled, nullptr, segmentWait, tick);
        if (std::chrono::steady_clock::now() >= deadline) throw std::runtime_error("TCP 目标移动总时间超时");
        const double duration = std::max(linear / speeds.linearMmS, angularBound / speeds.angularDegS) / count;
        const auto due = began + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(duration));
        if (due >= deadline) throw std::runtime_error("TCP 目标移动总时间超时");
        paceUntil(due, cancelled, nullptr, tick);
        if (std::chrono::steady_clock::now() >= deadline) throw std::runtime_error("TCP 目标移动总时间超时");
        before = next;
    }
}

void jogSelectedTcp(RobotHandle handle, const QString& name, const TeachJogRequest& request,
    const TeachJogLease& lease, const std::atomic<bool>& cancelled, MotionTick tick) {
    speedsChecked(request.speeds);
    representation(request.orientation);
    if (request.component < 0 || request.component >= ROB_AXIS_NUM ||
        (request.direction != -1 && request.direction != 1))
        throw std::runtime_error("示教方向或分量无效");
    if (!lease.valid() || cancelled.load()) return;
    const auto origin = freshPose(handle, name, cancelled, &lease);
    const bool rotating = request.component >= 3;
    // Translation does not require Euler decomposition, including at gimbal lock.
    auto displayed = rotating ? tcpPoseForDisplay(origin.pose, request.orientation) : origin.pose;
    const double speed = rotating ? request.speeds.angularDegS : request.speeds.linearMmS;
    const double step = request.direction * std::min(speed * 0.1, rotating ? angularStepDeg : 1.0);
    auto before = origin.pose;
    while (lease.valid() && !cancelled.load()) {
        const auto began = std::chrono::steady_clock::now();
        auto next = rotating ? continuousVector(teachJogTarget(displayed, request.orientation, request.component, step), before) : before;
        if (!rotating) next[request.component] += step;
        segment(handle, name, origin, before, next, request.speeds, cancelled, &lease, {0, 3000, 0.005}, tick);
        const auto due = began + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(std::abs(step) / speed));
        paceUntil(due, cancelled, &lease, tick);
        displayed[request.component] += step;
        before = next;
    }
}
} // namespace collection
