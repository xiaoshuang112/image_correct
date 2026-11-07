#include "ManualTcpMotion.hpp"
#include "DiagnosticLogger.hpp"
#include "utils/calib_transform.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>

namespace collection {
namespace {
void check(CRresult code, const char* action) {
    if (code != sucess)
        throw std::runtime_error(std::string(action) + "（错误码 " + std::to_string(code) + "）");
}
void finite(const FlangePose& pose) {
    if (!std::all_of(pose.begin(), pose.end(), [](double v) { return std::isfinite(v); }))
        throw std::runtime_error("TCP 目标、工具偏移或机器人状态包含无效数值");
}
void cancellation(const std::atomic<bool>& cancelled) {
    if (cancelled.load()) throw std::runtime_error("手动 TCP 运动已停止");
}
RobotModes enabled(RobotHandle handle, bool movingAllowed) {
    RobotModes mode{};
    check(cr_get_robotMode(handle, &mode), "读取机器人状态失败");
    if (mode != Enable && mode != ProgramStop && !(movingAllowed && mode == SDK_Moving))
        throw std::runtime_error("机器人须处于使能且停止状态");
    return mode;
}
void stationary(RobotHandle handle) {
    enabled(handle, false);
    BOOL moving = TRUE;
    check(cr_get_robotMoveStatus(handle, &moving), "读取机器人运动状态失败");
    if (moving) throw std::runtime_error("机器人尚未停止");
}
void checkJointLimits(RobotHandle handle, const FlangePose& current, const FlangePose& target) {
    for (int axis = 0; axis < ROB_AXIS_NUM; ++axis) {
        // SDK Copy_JointLimit indexes 0..5. No API exposes the active safety mode;
        // require both configured envelopes so a reduced-mode transition is safe.
        for (auto mode : {mode_Normal, mode_Reduced}) {
            SafetyLimitsJointAngle limit{};
            limit.minJointPosition = limit.maxJointPosition = std::numeric_limits<double>::quiet_NaN();
            check(cr_cfg_safety_joint_limit_get(handle, mode, axis, &limit), "读取关节安全限位失败");
            if (!std::isfinite(limit.minJointPosition) || !std::isfinite(limit.maxJointPosition) ||
                limit.minJointPosition >= limit.maxJointPosition)
                throw std::runtime_error("机器人关节安全限位无效，未下发运动");
            for (double value : {current[axis], target[axis]})
                if (value < limit.minJointPosition || value > limit.maxJointPosition)
                    throw std::runtime_error("当前或目标关节角超出安全限位，未下发运动");
        }
    }
}
struct TcpTargetCheck {
    TcpPose selected;
    FlangePose reference;
};
TcpTargetCheck preflight(RobotHandle handle, const QString& name, const FlangePose& target,
    const std::atomic<bool>& cancelled) {
    cancellation(cancelled);
    finite(target);
    stationary(handle);
    TcpTargetCheck result{readTcpPose(handle, name), {}};
    auto flangeTarget = tcpTargetToFlange(target, result.selected.offset);
    FlangePose targetJoints{};
    result.reference.fill(std::numeric_limits<double>::quiet_NaN());
    targetJoints.fill(std::numeric_limits<double>::quiet_NaN());
    check(cr_get_jointActualPos(handle, result.reference.data()), "读取参考关节角失败");
    finite(result.reference);
    check(cr_kineInverse(handle, flangeTarget.data(), result.reference.data(), targetJoints.data()), "TCP 目标逆解失败");
    finite(targetJoints);
    checkJointLimits(handle, result.reference, targetJoints);
    cancellation(cancelled);
    return result;
}
void waitSelected(RobotHandle handle, const QString& name, const FlangePose& offset,
    const FlangePose& target, const std::atomic<bool>& cancelled, LineMotionWait wait,
    const MotionTick& tick) {
    using Clock = std::chrono::steady_clock;
    const auto started = Clock::now();
    auto stableSince = started;
    bool stable = false;
    do {
        cancellation(cancelled);
        const auto mode = enabled(handle, true);
        BOOL moving = TRUE;
        check(cr_get_robotMoveStatus(handle, &moving), "读取机器人运动状态失败");
        const auto actual = readTcpPose(handle, name);
        if (actual.offset != offset)
            throw std::runtime_error("所选 TCP 工具偏移在运动中发生变化");
        if (tick) tick();
        cancellation(cancelled);
        const auto now = Clock::now();
        if (now - started >= std::chrono::milliseconds(wait.timeoutMs))
            throw std::runtime_error("所选 TCP 到位或稳定等待超时");
        if (!moving && mode != SDK_Moving && flangeAtTarget(actual.pose, target, wait.angleToleranceDeg)) {
            if (!stable) stableSince = now;
            stable = true;
            if (now - stableSince >= std::chrono::milliseconds(wait.settlingMs)) return;
        } else stable = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } while (Clock::now() - started < std::chrono::milliseconds(wait.timeoutMs));
    throw std::runtime_error("所选 TCP 到位或稳定等待超时");
}
} // namespace

FlangePose tcpTargetToFlange(const FlangePose& target, const FlangePose& toolOffset) {
    finite(target);
    finite(toolOffset);
    const cv::Mat flange = calib_transform::axisAnglePoseToTransform(target.data()) *
        calib_transform::invertTransform(calib_transform::axisAnglePoseToTransform(toolOffset.data()));
    FlangePose result{};
    calib_transform::transformToAxisAnglePose(flange, result.data());
    finite(result);
    return result;
}

void checkSelectedTcpTarget(RobotHandle handle, const QString& name, const FlangePose& target,
    const std::atomic<bool>& cancelled) {
    preflight(handle, name, target, cancelled);
}

void moveSelectedTcpLinear(RobotHandle handle, const QString& name, const FlangePose& target,
    const std::atomic<bool>& cancelled, LineMotionWait wait, MotionTick tick, double speedMmS,
    MotionTick safetyCheck) {
    cancellation(cancelled);
    if (safetyCheck) safetyCheck();
    if (!validRobotSpeed(speedMmS)) throw std::runtime_error("机器人移动速度超出 1～1000 mm/s，未下发运动");
    finite(target);
    if (wait.settlingMs < 0 || wait.timeoutMs <= wait.settlingMs ||
        !std::isfinite(wait.angleToleranceDeg) || wait.angleToleranceDeg <= 0 || wait.angleToleranceDeg > 0.2)
        throw std::runtime_error("机器人等待参数无效");
    const auto checked = preflight(handle, name, target, cancelled);
    stationary(handle);
    // Refresh immediately before motion; configuration changes invalidate the IK.
    const auto fresh = readTcpPose(handle, name);
    if (fresh.offset != checked.selected.offset)
        throw std::runtime_error("所选 TCP 工具偏移已变化，请重新确认目标");
    if (!flangeAtTarget(fresh.pose, checked.selected.pose))
        throw std::runtime_error("机器人位置在运动准备期间发生变化，请重新确认目标");
    PointControlPara command{};
    std::copy(target.begin(), target.end(), command.pose);
    std::copy(checked.reference.begin(), checked.reference.end(), command.jointpos);
    std::copy(fresh.offset.begin(), fresh.offset.end(), command.tcpOffset);
    // SDK v2.50c/d: negative ID explicitly selects tcpOffset, never active TCP.
    command.tcpID = -1;
    command.coordinateType = baseCoordinate;
    command.pointTransType = pointTransStop;
    command.poseTranType = poseTranMoveToTargetPose;
    command.motiontriggerMode = MovetriggerbyOnlyRpc;
    std::fill_n(command.speed, ROB_AXIS_NUM, speedMmS);
    std::fill_n(command.acc, ROB_AXIS_NUM, 20.0);
    cancellation(cancelled);
    if (safetyCheck) safetyCheck();
    APP_LOG_EVENT_INFO("robot.motion", "manual_tcp_line", "start",
        {{"tcp", name}, {"speed_mm_s", QString::number(speedMmS)}});
    try {
        check(cr_move_line(handle, command, FALSE), "TCP 直线运动指令失败");
        waitSelected(handle, name, fresh.offset, target, cancelled, wait, [&] {
            if (safetyCheck) safetyCheck();
            if (tick) tick();
            if (safetyCheck) safetyCheck();
        });
    } catch (const std::exception& error) {
        // cr_stop stops Lua programs; SDK-dispatched moves use motion control.
        const auto stop = cr_moveControl(handle, DecStop);
        APP_LOG_EVENT_ERROR("robot.motion", "manual_tcp_stop", "requested",
            {{"reason", QString::fromUtf8(error.what())}, {"stop_result", QString::number(stop)}});
        if (stop != sucess)
            throw std::runtime_error(std::string(error.what()) + "；停止指令失败（错误码 " +
                std::to_string(stop) + "），请在示教器确认机器人状态");
        throw;
    } catch (...) {
        const auto stop = cr_moveControl(handle, DecStop);
        if (stop != sucess)
            throw std::runtime_error("TCP 运动异常；停止指令失败，请在示教器确认机器人状态");
        throw;
    }
}
} // namespace collection
