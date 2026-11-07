#include "ManualTcpMotion.hpp"
#include "DiagnosticLogger.hpp"
#include "utils/calib_transform.hpp"
#include <QStringList>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <thread>

namespace collection {
namespace {
using ControlCheck = std::function<void(const char*)>;
using MotionClock = std::chrono::steady_clock;
QString poseText(const std::optional<FlangePose>& pose) {
    if (!pose) return "unavailable";
    QStringList values;
    for (double value : *pose) values << QString::number(value, 'g', 17);
    return values.join(',');
}
struct ArrivalSample {
    std::optional<int> mode, moving;
    std::optional<FlangePose> pose;
    std::optional<MotionClock::time_point> returned;
    qint64 poseReadMs = -1, sampleReadMs = -1;
};
struct ArrivalDiagnostics {
    ArrivalSample current, lastComplete;
    int polls = 0;
    qint64 stableObservedMs = -1;
};
QString measuredMs(qint64 value) {
    return value < 0 ? QString("unavailable") : QString::number(value);
}
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
RobotModes enabled(RobotHandle handle, bool movingAllowed, const ControlCheck& control,
    std::optional<int>* observedMode = nullptr) {
    control("robot_mode");
    RobotModes mode{};
    const auto code = cr_get_robotMode(handle, &mode);
    if (code == sucess && observedMode) *observedMode = mode;
    control("robot_mode");
    check(code, "读取机器人状态失败");
    if (mode != Enable && mode != ProgramStop && !(movingAllowed && mode == SDK_Moving))
        throw std::runtime_error("机器人须处于使能且停止状态");
    return mode;
}
void stationary(RobotHandle handle, const ControlCheck& control) {
    enabled(handle, false, control);
    BOOL moving = TRUE;
    const auto code = cr_get_robotMoveStatus(handle, &moving);
    control("stationary_status");
    check(code, "读取机器人运动状态失败");
    if (moving) throw std::runtime_error("机器人尚未停止");
}
void checkJointLimits(RobotHandle handle, const FlangePose& current, const FlangePose& target,
    const ControlCheck& control) {
    for (int axis = 0; axis < ROB_AXIS_NUM; ++axis) {
        // SDK Copy_JointLimit indexes 0..5. No API exposes the active safety mode;
        // require both configured envelopes so a reduced-mode transition is safe.
        for (auto mode : {mode_Normal, mode_Reduced}) {
            SafetyLimitsJointAngle limit{};
            limit.minJointPosition = limit.maxJointPosition = std::numeric_limits<double>::quiet_NaN();
            const auto code = cr_cfg_safety_joint_limit_get(handle, mode, axis, &limit);
            control("joint_limit");
            check(code, "读取关节安全限位失败");
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
    const ControlCheck& control, const TcpPoseCheck& poseCheck = {}) {
    control("preflight");
    finite(target);
    stationary(handle, control);
    TcpTargetCheck result{readTcpPose(handle, name), {}};
    control("preflight_pose");
    if (poseCheck) poseCheck(result.selected);
    auto flangeTarget = tcpTargetToFlange(target, result.selected.offset);
    FlangePose targetJoints{};
    result.reference.fill(std::numeric_limits<double>::quiet_NaN());
    targetJoints.fill(std::numeric_limits<double>::quiet_NaN());
    const auto jointCode = cr_get_jointActualPos(handle, result.reference.data());
    control("reference_joints");
    check(jointCode, "读取参考关节角失败");
    finite(result.reference);
    const auto inverseCode = cr_kineInverse(handle, flangeTarget.data(), result.reference.data(), targetJoints.data());
    control("inverse_kinematics");
    check(inverseCode, "TCP 目标逆解失败");
    finite(targetJoints);
    checkJointLimits(handle, result.reference, targetJoints, control);
    control("preflight_complete");
    return result;
}
void waitSelected(RobotHandle handle, const QString& name, const FlangePose& offset,
    const FlangePose& target, LineMotionWait wait, const MotionTick& tick,
    const ControlCheck& control, const TcpPoseCheck& poseCheck, ArrivalDiagnostics& diagnostics) {
    using Clock = std::chrono::steady_clock;
    auto stableSince = Clock::now();
    bool stable = false;
    for (;;) {
        diagnostics.current = {};
        control("before_tick");
        if (tick) tick();
        control("after_tick");
        // Tick may itself read telemetry or take time. Read every state used
        // for arrival after tick, so its older snapshot cannot announce arrival.
        const auto sampleStarted = Clock::now();
        ++diagnostics.polls;
        const auto mode = enabled(handle, true, control, &diagnostics.current.mode);
        BOOL moving = TRUE;
        const auto code = cr_get_robotMoveStatus(handle, &moving);
        if (code == sucess) diagnostics.current.moving = moving;
        control("arrival_status");
        check(code, "读取机器人运动状态失败");
        const auto poseStarted = Clock::now();
        const auto actual = readTcpPose(handle, name);
        // Save the returned sample before enforcing the deadline. It remains
        // diagnostic evidence only when control rejects a late response.
        diagnostics.current.pose = actual.pose;
        diagnostics.current.returned = Clock::now();
        diagnostics.current.poseReadMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            *diagnostics.current.returned - poseStarted).count();
        diagnostics.current.sampleReadMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            *diagnostics.current.returned - sampleStarted).count();
        control("arrival_pose");
        if (actual.offset != offset)
            throw std::runtime_error("所选 TCP 工具偏移在运动中发生变化");
        if (poseCheck) poseCheck(actual);
        control("arrival_check");
        diagnostics.lastComplete = diagnostics.current;
        const auto now = Clock::now();
        if (!moving && mode != SDK_Moving && flangeAtTarget(actual.pose, target, wait.angleToleranceDeg)) {
            if (!stable) stableSince = now;
            stable = true;
            diagnostics.stableObservedMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - stableSince).count();
            if (now - stableSince >= std::chrono::milliseconds(wait.settlingMs)) return;
        } else {
            stable = false;
            diagnostics.stableObservedMs = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
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

void moveSelectedTcpLinear(RobotHandle handle, const QString& name, const FlangePose& target,
    const std::atomic<bool>& cancelled, LineMotionWait wait, MotionTick tick, double speedMmS,
    MotionTick controlCheck, TcpPoseCheck poseCheck) {
    using Clock = std::chrono::steady_clock;
    const auto started = Clock::now();
    cancellation(cancelled);
    if (!validRobotSpeed(speedMmS)) throw std::runtime_error("机器人移动速度超出 1～1000 mm/s，未下发运动");
    finite(target);
    if (wait.settlingMs < 0 || wait.timeoutMs <= wait.settlingMs ||
        !std::isfinite(wait.angleToleranceDeg) || wait.angleToleranceDeg <= 0 || wait.angleToleranceDeg > 0.2)
        throw std::runtime_error("机器人等待参数无效");
    ArrivalDiagnostics diagnostics;
    std::optional<Clock::time_point> dispatched;
    auto control = [&](const char* phase) {
        cancellation(cancelled);
        if (controlCheck) controlCheck();
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - started).count();
        if (elapsed >= wait.timeoutMs) {
            const auto& sample = diagnostics.current;
            QString positionError = "unavailable", angleError = "unavailable";
            if (sample.pose) {
                positionError = QString::number(std::hypot((*sample.pose)[0] - target[0],
                    (*sample.pose)[1] - target[1], (*sample.pose)[2] - target[2]), 'g', 17);
                const auto a = calib_transform::axisAnglePoseToTransform(sample.pose->data());
                const auto b = calib_transform::axisAnglePoseToTransform(target.data());
                const cv::Mat rotation = a(cv::Rect(0, 0, 3, 3)).t() * b(cv::Rect(0, 0, 3, 3));
                angleError = QString::number(std::acos(std::clamp((cv::trace(rotation)[0] - 1) / 2,
                    -1.0, 1.0)) * 180 / CV_PI, 'g', 17);
            }
            APP_LOG_EVENT_ERROR("robot.motion", "manual_tcp_timeout", "expired",
                {{"tcp", name}, {"phase", QString::fromLatin1(phase)},
                 {"elapsed_ms", QString::number(elapsed)}, {"timeout_ms", QString::number(wait.timeoutMs)},
                 {"dispatch_elapsed_ms", dispatched ? QString::number(std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - *dispatched).count()) : QString("unavailable")},
                 {"polls", QString::number(diagnostics.polls)}, {"target", poseText(target)},
                 {"mode", sample.mode ? QString::number(*sample.mode) : QString("unavailable")},
                 {"moving", sample.moving ? QString::number(*sample.moving) : QString("unavailable")},
                 {"actual", poseText(sample.pose)}, {"position_error_mm", positionError}, {"angle_error_deg", angleError},
                 {"position_tolerance_mm", QString::number(positionToleranceMm, 'g', 17)},
                 {"angle_tolerance_deg", QString::number(wait.angleToleranceDeg, 'g', 17)},
                 {"stable_observed_ms", measuredMs(diagnostics.stableObservedMs)}, {"settling_ms", QString::number(wait.settlingMs)},
                 {"pose_read_ms", measuredMs(sample.poseReadMs)}, {"sample_read_ms", measuredMs(sample.sampleReadMs)},
                 {"last_complete_actual", poseText(diagnostics.lastComplete.pose)},
                 {"last_complete_mode", diagnostics.lastComplete.mode ? QString::number(*diagnostics.lastComplete.mode) : QString("unavailable")},
                 {"last_complete_moving", diagnostics.lastComplete.moving ? QString::number(*diagnostics.lastComplete.moving) : QString("unavailable")},
                 {"last_complete_age_ms", diagnostics.lastComplete.returned ? QString::number(std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - *diagnostics.lastComplete.returned).count()) : QString("unavailable")}});
            throw std::runtime_error("TCP 移动超时，请检查机器人连接和状态");
        }
    };
    const auto checked = preflight(handle, name, target, control, poseCheck);
    stationary(handle, control);
    // Refresh immediately before motion; configuration changes invalidate the IK.
    const auto fresh = readTcpPose(handle, name);
    control("final_pose");
    if (fresh.offset != checked.selected.offset)
        throw std::runtime_error("所选 TCP 工具偏移已变化，请重新确认目标");
    if (!flangeAtTarget(fresh.pose, checked.selected.pose))
        throw std::runtime_error("机器人位置在运动准备期间发生变化，请重新确认目标");
    if (poseCheck) poseCheck(fresh);
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
    APP_LOG_EVENT_INFO("robot.motion", "manual_tcp_line", "start",
        {{"tcp", name}, {"speed_mm_s", QString::number(speedMmS)}, {"acc_mm_s2", "20"},
         {"target", poseText(target)}, {"fresh", poseText(fresh.pose)},
         {"angle_tolerance_deg", QString::number(wait.angleToleranceDeg, 'g', 17)},
         {"position_tolerance_mm", QString::number(positionToleranceMm, 'g', 17)},
         {"settling_ms", QString::number(wait.settlingMs)}, {"timeout_ms", QString::number(wait.timeoutMs)}});
    control("dispatch");
    try {
        dispatched = Clock::now();
        const auto code = cr_move_line(handle, command, FALSE);
        control("dispatch_result");
        check(code, "TCP 直线运动指令失败");
        waitSelected(handle, name, fresh.offset, target, wait, tick, control, poseCheck, diagnostics);
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
