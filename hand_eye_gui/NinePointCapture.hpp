#pragma once

#include "robotapi.h"
#include "RobotMotionLimits.hpp"
#include <array>
#include <atomic>
#include <functional>

namespace collection {

using FlangePose = std::array<double, ROB_AXIS_NUM>;
inline constexpr std::array<double, 9> ninePointYawOffsets{0, 15, -15, -20, 0, 20, 15, -15, 0};
using NinePointTargets = std::array<FlangePose, ninePointYawOffsets.size()>;
constexpr double maximumNinePointOffsetMm = 50.0;

NinePointTargets ninePointTargets(const FlangePose& origin, double xMm, double yMm);
// Row-major grid: X increases left to right, Y increases bottom to top.
// Center (index 4) is the baseline. Angles modify ZYX Euler yaw only.
// Callbacks complete synchronously or throw. Return moves use index 4 and do not
// capture. Never move after a failed save.
void runNinePointCapture(const NinePointTargets& targets,
    const std::function<void(const FlangePose&, int)>& move,
    const std::function<void(int)>& capture,
    const std::atomic<bool>& cancelled);

FlangePose readStationaryFlange(RobotHandle handle);
bool flangeAtTarget(const FlangePose& actual, const FlangePose& target);
void checkNinePointReachability(RobotHandle handle, const NinePointTargets& targets);
void confirmRobotStopped(RobotHandle handle);

struct LineMotionWait {
    int settlingMs = 1200;
    int timeoutMs = 30000;
};
// Invoked synchronously on the motion worker. The caller can sample telemetry
// without making concurrent SDK calls on the same handle. Exceptions stop motion.
using MotionTick = std::function<void()>;
void waitForStableFlange(RobotHandle handle, const FlangePose& target,
    const std::atomic<bool>& cancelled, LineMotionWait wait = {}, MotionTick tick = {});
void moveFlangeLinear(RobotHandle handle, const FlangePose& origin, const FlangePose& target,
    const std::atomic<bool>& cancelled, LineMotionWait wait = {}, MotionTick tick = {},
    double speedMmS = defaultRobotSpeedMmS);

} // namespace collection
