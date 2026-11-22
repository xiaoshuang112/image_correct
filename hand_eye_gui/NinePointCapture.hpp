#pragma once

#include "robotapi.h"
#include "RobotMotionLimits.hpp"
#include <array>
#include <atomic>
#include <functional>
#include <string>
#include <vector>

namespace collection {

using FlangePose = std::array<double, ROB_AXIS_NUM>;
constexpr double positionToleranceMm = 0.05;
using NinePointAngles = std::array<double, 9>;
inline constexpr NinePointAngles defaultNinePointYawOffsets{0, 15, -15, -20, 0, 20, 15, -15, 0};
using NinePointTargets = std::array<FlangePose, defaultNinePointYawOffsets.size()>;
constexpr double maximumNinePointOffsetMm = 50.0;
constexpr double maximumNinePointYawDegrees = 20.0;

// Row-major grid: X increases left to right, Y increases bottom to top.
// Angles modify ZYX Euler yaw only; the center can also have a nonzero angle.
NinePointTargets ninePointTargets(const FlangePose& origin, double xMm, double yMm,
    const NinePointAngles& yawOffsets = defaultNinePointYawOffsets);

// Callbacks complete synchronously or throw. Every capture point is visited in
// order, then index -1 returns once to the original baseline. Waypoints inside
// a move are not captured. Never move after a failed save, and do not return
// early when a move, capture, or the final return fails.
void runPointCapture(const FlangePose& origin, const std::vector<FlangePose>& targets,
    const std::function<void(const FlangePose&, int)>& move,
    const std::function<void(int)>& capture,
    const std::atomic<bool>& cancelled);
FlangePose withEulerYawOffset(const FlangePose& origin, double degrees);
double flangeRotationDistanceDeg(const FlangePose& from, const FlangePose& to);

FlangePose readStationaryFlange(RobotHandle handle);
bool flangeAtTarget(const FlangePose& actual, const FlangePose& target, double angleToleranceDeg = 0.2);
void checkNinePointReachability(RobotHandle handle, const NinePointTargets& targets);
// Inverse checks for the nine capture poses and any straight-line waypoints.
// This is not a collision check; the physical path still needs a field check.
void checkNinePointRoute(RobotHandle handle, const FlangePose& origin, const NinePointTargets& targets);
// Each leg contains preflighted stops, including its target. The last leg returns to the origin.
std::vector<std::vector<FlangePose>> planConfiguredRoute(RobotHandle handle, const FlangePose& origin,
    const std::vector<FlangePose>& targets, const std::vector<std::string>& labels,
    const std::atomic<bool>& cancelled);
void confirmRobotStopped(RobotHandle handle);
// Stops along the straight grid path from `from` to `to`, including `to`.
// Each segment stays within the 50 mm motion limit. Intermediate stops are not photos.
std::vector<FlangePose> linearGridStops(const FlangePose& origin, const FlangePose& from,
    const FlangePose& to);

struct LineMotionWait {
    int settlingMs = 1200;
    int timeoutMs = 30000;
    double angleToleranceDeg = 0.2;
};
// Invoked synchronously on the motion worker. The caller can sample telemetry
// without making concurrent SDK calls on the same handle. Exceptions stop motion.
using MotionTick = std::function<void()>;
void waitForStableFlange(RobotHandle handle, const FlangePose& target,
    const std::atomic<bool>& cancelled, LineMotionWait wait = {}, MotionTick tick = {});
void moveFlangeLinear(RobotHandle handle, const FlangePose& origin, const FlangePose& target,
    const std::atomic<bool>& cancelled, LineMotionWait wait = {}, MotionTick tick = {},
    double speedMmS = defaultRobotSpeedMmS);
// Split a grid leg that exceeds one 50 mm segment, then run each segment through
// moveFlangeLinear so status, stop and arrival checks still apply.
void moveFlangeAlongGrid(RobotHandle handle, const FlangePose& origin, const FlangePose& target,
    const std::atomic<bool>& cancelled, LineMotionWait wait = {}, MotionTick tick = {},
    double speedMmS = defaultRobotSpeedMmS);
void moveFlangeAlongConfiguredLeg(RobotHandle handle, const FlangePose& origin, FlangePose expected,
    const std::vector<FlangePose>& stops, const std::atomic<bool>& cancelled,
    LineMotionWait wait, MotionTick tick, double speedMmS);

} // namespace collection
