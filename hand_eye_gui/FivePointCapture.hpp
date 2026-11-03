#pragma once

#include "robotapi.h"
#include <array>
#include <atomic>
#include <functional>

namespace collection {

using FlangePose = std::array<double, ROB_AXIS_NUM>;
using FivePointTargets = std::array<FlangePose, 5>;
constexpr double maximumFivePointOffsetMm = 50.0;

FivePointTargets fivePointTargets(const FlangePose& origin, double xMm, double yMm);
// Callbacks complete synchronously or throw. Never move after a failed save.
void runFivePointCapture(const FivePointTargets& targets,
    const std::function<void(const FlangePose&, int)>& move,
    const std::function<void(int)>& capture,
    const std::atomic<bool>& cancelled);

FlangePose readStationaryFlange(RobotHandle handle);
bool flangeAtTarget(const FlangePose& actual, const FlangePose& target);
void checkFivePointReachability(RobotHandle handle, const FivePointTargets& targets);
void confirmRobotStopped(RobotHandle handle);

struct LineMotionWait {
    int settlingMs = 1200;
    int timeoutMs = 30000;
};
void waitForStableFlange(RobotHandle handle, const FlangePose& target,
    const std::atomic<bool>& cancelled, LineMotionWait wait = {});
void moveFlangeLinear(RobotHandle handle, const FlangePose& target,
    const std::atomic<bool>& cancelled, LineMotionWait wait = {});

} // namespace collection
