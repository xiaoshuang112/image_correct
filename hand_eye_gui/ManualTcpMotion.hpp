#pragma once

#include "NinePointCapture.hpp"
#include "RobotTcp.hpp"

namespace collection {

// Pure rigid-transform conversion used only for flange IK preflight. The motion
// command itself retains the TCP target and explicit tool for TCP-linear motion.
FlangePose tcpTargetToFlange(const FlangePose& target, const FlangePose& toolOffset);

// Read-only preflight of the complete requested destination before a sequence
// starts. Each actual segment still repeats these checks using fresh state.
void checkSelectedTcpTarget(RobotHandle handle, const QString& name, const FlangePose& target,
    const std::atomic<bool>& cancelled);

// Moves the selected tool in base coordinates without changing the active tool.
// Re-reads tool configuration before dispatch; waits on actual selected-TCP pose
// and stationary status. Any exception after dispatch requests stop and rethrows;
// the owner must independently confirmRobotStopped before unlocking operations.
void moveSelectedTcpLinear(RobotHandle handle, const QString& name, const FlangePose& target,
    const std::atomic<bool>& cancelled, LineMotionWait wait = {1200, 180000}, MotionTick tick = {},
    double speedMmS = defaultRobotSpeedMmS, MotionTick safetyCheck = {});

} // namespace collection
