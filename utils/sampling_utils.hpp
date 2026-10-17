#pragma once

#include "robotapi.h"

namespace sampling {

// Resolve the flange pose using the current joints, preserving sampler behavior.
CRresult moveToFlangeAxisPos(RobotHandle robotHandle, double axisAnglePos[6]);
void printProgress(int current, int total);

} // namespace sampling
