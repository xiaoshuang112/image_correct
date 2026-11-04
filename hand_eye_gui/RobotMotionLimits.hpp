#pragma once

namespace collection {
constexpr int maximumRobotSpeedMmS = 1000;
constexpr int defaultRobotSpeedMmS = 10;
constexpr bool validRobotSpeed(double speedMmS) {
    return speedMmS >= 1 && speedMmS <= maximumRobotSpeedMmS;
}
}
