#pragma once

#include "ManualTcpMotion.hpp"
#include <atomic>
#include <chrono>
#include <cstdint>

namespace collection {

// Displayed Euler angles are roll, pitch, yaw in degrees, R = Rz(yaw) Ry(pitch) Rx(roll).
enum class TeachOrientation { RotationVector, EulerZYX };
struct TeachSpeeds {
    double linearMmS = 10;
    // Target-parameter progression rate, NOT a controller instantaneous angular
    // velocity limit. SDK only documents Cartesian command speed as mm/s.
    double angularDegS = 5;
};
constexpr double maximumTeachAngularDegS = 180;
bool validTeachSpeeds(const TeachSpeeds& speeds);

// Pure conversions. Euler extraction refuses the ambiguous +/-90 degree region;
// input is never silently reinterpreted as another orientation convention.
FlangePose tcpPoseForDisplay(const FlangePose& axisAngle, TeachOrientation orientation);
FlangePose tcpPoseFromDisplay(const FlangePose& displayed, TeachOrientation orientation);
FlangePose teachJogTarget(const FlangePose& displayed, TeachOrientation orientation,
    int component, double increment);

// A lease belongs to one press only. release()/expiry permanently invalidate it; a
// delayed UI heartbeat cannot restart a released press. Heartbeat from the UI
// timer while the original pointer/key remains held (recommended interval 50ms).
class TeachJogLease {
public:
    static constexpr int expiryMs = 250;
    TeachJogLease();
    void heartbeat();
    void release();
    bool valid() const;
private:
    std::atomic<bool> released_{false};
    std::atomic<std::int64_t> heartbeatMs_;
};
struct TeachJogRequest {
    int component = 0;
    int direction = 1;
    TeachOrientation orientation = TeachOrientation::RotationVector;
    TeachSpeeds speeds;
};

// Bounded, non-blended, strictly serial segments, using the selected TCP in BASE.
// Exceptions always propagate to the owner, which must independently confirm
// robot stopped before unlocking. Return on release does not confirm stopped.
void jogSelectedTcp(RobotHandle handle, const QString& name, const TeachJogRequest& request,
    const TeachJogLease& lease, const std::atomic<bool>& cancelled, MotionTick tick = {});
} // namespace collection
