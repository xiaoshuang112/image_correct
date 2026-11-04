#pragma once

#include "robotapi.h"
#include <QStringList>
#include <array>

namespace collection {
struct TcpList {
    QStringList names;
    QString current;
    QString warning;
};

struct TcpPose {
    QString name;
    std::array<double, ROB_AXIS_NUM> offset{};
    std::array<double, ROB_AXIS_NUM> pose{};
};

// Absolute base-coordinate TCP pose: mm and axis-angle degrees, in X,Y,Z,Rx,Ry,Rz order.
// Pure input validation; throws a Chinese error and never calls the SDK.
std::array<double, ROB_AXIS_NUM> parseTcpTarget(const QString& text);

// Read-only SDK calls. Failures and malformed responses throw; no tool is activated.
TcpList readTcpList(RobotHandle handle);
TcpPose readTcpPose(RobotHandle handle, const QString& name);
} // namespace collection
