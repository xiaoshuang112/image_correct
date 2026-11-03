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

// Read-only SDK calls. Failures and malformed responses throw; no tool is activated.
TcpList readTcpList(RobotHandle handle);
TcpPose readTcpPose(RobotHandle handle, const QString& name);
} // namespace collection
