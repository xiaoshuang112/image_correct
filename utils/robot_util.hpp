#ifndef ROBOT_UTIL_HPP
#define ROBOT_UTIL_HPP

#include "robotapi.h"
#include "basestruct.h"
#include <array>
#include <string>
#include <vector>

using RobotMotionCancelCheck = bool (*)();

// SDK 没有“用户取消”返回码，使用不会与 CRresult 正值枚举冲突的本地值。
constexpr CRresult robot_motion_cancelled = static_cast<CRresult>(-1000);

// Collection leveling registers cancellation; nullptr restores the default.
void setRobotMotionCancelCheck(RobotMotionCancelCheck check);
bool robotMotionCancellationRequested();

struct RobotMotionDiagnostics {
    std::string outcome = "pending";
    std::string phase;
    std::string origin;       // application / sdk / user
    std::string sdk_function;
    CRresult result = sucess;
    CRresult sdk_result = sucess;
    CRresult stop_result = sucess;
    long long elapsed_ms = 0;
    int timeout_ms = 0;
    int polls = 0;
    int joint_read_failures = 0;
    int status_read_failures = 0;
    int stable_polls = 0;
    bool moving = true;
    double initial_error_degrees = 0.0;
    std::array<double, ROB_AXIS_NUM> initial{};
    std::array<double, ROB_AXIS_NUM> target{};
    std::array<double, ROB_AXIS_NUM> actual{};
};

// 运动到目标关节角（等效 goto_jointpos）
CRresult moveToJointPos(int robot_handle, const double joint[6],
                        RobotMotionDiagnostics* diagnostics = nullptr);


#endif // ROBOT_UTIL_HPP
