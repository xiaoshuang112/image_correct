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

// 标定线程注册取消检查；传 nullptr 可恢复为不检查取消。
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

CRresult moveToJointPosByLine(int robot_handle, const double flangAxisPos[ROB_AXIS_NUM]);

// 法兰轴角 → 关节角
bool convertFlangeAxisPos2Joint(int robot_handle, double* flangeAnglePos, double *joint);

// TCP 轴角 → 法兰轴角
bool convertTcpAxisPos2FlangPos(int robotHandle, double *tcpAnglePos, double *flangeAnglePos);

// 法兰欧拉 → 关节角
bool convertFlangeOulePos2Joint(int robot_handle, double* flangeEulePos, double *joint);

// TCP 轴角 → 法兰欧拉
bool convertTcpAxisPos2FlangeEule(int robot_handle, double *tcpAxisPos, double *flangeEulePos);
bool convertJointToFlangeEule(int robot_handle, const double joint[ROB_AXIS_NUM], double flangeEule[ROB_AXIS_NUM]);

// 运动到目标法兰轴角位姿
bool moveToFlangeAxisPos(int robot_handle, double* axisAnglePos);

// 运动到目标 TCP 轴角位姿
bool moveToTcpAxisPos(int robotHandle, double *tcpAnglePos);

// 运动到目标法兰欧拉位姿
bool moveToFlangeEulePos(int robotHandle, double *flangeEulePos);

#endif // ROBOT_UTIL_HPP
