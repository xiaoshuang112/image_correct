#include "robot_util.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

// ============================================================
// 运动到目标关节角（等效 goto_jointpos）
// ============================================================
static bool jointsWithinTolerance(const double actual[6], const double target[6],
                                   double tol_deg) {
    for (int i = 0; i < ROB_AXIS_NUM; i++) {
        if (std::abs(actual[i] - target[i]) > tol_deg)
            return false;
    }
    return true;
}

CRresult moveToJointPos(int robot_handle, const double joint[6]) {
    double pose[6] = {0, 0, 0, 0, 0, 0};
    cr_get_jointActualPos(robot_handle, pose);
    PointControlParaSimple simple;
    for (int i = 0; i < ROB_AXIS_NUM; i++) {
        simple.pose[i] = pose[i];
        simple.jointpos[i] = joint[i];
        simple.tcpOffset[i] = 0;
        simple.coordinatePose[i] = 0;
        simple.speed[i] = 30;
        simple.acc[i] = 30;
    }
    simple.coordinateType = jointCoordinate;
    simple.pointTransType = pointTransStop;
    simple.motiontriggerMode = MovetriggerbyOnlyRpc;

    PointControlPara para;
    CRresult ret = cr_move_pointControlPara_transfer(
        (RobotHandle)robot_handle, simple, &para);
    if (ret != sucess) {
        std::cerr << "[robot_util] cr_move_pointControlPara_transfer failed: "
                  << ret << std::endl;
        return ret;
    }
    ret = cr_move_joint((RobotHandle)robot_handle, para, TRUE);
    if (ret != sucess) {
        std::cerr << "[robot_util] cr_move_joint failed: " << ret << std::endl;
        return ret;
    }

    // 等待机器人实际到位（轮询实际关节角，到位后再加稳定延时）
    double actual[6];
    const double tol_deg = 0.5;
    const int max_polls = 200;       // 最多等 10 秒
    const int poll_ms = 50;
    for (int wait = 0; wait < max_polls; wait++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(poll_ms));
        if (cr_get_jointActualPos((RobotHandle)robot_handle, actual) != sucess)
            continue;
        if (jointsWithinTolerance(actual, joint, tol_deg)) {
            // 到位后再稳定 200ms，消除残余抖动
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            break;
        }
    }

    return ret;
}

// ============================================================
// 法兰轴角 → 关节角
// ============================================================
bool convertFlangeAxisPos2Joint(int robot_handle, double* flangeAnglePos, double *joint) {
    double ref_j[ROB_AXIS_NUM];
    if (cr_get_jointActualPos
        ((RobotHandle)robot_handle, ref_j) != sucess)
        return false;
    if (cr_kineInverse((RobotHandle)robot_handle, flangeAnglePos, ref_j, joint) != sucess)
        return false;
    return true;
}

// ============================================================
// TCP 轴角 → 法兰轴角
// ============================================================
bool convertTcpAxisPos2FlangPos(int robotHandle, double *tcpAnglePos, double *flangeAnglePos) {
    TCPMsg tcpMsgList;
    if (cr_get_currentTCPmsg(robotHandle, &tcpMsgList) == sucess) {
        if (cr_TcpToFlangePose(tcpAnglePos, tcpMsgList.tcpOffset, flangeAnglePos) == sucess)
            return true;
    }
    return false;
}

// ============================================================
// 法兰欧拉 → 关节角
// ============================================================
bool convertFlangeOulePos2Joint(int robot_handle, double* flangeEulePos, double *joint) {
    double flangeAxisPos[ROB_AXIS_NUM];
    if (cr_Eule2AxisAngle(flangeEulePos, flangeAxisPos) != sucess)
        return false;
    return convertFlangeAxisPos2Joint(robot_handle, flangeAxisPos, joint);
}

// ============================================================
// TCP 轴角 → 法兰欧拉
// ============================================================
bool convertTcpAxisPos2FlangeEule(int robot_handle, double *tcpAxisPos, double *flangeEulePos) {
    double flangeAxisPos[ROB_AXIS_NUM];
    if (convertTcpAxisPos2FlangPos(robot_handle, tcpAxisPos, flangeAxisPos)) {
        cr_AxisAngle2Eule(flangeAxisPos, flangeEulePos);
        return true;
    }
    return false;
}

// ============================================================
// 运动到目标法兰轴角位姿
// ============================================================
bool moveToFlangeAxisPos(int robot_handle, double* axisAnglePos) {
    double joint[6];
    if (convertFlangeAxisPos2Joint(robot_handle, axisAnglePos, joint)) {
        moveToJointPos(robot_handle, joint);
        return true;
    }
    return false;
}

// ============================================================
// 运动到目标 TCP 轴角位姿
// ============================================================
bool moveToTcpAxisPos(int robotHandle, double *tcpAnglePos) {
    double flangeAxisAnglePos[6];
    if (convertTcpAxisPos2FlangPos(robotHandle, tcpAnglePos, flangeAxisAnglePos))
        return moveToFlangeAxisPos(robotHandle, flangeAxisAnglePos);
    return false;
}

// ============================================================
// 运动到目标法兰欧拉位姿
// ============================================================
bool moveToFlangeEulePos(int robotHandle, double *flangeEulePos) {
    double flangeAxisAnglePos[6];
    cr_Eule2AxisAngle(flangeEulePos, flangeAxisAnglePos);
    return moveToFlangeAxisPos(robotHandle, flangeAxisAnglePos);
}
