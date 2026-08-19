#include "robot_util.hpp"
#include "hand_eye_gui/DiagnosticLogger.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <vector>
#include <algorithm>
#include <atomic>

namespace {

constexpr int kMotionPollMs = 50;
constexpr int kMotionProgressLogMs = 2000;
constexpr int kMotionMinTimeoutMs = 20000;
constexpr int kMotionMaxTimeoutMs = 120000;
constexpr double kMotionTimeoutPerDegreeMs = 1500.0;

std::atomic<RobotMotionCancelCheck> g_motionCancelCheck{nullptr};

double nearestEquivalentAngle(double angle, double reference)
{
    return angle + std::round((reference - angle) / 360.0) * 360.0;
}

double maximumJointError(const double actual[6], const double target[6])
{
    double maximum = 0.0;
    for (int i = 0; i < ROB_AXIS_NUM; ++i) {
        double delta = actual[i] - target[i];
        if (i == 5)
            delta = std::remainder(delta, 360.0);
        maximum = std::max(maximum, std::abs(delta));
    }
    return maximum;
}

#ifdef QT_GUI
QString formatSixValues(const double values[6])
{
    return QString("[%1,%2,%3,%4,%5,%6]")
        .arg(values[0], 0, 'f', 4).arg(values[1], 0, 'f', 4)
        .arg(values[2], 0, 'f', 4).arg(values[3], 0, 'f', 4)
        .arg(values[4], 0, 'f', 4).arg(values[5], 0, 'f', 4);
}
#endif

} // namespace

void setRobotMotionCancelCheck(RobotMotionCancelCheck check)
{
    g_motionCancelCheck.store(check, std::memory_order_release);
}

bool robotMotionCancellationRequested()
{
    const RobotMotionCancelCheck check =
        g_motionCancelCheck.load(std::memory_order_acquire);
    return check && check();
}

// ============================================================
// 运动到目标关节角（等效 goto_jointpos）
// ============================================================
static bool jointsWithinTolerance(const double actual[6], const double target[6],
                                   double tol_deg) {
    for (int i = 0; i < ROB_AXIS_NUM; i++) {
        double delta = actual[i] - target[i];
        if (i == 5)
            delta = std::remainder(delta, 360.0);
        if (std::abs(delta) > tol_deg)
            return false;
    }
    return true;
}


CRresult moveToJointPosByLine(int robot_handle, const double tcpAxisPos[ROB_AXIS_NUM])//, const std::vector<double>& basepose)
{
    double init_joint[ROB_AXIS_NUM] = {};
    CRresult result = cr_get_jointActualPos((RobotHandle)robot_handle, init_joint);
    if (result != sucess) {
        APP_LOG_EVENT_ERROR("robot.motion",
                            "joint_move_linear",
                            "fail",
                            {{"reason", "initial_joint_read_failed"},
                             {"handle", QString::number(robot_handle)},
                             {"error", QString::number(result)}});
        std::cerr << "[moveLine] 读取当前关节角失败 ret=" << result << std::endl;
        return result;
    }
    // if ( jointpos.size()!=ROB_AXIS_NUM){        
    //     std::cerr << "moveLine失败：未连接或参数错误 " << std::endl;
    //     return robotmode_error;
    // }
    // 读取当前激活的TCP
	int tcpindex = -1;
	result = cr_cfg_tcp_active_get(robot_handle, &tcpindex);
	if (result != sucess)
	{		
        std::cerr << "[moveLine] 读取当前激活的TCP失败 " << std::endl;
        return result;
	}

	PointControlPara pointControlPara;
	for (int index = 0; index < ROB_AXIS_NUM; index++)
	{
		pointControlPara.pose[index] = tcpAxisPos[index];
		pointControlPara.jointpos[index] = init_joint[index];
		pointControlPara.tcpOffset[index] = 0;
		pointControlPara.coordinatePose[index] = 0;

#ifdef X86_64
		pointControlPara.speed[index] = 150;
		pointControlPara.acc[index] = 250;

#else
		pointControlPara.speed[index] = 150;
		pointControlPara.acc[index] = 250;

#endif // X86_64
        
        
        pointControlPara.jerk[index] = 60;
	}
    // 当tcpID为非负数时，使用tcpID对应的TCP偏移量作为输入；当tcpID为负数时，使用tcpoffset数据作为TCP偏移量输入；
	pointControlPara.tcpID = tcpindex;		
	pointControlPara.coordinateType = PointCoordinate; 
	pointControlPara.pointTransType = pointTransStop;
	pointControlPara.pointTransRadius = 0;
	pointControlPara.poseTranType = poseTranMoveToTargetPose;
	pointControlPara.motiontriggerMode = MovetriggerbyOnlyRpc;

	result = cr_move_line(robot_handle, pointControlPara, TRUE);
    if (result != sucess)
    {
        std::cerr << "moveLine失败：cr_move_line ret= " << result << std::endl;
        return result;
    }
    return result;
}

CRresult moveToJointPos(int robot_handle, const double joint[6]) {
    const auto startedAt = std::chrono::steady_clock::now();
    double pose[6] = {0, 0, 0, 0, 0, 0};
    CRresult ret = cr_get_jointActualPos(robot_handle, pose);
    if (ret != sucess) {
        APP_LOG_EVENT_ERROR("robot.motion",
                            "joint_move",
                            "fail",
                            {{"phase", "query_initial_joint"},
                             {"reason", "initial_joint_read"},
                             {"handle", QString::number(robot_handle)},
                             {"error", QString::number(ret)}});
        std::cerr << "[robot_util] cr_get_jointActualPos failed: "
                  << ret << std::endl;
        return ret;
    }

    // SDK 逆解可能把 J6 归一化到另一圈。保持与当前角度连续，避免例如
    // 318° -> -42° 这种物理上无意义的整圈运动。
    double target[ROB_AXIS_NUM];
    std::copy(joint, joint + ROB_AXIS_NUM, target);
    target[5] = nearestEquivalentAngle(target[5], pose[5]);

    const double initialError = maximumJointError(pose, target);
    const int timeoutMs = std::clamp(
        static_cast<int>(std::lround(15000.0 +
                                     initialError * kMotionTimeoutPerDegreeMs)),
        kMotionMinTimeoutMs,
        kMotionMaxTimeoutMs);

    APP_LOG_EVENT_INFO("robot.motion",
                       "joint_move",
                       "start",
                       {{"handle", QString::number(robot_handle)},
                        {"requested", formatSixValues(joint)},
                        {"target", formatSixValues(target)},
                        {"initial", formatSixValues(pose)},
                        {"initial_error_deg", QString::number(initialError, 'f', 3)},
                        {"timeout_ms", QString::number(timeoutMs)},
                        {"mode", "non_blocking"}});

    if (robotMotionCancellationRequested()) {
        APP_LOG_EVENT_WARNING("robot.motion",
                              "joint_move",
                              "cancelled",
                              {{"phase", "before_command"},
                               {"elapsed_ms", "0"},
                               {"target", formatSixValues(target)}});
        return robot_motion_cancelled;
    }

    PointControlParaSimple simple{};
    for (int i = 0; i < ROB_AXIS_NUM; i++) {
        simple.pose[i] = pose[i];
        simple.jointpos[i] = target[i];
        simple.tcpOffset[i] = 0;
        simple.coordinatePose[i] = 0;
        simple.speed[i] = 30;
        simple.acc[i] = 30;
    }
    simple.coordinateType = jointCoordinate;
    simple.pointTransType = pointTransStop;
    simple.motiontriggerMode = MovetriggerbyOnlyRpc;

    PointControlPara para{};
    ret = cr_move_pointControlPara_transfer(
        (RobotHandle)robot_handle, simple, &para);
    if (ret != sucess) {
        APP_LOG_EVENT_ERROR("robot.motion",
                            "joint_move",
                            "fail",
                            {{"phase", "command_build"},
                             {"reason", "point_parameter_conversion"},
                             {"error", QString::number(ret)},
                             {"target", formatSixValues(joint)}});
        std::cerr << "[robot_util] cr_move_pointControlPara_transfer failed: "
                  << ret << std::endl;
        return ret;
    }
    ret = cr_move_joint((RobotHandle)robot_handle, para, FALSE);
    if (ret != sucess) {
        APP_LOG_EVENT_ERROR("robot.motion",
                            "joint_move",
                            "fail",
                            {{"phase", "command_send"},
                             {"reason", "move_command"},
                             {"error", QString::number(ret)},
                             {"target", formatSixValues(target)}});
        std::cerr << "[robot_util] cr_move_joint failed: " << ret << std::endl;
        return ret;
    }

    // 非阻塞下发后由应用轮询实际关节角，因此超时和取消可以真正生效。
    double actual[6] = {0, 0, 0, 0, 0, 0};
    const double tol_deg = 0.5;
    bool reached = false;
    int readFailures = 0;
    int polls = 0;
    long long nextProgressLogMs = 0;
    while (true) {
        const auto now = std::chrono::steady_clock::now();
        const long long elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - startedAt).count();

        if (robotMotionCancellationRequested()) {
            const CRresult stopRet = cr_stop((RobotHandle)robot_handle);
            APP_LOG_EVENT_WARNING("robot.motion",
                                  "joint_move",
                                  "cancelled",
                                  {{"phase", "poll"},
                                   {"elapsed_ms", QString::number(elapsedMs)},
                                   {"polls", QString::number(polls)},
                                   {"stop_result", QString::number(stopRet)},
                                   {"target", formatSixValues(target)},
                                   {"actual", formatSixValues(actual)}});
            return robot_motion_cancelled;
        }

        if (elapsedMs >= timeoutMs) {
            const CRresult stopRet = cr_stop((RobotHandle)robot_handle);
            APP_LOG_EVENT_ERROR("robot.motion",
                                "joint_move",
                                "fail",
                                {{"phase", "reach"},
                                 {"reason", "timeout"},
                                 {"elapsed_ms", QString::number(elapsedMs)},
                                 {"timeout_ms", QString::number(timeoutMs)},
                                 {"polls", QString::number(polls)},
                                 {"read_failures", QString::number(readFailures)},
                                 {"stop_result", QString::number(stopRet)},
                                 {"target", formatSixValues(target)},
                                 {"actual", formatSixValues(actual)}});
            return operate_timeout;
        }

        ++polls;
        if (cr_get_jointActualPos((RobotHandle)robot_handle, actual) != sucess) {
            ++readFailures;
        } else if (jointsWithinTolerance(actual, target, tol_deg)) {
            // 到位后再稳定 200ms，消除残余抖动
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            reached = true;
            break;
        }

        if (elapsedMs >= nextProgressLogMs) {
            const double remainingError = maximumJointError(actual, target);
            const int percent = initialError <= tol_deg
                ? 100
                : std::clamp(static_cast<int>(std::lround(
                      (1.0 - remainingError / initialError) * 100.0)), 0, 99);
            APP_LOG_EVENT_INFO("robot.motion",
                               "joint_move",
                               "progress",
                               {{"elapsed_ms", QString::number(elapsedMs)},
                                {"timeout_ms", QString::number(timeoutMs)},
                                {"percent", QString::number(percent)},
                                {"remaining_error_deg", QString::number(remainingError, 'f', 3)},
                                {"polls", QString::number(polls)},
                                {"read_failures", QString::number(readFailures)},
                                {"actual", formatSixValues(actual)},
                                {"target", formatSixValues(target)}});
            nextProgressLogMs = elapsedMs + kMotionProgressLogMs;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(kMotionPollMs));
    }

    const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - startedAt).count();
    if (reached) {
        APP_LOG_EVENT_INFO("robot.motion",
                           "joint_move",
                           "success",
                           {{"elapsed_ms", QString::number(elapsedMs)},
                            {"polls", QString::number(polls)},
                            {"read_failures", QString::number(readFailures)},
                            {"actual", formatSixValues(actual)},
                            {"target", formatSixValues(target)}});
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
    // 选择与当前 J6 最近的等价圈数，保持逆解连续。
    joint[5] = nearestEquivalentAngle(joint[5], ref_j[5]);
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
        return cr_AxisAngle2Eule(flangeAxisPos, flangeEulePos) == sucess;
    }
    return false;
}

bool convertJointToFlangeEule(int robot_handle, const double joint[ROB_AXIS_NUM], double flangeEule[ROB_AXIS_NUM]){
    double flangePos[ROB_AXIS_NUM] = {};
    CRresult ret = cr_kineForward(robot_handle, const_cast<double*>(joint), flangePos);
    if (ret != sucess) {
        APP_LOG_EVENT_ERROR("robot.kinematics",
                            "transform",
                            "fail",
                            {{"phase", "forward_kinematics"},
                             {"reason", "forward_kinematics_failed"},
                             {"handle", QString::number(robot_handle)},
                             {"error", QString::number(ret)},
                             {"joint", formatSixValues(joint)}});
        return false;
    }
    ret = cr_AxisAngle2Eule(flangePos, flangeEule);
    if (ret != sucess) {
        APP_LOG_EVENT_ERROR("robot.kinematics",
                            "transform",
                            "fail",
                            {{"phase", "axis_to_euler"},
                             {"reason", "axis_to_euler_failed"},
                             {"handle", QString::number(robot_handle)},
                             {"error", QString::number(ret)},
                             {"flange_axis", formatSixValues(flangePos)}});
        return false;
    }
    return true;
}

// ============================================================
// 运动到目标法兰轴角位姿
// ============================================================
bool moveToFlangeAxisPos(int robot_handle, double* axisAnglePos) {
    double joint[6];
    if (convertFlangeAxisPos2Joint(robot_handle, axisAnglePos, joint)) {
        return moveToJointPos(robot_handle, joint) == sucess;
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
    if (cr_Eule2AxisAngle(flangeEulePos, flangeAxisAnglePos) != sucess)
        return false;
    return moveToFlangeAxisPos(robotHandle, flangeAxisAnglePos);
}
