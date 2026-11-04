#include "hand_eye_gui/CollectionController.hpp"
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QTemporaryDir>
#include <algorithm>
#include <array>
#include <atomic>
#include <functional>
#include <cstring>
#include <limits>
#include <iostream>
#include <stdexcept>
#include <thread>

void testCollectionSettings();

namespace {
void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
void waitFor(const std::function<bool()>& ready, int timeoutMs = 6000) {
    QElapsedTimer timer; timer.start();
    while (!ready() && timer.elapsed() < timeoutMs) {
        QCoreApplication::processEvents();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    require(ready(), "controller transition timed out");
}
const auto sdkError = static_cast<CRresult>(1);
struct Robot {
    std::array<double, 6> pose{100, 200, 300, 180, 0, 0};
    std::atomic<bool> moving{false}, stopWorks{false}, statusFails{false};
    std::atomic<int> lines{0}, stops{0}, destroys{0}, unexpected{0};
    std::atomic<int> readDelayMs{0}, activeReads{0}, toolReads{0}, destroyDuringRead{0};
    std::atomic<bool> invalidPose{false}, poseReadFails{false};
    bool createWorks = false, powerFails = false;
    std::atomic<int> mode{Enable}, creates{0}, resets{0}, powers{0}, enables{0};
    CRresult destroyCode = sucess;
    CRresult stopCode = sdkError;
    bool failLine = true;
    void reset() {
        pose = {100, 200, 300, 180, 0, 0};
        moving = stopWorks = statusFails = false;
        lines = stops = destroys = unexpected = 0;
        destroyCode = sucess;
        stopCode = sdkError; failLine = true;
        readDelayMs = activeReads = toolReads = destroyDuringRead = 0;
        invalidPose = poseReadFails = false;
        createWorks = powerFails = false;
        mode = Enable; creates = resets = powers = enables = 0;
    }
} robot;
class Camera : public AICamera {
public:
    Camera() : AICamera(CameraConfig{}) {}
    bool open() override { return true; }
    void close() override {}
    bool isOpened() const override { return true; }
    cv::Mat captureOneFrame(int) override { return {}; }
    bool setColorExposureTime(int) override { return true; }
    int getColorExposureTime() override { return 5000; }
    bool setAutoExposure(bool) override { return true; }
    bool getAutoExposure() override { return false; }
    bool getExposureRange(int& low, int& high) override { low = 100; high = 50000; return true; }
    std::vector<cv::Mat> captureMultiExposureImages(const std::vector<int>& values) override {
        std::vector<cv::Mat> frames;
        for (int value : values) frames.emplace_back(32, 32, CV_8UC3, cv::Scalar(value / 100, 80, 100));
        return frames;
    }
};
}

// Inject devices only; exercise the real asynchronous controller and public API.
struct CollectionControllerTestAccess {
    static void prepare(CollectionController& controller, const QString& output) {
        controller.m_camera = std::make_shared<Camera>();
        controller.m_cameraOpened = controller.m_previewActive = controller.m_robotConnected = true;
        controller.m_previewFrameCount = 1;
        controller.m_robotHandle = 42;
        controller.m_outputDirectory = output;
        controller.m_exposureList = "5000,8500";
    }
    static void tools(CollectionController& controller) {
        controller.m_tcpNames = QStringList{"ToolA", "ToolB"};
    }
    static void verifySessionGates() {
        CollectionController::RobotSession session(42);
        std::atomic<bool> cancelled{true};
        require(!session.acquire(cancelled).owns_lock(), "cancelled session acquired free SDK mutex");
        cancelled = false;
        session.closing = true;
        require(!session.acquire(cancelled).owns_lock(), "closing session acquired free SDK mutex");
        session.closing = false;
        require(session.acquire(cancelled).owns_lock(), "healthy session could not acquire SDK mutex");
    }
};

extern "C" CRresult cr_get_robotMode(RobotHandle, RobotModes* mode) {
    *mode = robot.moving ? SDK_Moving : static_cast<RobotModes>(robot.mode.load());
    return sucess;
}
extern "C" CRresult cr_get_robotMoveStatus(RobotHandle, BOOL* moving) {
    *moving = robot.moving;
    return robot.statusFails ? sdkError : sucess;
}
extern "C" CRresult cr_get_AssignTCP_Pose(RobotHandle, double* tool, double* pose) {
    ++robot.activeReads;
    if (tool[0] != 0) ++robot.toolReads;
    const int delay = robot.readDelayMs;
    if (delay) std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    std::copy(robot.pose.begin(), robot.pose.end(), pose);
    pose[0] += tool[0];
    if (robot.invalidPose) pose[1] = std::numeric_limits<double>::quiet_NaN();
    --robot.activeReads;
    return robot.poseReadFails ? sdkError : sucess;
}
extern "C" CRresult cr_get_jointActualPos(RobotHandle, double* joints) {
    std::fill_n(joints, ROB_AXIS_NUM, 0); return sucess;
}
extern "C" CRresult cr_kineInverse(RobotHandle, double*, double*, double* joints) {
    std::fill_n(joints, ROB_AXIS_NUM, 0); return sucess;
}
extern "C" CRresult cr_move_line(RobotHandle, PointControlPara command, BOOL) {
    ++robot.lines;
    if (robot.failLine) { robot.moving = true; return sdkError; }
    std::copy_n(command.pose, ROB_AXIS_NUM, robot.pose.begin());
    robot.pose[0] -= command.tcpID < 0 ? command.tcpOffset[0] : 0;
    return sucess;
}
extern "C" CRresult cr_stop(RobotHandle) {
    ++robot.stops;
    if (robot.stopWorks) robot.moving = false;
    return robot.stopCode;
}
extern "C" CRresult cr_destroy_robot(RobotHandle) { if (robot.activeReads) ++robot.destroyDuringRead; ++robot.destroys; return robot.destroyCode; }
extern "C" CRresult cr_create_robot(RobotHandle* handle, const char*, int, const char*) {
    ++robot.creates;
    if (!robot.createWorks) { ++robot.unexpected; return sdkError; }
    *handle = 42;
    return sucess;
}
extern "C" CRresult cr_FaultReset(RobotHandle) { ++robot.resets; return sucess; }
extern "C" CRresult cr_poweron(RobotHandle) { ++robot.powers; if (robot.powerFails) return sdkError; robot.mode = JointIdle; return sucess; }
extern "C" CRresult cr_enable(RobotHandle) { ++robot.enables; robot.mode = ProgramStop; return sucess; }
extern "C" CRresult cr_kineForward(RobotHandle, double*, double*) { ++robot.unexpected; return sdkError; }
extern "C" CRresult cr_move_joint(RobotHandle, PointControlPara, BOOL) { ++robot.unexpected; return sdkError; }
extern "C" CRresult cr_move_pointControlPara_transfer(RobotHandle, PointControlParaSimple, PointControlPara*) {
    ++robot.unexpected; return sdkError;
}
extern "C" CRresult cr_get_allTCPmsg(RobotHandle, TCPMsg*, int* count) {
    if (!robot.createWorks) { ++robot.unexpected; return sdkError; }
    *count = 0;
    return sucess;
}
extern "C" CRresult cr_get_currentTCPmsg(RobotHandle, TCPMsg*) { ++robot.unexpected; return sdkError; }
extern "C" CRresult cr_cfg_tcp_accord_name_get(RobotHandle, char* name, TCPMsg* tcp) {
    *tcp = {};
    std::strncpy(tcp->tcpName, name, sizeof(tcp->tcpName) - 1);
    tcp->tcpOffset[0] = std::strcmp(name, "ToolA") == 0 ? 10 : 20;
    return sucess;
}
extern "C" CRresult cr_cfg_safety_joint_limit_get(RobotHandle, SafetyLimitsMode, int, SafetyLimitsJointAngle* limit) {
    limit->minJointPosition = -360;
    limit->maxJointPosition = 360;
    return sucess;
}

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);
    try {
        CollectionControllerTestAccess::verifySessionGates();
        testCollectionSettings();
        QTemporaryDir temp;
        require(temp.isValid(), "temporary output directory failed");
        {
            CollectionController controller;
            CollectionControllerTestAccess::prepare(controller, temp.path());
            controller.captureFivePoints(10, 10);
            waitFor([&] { return controller.motionFault(); });
            require(controller.collectionBusy() && robot.moving && controller.savedImageCount() == 1,
                    "stop failure unlocked controller or continued collecting");
            const auto lines = robot.lines.load();
            controller.disconnectRobot(); controller.closeCamera(); controller.connectRobot();
            controller.capturePhoto(); controller.captureFivePoints(10, 10); controller.moveTcpTo("100,200,300,180,0,0");
            require(controller.robotConnected() && controller.cameraOpened() && robot.destroys == 0 &&
                    robot.lines == lines && robot.unexpected == 0, "fault lock allowed device teardown or new operation");
            const auto stops = robot.stops.load();
            controller.cancelCollectionOperation();
            controller.cancelCollectionOperation();
            waitFor([&] { return controller.fivePointProgress().contains("操作已锁定") && robot.stops > stops; });
            require(controller.collectionBusy() && controller.motionFault() && robot.stops == stops + 1,
                    "failed retry unlocked controller or sent duplicate stop");
            robot.moving = false; robot.statusFails = true;
            controller.cancelCollectionOperation();
            waitFor([&] { return controller.fivePointProgress().contains("无法确认机器人运动状态"); });
            require(controller.collectionBusy() && controller.motionFault(), "unknown status unlocked controller");
            robot.statusFails = false;
            controller.cancelCollectionOperation();
            waitFor([&] { return !controller.collectionBusy(); });
            require(!controller.motionFault() && controller.statusMessage().contains("已独立确认机器人停止"),
                    "independently verified stop did not clear fault");
            controller.disconnectRobot();
            waitFor([&] { return !controller.robotConnected(); });
            require(!controller.robotConnected() && robot.destroys == 1, "confirmed stop did not permit disconnect");
        }
        robot.reset(); robot.stopCode = sucess;
        {
            CollectionController controller;
            CollectionControllerTestAccess::prepare(controller, temp.path());
            controller.captureFivePoints(10, 10);
            waitFor([&] { return controller.motionFault(); });
            require(controller.collectionBusy() && robot.moving, "successful stop return unlocked a moving robot");
            // Even direct destruction must not destroy an unconfirmed robot handle.
        }
        require(robot.destroys == 0 && robot.moving, "destructor destroyed robot after unconfirmed stop");
        robot.reset(); robot.stopWorks = true; robot.stopCode = sucess; robot.failLine = false;
        {
            CollectionController controller;
            CollectionControllerTestAccess::prepare(controller, temp.path());
            controller.captureFivePoints(0.1, 0.1);
            waitFor([&] { return !controller.collectionBusy(); }, 18000);
            require(!controller.motionFault() && controller.savedImageCount() == 5 && robot.lines == 8 &&
                    robot.pose == std::array<double, 6>{100,200,300,180,0,0}, "normal small-offset batch did not finish and unlock");
        }
        waitFor([&] { return robot.destroys == 1; });
        require(robot.destroys == 1 && robot.unexpected == 0, "normal cleanup failed or real SDK path was reached");
        robot.reset();
        {
            CollectionController controller;
            CollectionControllerTestAccess::prepare(controller, temp.path());
            CollectionControllerTestAccess::tools(controller);
            require(!controller.parseTcpTarget("1,2,3,4,5,nan")["valid"].toBool(), "backend accepted non-finite target");
            require(controller.parseTcpTarget("1， 2,3,4,5,6")["values"].toList().size() == 6, "backend rejected valid target");
            controller.moveTcpTo("1,2,3");
            require(!controller.collectionBusy() && robot.lines == 0, "invalid target started operation");
            robot.readDelayMs = 350;
            controller.setSelectedTcp("ToolA");
            waitFor([&] { return robot.activeReads > 0; });
            controller.setSelectedTcp("ToolB");
            require(!controller.tcpPoseValid() && controller.tcpPoseValues().isEmpty(), "tool switch retained old coordinates");
            waitFor([&] { return robot.activeReads == 0; });
            QCoreApplication::processEvents();
            require(!controller.tcpPoseValid(), "late old-tool read was published");
            robot.readDelayMs = 0;
            waitFor([&] { return controller.tcpPoseValid(); });
            require(controller.tcpPoseTool() == "ToolB" && controller.tcpPoseValues()[0].toDouble() == 120 &&
                    !controller.tcpPoseTimestamp().isEmpty(), "pose/tool/timestamp were not published together");
            robot.invalidPose = true;
            waitFor([&] { return !controller.tcpPoseValid(); });
            require(controller.tcpPoseValues().isEmpty() && controller.tcpPoseTimestamp().isEmpty(), "invalid read retained coordinates");
            robot.invalidPose = false;
            waitFor([&] { return controller.tcpPoseValid(); });
            robot.readDelayMs = 1500;
            waitFor([&] { return robot.activeReads > 0; });
            waitFor([&] { return !controller.tcpPoseValid(); });
            require(controller.tcpPoseValues().isEmpty(), "stale pose retained coordinates");
            QElapsedTimer elapsed; elapsed.start();
            controller.disconnectRobot();
            require(elapsed.elapsed() < 150 && robot.destroys == 0 && !controller.tcpPoseValid(), "disconnect blocked on telemetry or destroyed handle in use");
            waitFor([&] { return robot.destroys == 1; });
            require(robot.destroyDuringRead == 0 && !controller.tcpPoseValid(), "disconnect raced read or published late result");
        }
        robot.reset();
        {
            auto controller = std::make_unique<CollectionController>();
            CollectionControllerTestAccess::prepare(*controller, temp.path());
            CollectionControllerTestAccess::tools(*controller);
            robot.readDelayMs = 700;
            controller->setSelectedTcp("ToolA");
            waitFor([&] { return robot.activeReads > 0; });
            QElapsedTimer elapsed; elapsed.start();
            controller.reset();
            require(elapsed.elapsed() < 150 && robot.destroys == 0, "destructor joined idle telemetry or destroyed in-use handle");
            waitFor([&] { return robot.destroys == 1; });
            require(robot.destroyDuringRead == 0, "destructor raced SDK reader");
        }
        robot.reset();
        {
            auto controller = std::make_unique<CollectionController>();
            CollectionControllerTestAccess::prepare(*controller, temp.path());
            CollectionControllerTestAccess::tools(*controller);
            robot.readDelayMs = 700;
            controller->setSelectedTcp("ToolA");
            waitFor([&] { return robot.activeReads > 0; });
            controller->moveTcpTo("120,200,300,180,0,0");
            QElapsedTimer elapsed; elapsed.start();
            controller.reset();
            require(elapsed.elapsed() < 150 && robot.lines == 0 && robot.destroys == 0,
                    "queued operation made destructor join telemetry or dispatch after shutdown");
            waitFor([&] { return robot.destroys == 1; });
            require(robot.destroyDuringRead == 0, "queued operation cleanup raced SDK reader");
        }
        robot.reset();
        {
            CollectionController controller;
            CollectionControllerTestAccess::prepare(controller, temp.path());
            CollectionControllerTestAccess::tools(controller);
            controller.setSelectedTcp("ToolA");
            controller.closeCamera(); // Manual robot motion does not require camera readiness.
            controller.moveTcpTo("120,200,300,180,0,0");
            require(controller.collectionBusy() && controller.manualMoveActive(), "manual move did not lock immediately");
            controller.setSelectedTcp("ToolB");
            require(controller.selectedTcp() == "ToolA", "selected tool changed during dispatch");
            waitFor([&] { return controller.motionFault(); });
            require(controller.collectionBusy() && controller.manualMoveProgress().contains("操作已锁定") && robot.lines == 1,
                    "manual failure did not preserve motion lock");
            controller.disconnectRobot();
            require(robot.destroys == 0, "manual failure allowed disconnect");
            robot.stopWorks = true;
            controller.cancelCollectionOperation();
            waitFor([&] { return !controller.collectionBusy(); });
            require(!controller.motionFault(), "manual stop retry failed to unlock after independent confirmation");
        }
        waitFor([&] { return robot.destroys == 1; });
        robot.reset(); robot.failLine = false; robot.stopWorks = true; robot.stopCode = sucess;
        {
            CollectionController controller;
            CollectionControllerTestAccess::prepare(controller, temp.path());
            CollectionControllerTestAccess::tools(controller);
            controller.setSelectedTcp("ToolA");
            controller.moveTcpTo("120,200,300,180,0,0");
            waitFor([&] { return robot.lines > 0; });
            controller.cancelCollectionOperation();
            waitFor([&] { return !controller.collectionBusy(); });
            require(!controller.motionFault() && robot.stops > 0, "manual cancellation did not stop and verify stationary");
        }
        waitFor([&] { return robot.destroys == 1; });
        robot.reset();
        {
            CollectionController controller;
            CollectionControllerTestAccess::prepare(controller, temp.path());
            CollectionControllerTestAccess::tools(controller);
            controller.setSelectedTcp("ToolA");
            waitFor([&] { return controller.tcpPoseValid(); });
            robot.destroyCode = sdkError;
            controller.disconnectRobot();
            require(controller.collectionBusy() && controller.robotConnected() && !controller.tcpPoseValid(),
                    "disconnect released session before SDK destruction completed");
            waitFor([&] { return !controller.collectionBusy(); });
            require(controller.robotConnected() && robot.destroys == 1 && controller.statusMessage().contains("断开失败"),
                    "failed destruction discarded handle or diagnostic");
            robot.destroyCode = sucess;
            controller.disconnectRobot();
            waitFor([&] { return !controller.robotConnected(); });
            require(robot.destroys == 2 && !controller.collectionBusy(), "failed destruction could not retry safely");
        }
        robot.reset(); robot.createWorks = true; robot.mode = JointPowerOff;
        {
            CollectionController controller;
            controller.connectRobot();
            require(controller.collectionBusy(), "connect did not lock while preparing");
            waitFor([&] { return controller.robotConnected() && !controller.collectionBusy(); });
            require(robot.resets == 1 && robot.powers == 1 && robot.enables == 1 && robot.mode == ProgramStop,
                    "connect no longer preserves power-off to idle to enabled sequence");
        }
        waitFor([&] { return robot.destroys == 1; });
        robot.reset(); robot.createWorks = true; robot.mode = JointPowerOff; robot.powerFails = true;
        {
            CollectionController controller;
            controller.connectRobot();
            waitFor([&] { return controller.robotConnected() && !controller.collectionBusy(); });
            require(robot.resets == 1 && robot.powers == 1 && robot.enables == 0 &&
                    controller.statusMessage().contains("上电失败") && controller.statusMessage().contains("已连接"),
                    "prepare failure enabled robot anyway or lost diagnostic/connection state");
            require(!controller.tcpPoseValid() && controller.tcpPoseStatus().contains("示教器"),
                    "failed prepare fabricated ready telemetry");
        }
        waitFor([&] { return robot.destroys == 1; });
        std::cout << "Controller settings, motion locks/recovery, stale telemetry, generation rejection, nonblocking teardown and 0.1 mm batch passed\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
