#include "hand_eye_gui/CollectionController.hpp"
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QTemporaryDir>
#include <algorithm>
#include <array>
#include <atomic>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <thread>

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
    CRresult stopCode = sdkError;
    bool failLine = true;
    void reset() {
        pose = {100, 200, 300, 180, 0, 0};
        moving = stopWorks = statusFails = false;
        lines = stops = destroys = unexpected = 0;
        stopCode = sdkError; failLine = true;
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
};

extern "C" CRresult cr_get_robotMode(RobotHandle, RobotModes* mode) {
    *mode = robot.moving ? SDK_Moving : Enable;
    return sucess;
}
extern "C" CRresult cr_get_robotMoveStatus(RobotHandle, BOOL* moving) {
    *moving = robot.moving;
    return robot.statusFails ? sdkError : sucess;
}
extern "C" CRresult cr_get_AssignTCP_Pose(RobotHandle, double* tool, double* pose) {
    if (!std::all_of(tool, tool + ROB_AXIS_NUM, [](double v) { return v == 0; })) return sdkError;
    std::copy(robot.pose.begin(), robot.pose.end(), pose);
    return sucess;
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
    return sucess;
}
extern "C" CRresult cr_stop(RobotHandle) {
    ++robot.stops;
    if (robot.stopWorks) robot.moving = false;
    return robot.stopCode;
}
extern "C" CRresult cr_destroy_robot(RobotHandle) { ++robot.destroys; return sucess; }
extern "C" CRresult cr_create_robot(RobotHandle*, const char*, int, const char*) { ++robot.unexpected; return sdkError; }
extern "C" CRresult cr_FaultReset(RobotHandle) { ++robot.unexpected; return sdkError; }
extern "C" CRresult cr_poweron(RobotHandle) { ++robot.unexpected; return sdkError; }
extern "C" CRresult cr_enable(RobotHandle) { ++robot.unexpected; return sdkError; }
extern "C" CRresult cr_kineForward(RobotHandle, double*, double*) { ++robot.unexpected; return sdkError; }
extern "C" CRresult cr_move_joint(RobotHandle, PointControlPara, BOOL) { ++robot.unexpected; return sdkError; }
extern "C" CRresult cr_move_pointControlPara_transfer(RobotHandle, PointControlParaSimple, PointControlPara*) {
    ++robot.unexpected; return sdkError;
}
extern "C" CRresult cr_get_allTCPmsg(RobotHandle, TCPMsg*, int*) { ++robot.unexpected; return sdkError; }
extern "C" CRresult cr_get_currentTCPmsg(RobotHandle, TCPMsg*) { ++robot.unexpected; return sdkError; }
extern "C" CRresult cr_cfg_tcp_accord_name_get(RobotHandle, char*, TCPMsg*) { ++robot.unexpected; return sdkError; }

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);
    try {
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
            controller.capturePhoto(); controller.captureFivePoints(10, 10); controller.levelCamera();
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
        require(robot.destroys == 1 && robot.unexpected == 0, "normal cleanup failed or real SDK path was reached");
        std::cout << "Real controller stop-failure lock, recovery, teardown guard and 0.1 mm batch passed\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
