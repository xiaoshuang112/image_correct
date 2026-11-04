#include "hand_eye_gui/CollectionController.hpp"
#include <QFile>
#include <QEventLoop>
#include <QTimer>
#include <QTemporaryDir>
#include <stdexcept>

namespace {
void check(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
class SettingsCamera : public AICamera {
public:
    SettingsCamera() : AICamera(CameraConfig{}) {}
    int exposure = 12000;
    bool automatic = true, failNextSet = false, invalidReadback = false;
    std::atomic<bool> blockExposure{false}, releaseExposure{false}, settingExposure{false}, settingsOnGui{false};
    const std::thread::id guiThread = std::this_thread::get_id();
    bool open() override { return true; }
    void close() override {}
    bool isOpened() const override { return true; }
    cv::Mat captureOneFrame(int) override { return {}; }
    bool setColorExposureTime(int value) override {
        settingExposure = true;
        settingsOnGui = std::this_thread::get_id() == guiThread;
        const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        while (blockExposure && !releaseExposure && std::chrono::steady_clock::now() < until)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        settingExposure = false;
        if (failNextSet) { failNextSet = false; return false; }
        exposure = value / 10 * 10; return true;
    }
    int getColorExposureTime() override { return invalidReadback ? -1 : exposure; }
    bool setAutoExposure(bool value) override { automatic = value; return true; }
    bool getAutoExposure() override { return automatic; }
    bool getExposureRange(int& low, int& high) override { low = 100; high = 50000; return true; }
    std::vector<cv::Mat> captureMultiExposureImages(const std::vector<int>&) override { return {}; }
};
QString apply(CollectionController& controller, const QVariantMap& values) {
    QEventLoop loop;
    bool done = false;
    QString error;
    QObject::connect(&controller, &CollectionController::settingsFinished, &loop, [&](const QString& result) {
        done = true; error = result; loop.quit();
    });
    controller.applySettings(values);
    if (!done) {
        QTimer::singleShot(5000, &loop, &QEventLoop::quit);
        loop.exec();
    }
    check(done, "asynchronous settings completion timed out");
    return error;
}
QByteArray readFile(const QString& path) {
    QFile file(path); check(file.open(QIODevice::ReadOnly), "read settings fixture"); return file.readAll();
}
void writeFile(const QString& path, const QByteArray& contents) {
    QFile file(path); check(file.open(QIODevice::WriteOnly), "write settings fixture");
    check(file.write(contents) == contents.size(), "write full settings fixture");
}
}

struct CollectionSettingsTestAccess {
    static void initialize(CollectionController& c, const QString& directory) {
        c.m_appPath = directory.toStdString() + "/";
    }
    static void camera(CollectionController& c, const std::shared_ptr<SettingsCamera>& camera) {
        c.m_camera = camera; c.m_cameraOpened = bool(camera);
    }
    static void busy(CollectionController& c, bool value) { c.m_collectionBusy = value; }
    static void reload(CollectionController& c) { c.loadConfigYaml(); }
};

void testCollectionSettings() {
    QTemporaryDir temporary; check(temporary.isValid(), "settings temporary directory");
    const QString path = temporary.filePath("config.yaml");
    const QByteArray original = "%YAML:1.0\n---\n# production\ncam_ip: \"192.168.66.253\" # keep\n"
        "robot_ip: \"192.168.66.14\"\ncamera_exposure_us: 12000\ncamera_gain: 3\n"
        "fusion_exposures_us: [5000, 8500, 14500, 25000]\n";
    writeFile(path, original);
    CollectionController controller;
    CollectionSettingsTestAccess::initialize(controller, temporary.path());
    check(controller.robotSpeedMmS() == 10, "default movement speed changed");
    for (const char* invalid : {"0", "-1", "1001", "1.5", "nan", "inf", ""}) {
        check(!apply(controller, {{"robotSpeedMmS", invalid}}).isEmpty() && readFile(path) == original
              && controller.robotSpeedMmS() == 10, "invalid speed changed config or controller");
    }
    check(!apply(controller, {{"cameraIp", "10.0.0.40"}, {"exposureList", "5,5"}}).isEmpty()
          && readFile(path) == original && controller.cameraIp() == "192.168.66.253",
          "invalid exposure must reject entire settings page");
    check(!apply(controller, {{"previewWidth", "1.5"}}).isEmpty(), "fractional resolution rejected");
    check(!apply(controller, {{"robotIp", "10.0.0.256"}}).isEmpty(), "invalid IP rejected");
    check(apply(controller, {{"cameraIp", "10.0.0.40"}, {"exposureList", "25000，5000，8500"}}).isEmpty(),
          "valid settings applied");
    check(controller.cameraIp() == "10.0.0.40" && controller.exposureList() == "25000,5000,8500"
          && readFile(path).contains("# production") && readFile(path).contains("camera_gain: 3"),
          "save preserves comments, other values and exposure order");

    auto camera = std::make_shared<SettingsCamera>();
    CollectionSettingsTestAccess::camera(controller, camera);
    camera->blockExposure = true;
    bool responsive = false;
    const auto beforeSlowSave = readFile(path);
    QTimer uiAction;
    uiAction.setInterval(5);
    QObject::connect(&uiAction, &QTimer::timeout, [&] {
        if (!camera->settingExposure) return;
        responsive = !camera->settingsOnGui && controller.settingsSaving() && controller.collectionBusy() &&
            readFile(path) == beforeSlowSave;
        controller.applySettings({{"robotSpeedMmS", 800}}); // Ignore duplicate submissions while saving.
        controller.cancelCollectionOperation(); // Closing must wait for the atomic save/rollback.
        responsive = responsive && controller.collectionBusy();
        camera->releaseExposure = true;
        uiAction.stop();
    });
    uiAction.start();
    check(apply(controller, {{"cameraExposureUs", 12003}}).isEmpty() && responsive &&
          !controller.settingsSaving() && !controller.collectionBusy() && controller.robotSpeedMmS() == 10,
          "slow settings blocked the GUI, accepted duplicate submission, or released lock too early");
    camera->blockExposure = false;
    camera->automatic = true;
    const auto connectedBefore = readFile(path);
    check(!apply(controller, {{"cameraIp", "10.0.0.41"}}).isEmpty()
          && readFile(path) == connectedBefore, "connected camera IP locked");
    CollectionSettingsTestAccess::busy(controller, true);
    check(!apply(controller, {{"robotSpeedMmS", 500}}).isEmpty() && controller.robotSpeedMmS() == 10,
          "busy controller accepted speed change");
    check(!apply(controller, {{"cameraExposureUs", 20000}}).isEmpty() && camera->exposure == 12000,
          "busy prevents hardware/settings update");
    CollectionSettingsTestAccess::busy(controller, false);
    for (int speed : {1, 1000}) {
        check(apply(controller, {{"robotSpeedMmS", speed}}).isEmpty() && controller.robotSpeedMmS() == speed,
              "valid speed not applied");
        CollectionController reloaded;
        CollectionSettingsTestAccess::initialize(reloaded, temporary.path());
        CollectionSettingsTestAccess::reload(reloaded);
        check(reloaded.robotSpeedMmS() == speed, "saved speed lost on restart");
    }
    check(!apply(controller, {{"cameraExposureUs", 51000}}).isEmpty() && camera->exposure == 12000,
          "hardware range checked before apply");
    check(apply(controller, {{"cameraExposureUs", 20003}}).isEmpty()
          && controller.cameraExposureUs() == 20000 && controller.cameraActualExposureUs() == 20000
          && readFile(path).contains("camera_exposure_us: 20000"), "device readback is persisted");
    camera->automatic = true;
    camera->failNextSet = true;
    check(!apply(controller, {{"cameraExposureUs", 30000}}).isEmpty()
          && camera->exposure == 20000 && camera->automatic, "device failure restores exposure and auto mode");
    QByteArray duplicate = readFile(path) + "robot_ip: \"10.0.0.99\"\n";
    writeFile(path, duplicate);
    check(!apply(controller, {{"robotIp", "10.0.0.50"}, {"cameraExposureUs", 30000}, {"robotSpeedMmS", 50}}).isEmpty()
          && readFile(path) == duplicate && controller.robotIp() == "192.168.66.14"
          && controller.cameraExposureUs() == 20000 && camera->exposure == 20000 && camera->automatic && controller.robotSpeedMmS() == 1000,
          "write failure restores camera and leaves all settings unchanged");
    camera->invalidReadback = true;
    check(!apply(controller, {{"cameraExposureUs", 30000}}).isEmpty()
          && camera->exposure == 20000 && readFile(path) == duplicate, "no writes after invalid device readback");
    CollectionSettingsTestAccess::camera(controller, {});
}
