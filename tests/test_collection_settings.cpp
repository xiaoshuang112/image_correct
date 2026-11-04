#include "hand_eye_gui/CollectionController.hpp"
#include <QFile>
#include <QTemporaryDir>
#include <stdexcept>

namespace {
void check(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
class SettingsCamera : public AICamera {
public:
    SettingsCamera() : AICamera(CameraConfig{}) {}
    int exposure = 12000;
    bool automatic = true, failNextSet = false, invalidReadback = false;
    bool open() override { return true; }
    void close() override {}
    bool isOpened() const override { return true; }
    cv::Mat captureOneFrame(int) override { return {}; }
    bool setColorExposureTime(int value) override {
        if (failNextSet) { failNextSet = false; return false; }
        exposure = value / 10 * 10; return true;
    }
    int getColorExposureTime() override { return invalidReadback ? -1 : exposure; }
    bool setAutoExposure(bool value) override { automatic = value; return true; }
    bool getAutoExposure() override { return automatic; }
    bool getExposureRange(int& low, int& high) override { low = 100; high = 50000; return true; }
    std::vector<cv::Mat> captureMultiExposureImages(const std::vector<int>&) override { return {}; }
};
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
    check(!controller.applySettings({{"cameraIp", "10.0.0.40"}, {"exposureList", "5,5"}}).isEmpty()
          && readFile(path) == original && controller.cameraIp() == "192.168.66.253",
          "invalid exposure must reject entire settings page");
    check(!controller.applySettings({{"previewWidth", "1.5"}}).isEmpty(), "fractional resolution rejected");
    check(!controller.applySettings({{"robotIp", "10.0.0.256"}}).isEmpty(), "invalid IP rejected");
    check(controller.applySettings({{"cameraIp", "10.0.0.40"}, {"exposureList", "25000，5000，8500"}}).isEmpty(),
          "valid settings applied");
    check(controller.cameraIp() == "10.0.0.40" && controller.exposureList() == "25000,5000,8500"
          && readFile(path).contains("# production") && readFile(path).contains("camera_gain: 3"),
          "save preserves comments, other values and exposure order");

    auto camera = std::make_shared<SettingsCamera>();
    CollectionSettingsTestAccess::camera(controller, camera);
    const auto connectedBefore = readFile(path);
    check(!controller.applySettings({{"cameraIp", "10.0.0.41"}}).isEmpty()
          && readFile(path) == connectedBefore, "connected camera IP locked");
    CollectionSettingsTestAccess::busy(controller, true);
    check(!controller.applySettings({{"cameraExposureUs", 20000}}).isEmpty() && camera->exposure == 12000,
          "busy prevents hardware/settings update");
    CollectionSettingsTestAccess::busy(controller, false);
    check(!controller.applySettings({{"cameraExposureUs", 51000}}).isEmpty() && camera->exposure == 12000,
          "hardware range checked before apply");
    check(controller.applySettings({{"cameraExposureUs", 20003}}).isEmpty()
          && controller.cameraExposureUs() == 20000 && controller.cameraActualExposureUs() == 20000
          && readFile(path).contains("camera_exposure_us: 20000"), "device readback is persisted");
    camera->automatic = true;
    camera->failNextSet = true;
    check(!controller.applySettings({{"cameraExposureUs", 30000}}).isEmpty()
          && camera->exposure == 20000 && camera->automatic, "device failure restores exposure and auto mode");
    QByteArray duplicate = readFile(path) + "robot_ip: \"10.0.0.99\"\n";
    writeFile(path, duplicate);
    check(!controller.applySettings({{"robotIp", "10.0.0.50"}, {"cameraExposureUs", 30000}}).isEmpty()
          && readFile(path) == duplicate && controller.robotIp() == "192.168.66.14"
          && controller.cameraExposureUs() == 20000 && camera->exposure == 20000 && camera->automatic,
          "write failure restores camera and leaves all settings unchanged");
    camera->invalidReadback = true;
    check(!controller.applySettings({{"cameraExposureUs", 30000}}).isEmpty()
          && camera->exposure == 20000 && readFile(path) == duplicate, "no writes after invalid device readback");
    CollectionSettingsTestAccess::camera(controller, {});
}
