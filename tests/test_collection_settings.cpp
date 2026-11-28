#include "hand_eye_gui/CollectionController.hpp"
#include "utils/collection_undistortion.hpp"
#include <QDir>
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
    static void camera2(CollectionController& c, const std::shared_ptr<SettingsCamera>& camera) {
        c.m_camera2 = camera; c.m_camera2Opened = bool(camera);
    }
    static void busy(CollectionController& c, bool value) { c.m_collectionBusy = value; }
    static void reload(CollectionController& c) { c.loadConfigYaml(); }
    static const std::string& appPath(const CollectionController& c) { return c.m_appPath; }
    static void setupProjectionCameras(CollectionController& c, const QString& calibrationFile) {
        c.m_cameraOpened = c.m_camera2Opened = c.m_dualCameraReady = true;
        c.m_camActualWidth = c.m_cam2ActualWidth = 320;
        c.m_camActualHeight = c.m_cam2ActualHeight = 240;
        c.m_undistortionCalibration = c.m_undistortionCalibration2 =
            std::make_shared<collection::UndistortionCalibration>(
                collection::UndistortionCalibration::load(calibrationFile.toStdString()));
    }
    static void setSecondCameraWidth(CollectionController& c, int width) { c.m_cam2ActualWidth = width; }
    static auto maps(const CollectionController& c) { return std::make_pair(c.m_undistortionMaps, c.m_undistortionMaps2); }
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

    const QByteArray savedCamera2 = "cam2_ip: \"10.0.0.40\" # retained\ncam2_sn: \"OLD\"\n"
        "cam2_width: 2000\ncam2_height: 1500\ncamera2_exposure_us: 11000\ncamera2_gain: 4\n"
        "camera2_fusion_exposures_us: [4000, 8000]\n";
    writeFile(path, original + savedCamera2);
    CollectionController singleSettings;
    CollectionSettingsTestAccess::initialize(singleSettings, temporary.path());
    CollectionSettingsTestAccess::reload(singleSettings);
    check(!singleSettings.dualCameraMode(), "settings regression starts in single-camera mode");
    check(apply(singleSettings, {{"cameraIp", "10.0.0.40"}}).isEmpty() &&
          singleSettings.cameraIp() == "10.0.0.40",
          "stored camera2 IP conflict does not block single-camera settings");
    check(apply(singleSettings, {{"robotSpeedMmS", 50}, {"camera2Ip", "invalid"}, {"camera2Sn", "NEW"},
                                {"preview2Width", -1}, {"preview2Height", 34567}, {"camera2ExposureUs", 0},
                                {"camera2Gain", -1}, {"exposure2List", "100,100"}}).isEmpty() &&
          singleSettings.robotSpeedMmS() == 50,
          "invalid camera2 draft does not block single-camera settings");
    auto inactiveCamera2 = std::make_shared<SettingsCamera>();
    CollectionSettingsTestAccess::camera2(singleSettings, inactiveCamera2);
    check(apply(singleSettings, {{"robotSpeedMmS", 60}, {"camera2Ip", "10.0.0.40"},
                                {"camera2ExposureUs", 51000}, {"exposure2List", "100,90000"}}).isEmpty() &&
          singleSettings.robotSpeedMmS() == 60 && inactiveCamera2->exposure == 12000 && inactiveCamera2->automatic,
          "single-camera save skips camera2 device validation and updates");
    CollectionSettingsTestAccess::camera2(singleSettings, {});
    check(singleSettings.camera2Ip() == "10.0.0.40" && singleSettings.camera2Sn() == "OLD" &&
          singleSettings.preview2Width() == 2000 && singleSettings.preview2Height() == 1500 &&
          singleSettings.camera2ExposureUs() == 11000 && singleSettings.camera2Gain() == 4 &&
          singleSettings.exposure2List() == "4000,8000" && readFile(path).contains(savedCamera2),
          "single-camera saves preserve camera2 settings in memory and YAML");
    singleSettings.setDualCameraMode(true);
    const auto savedSingleSettings = readFile(path);
    check(singleSettings.dualCameraMode() &&
          !apply(singleSettings, {{"robotSpeedMmS", 70}}).isEmpty() && readFile(path) == savedSingleSettings,
          "switching to dual-camera mode restores validation of the stored camera2 IP");

    writeFile(path, original);
    CollectionController dualSettings;
    CollectionSettingsTestAccess::initialize(dualSettings, temporary.path());
    CollectionSettingsTestAccess::reload(dualSettings);
    dualSettings.setDualCameraMode(true);
    check(dualSettings.dualCameraMode(), "dual settings regression starts in dual-camera mode");
    check(!apply(dualSettings, {{"camera2Ip", "10.0.0.40"}, {"cameraIp", "10.0.0.40"}}).isEmpty(),
          "duplicate camera IP rejected");
    check(apply(dualSettings, {{"camera2Ip", "10.0.0.41"}, {"camera2Sn", "ABC"}, {"preview2Width", "2000"},
                              {"preview2Height", "1500"}, {"camera2ExposureUs", 11000}, {"camera2Gain", 4},
                              {"exposure2List", "4000,8000"}}).isEmpty(),
          "camera2 settings saved");
    check(dualSettings.camera2Ip() == "10.0.0.41" && dualSettings.camera2Sn() == "ABC" &&
          dualSettings.preview2Width() == 2000 && dualSettings.preview2Height() == 1500 &&
          dualSettings.camera2Gain() == 4 && dualSettings.exposure2List() == "4000,8000" &&
          readFile(path).contains("cam2_ip: \"10.0.0.41\""),
          "camera2 yaml persisted");
    auto cam2 = std::make_shared<SettingsCamera>();
    CollectionSettingsTestAccess::camera2(dualSettings, cam2);
    const auto beforeConnected2 = readFile(path);
    check(!apply(dualSettings, {{"camera2Ip", "10.0.0.42"}}).isEmpty() && readFile(path) == beforeConnected2,
          "connected camera2 IP locked");
    check(!apply(dualSettings, {{"camera2Gain", 5}}).isEmpty() && readFile(path) == beforeConnected2,
          "connected camera2 gain locked");
    check(!apply(dualSettings, {{"camera2ExposureUs", 51000}}).isEmpty() && readFile(path) == beforeConnected2 &&
          cam2->exposure == 12000 && cam2->automatic,
          "connected camera2 exposure range checked before apply");
    check(!apply(dualSettings, {{"exposure2List", "100,90000"}}).isEmpty() && readFile(path) == beforeConnected2,
          "connected camera2 fusion exposure range checked before apply");
    check(apply(dualSettings, {{"camera2ExposureUs", 15003}}).isEmpty() &&
          dualSettings.camera2ExposureUs() == 15000 && dualSettings.camera2ActualExposureUs() == 15000,
          "connected camera2 live exposure updated and read back");
    CollectionSettingsTestAccess::camera2(dualSettings, {});

    writeFile(path, original);
    CollectionController projectionSettings;
    CollectionSettingsTestAccess::initialize(projectionSettings, temporary.path());
    CollectionSettingsTestAccess::reload(projectionSettings);
    check(projectionSettings.undistortionProjection() == "opencv_alpha1_v1",
          "existing configs must keep the historical projection");
    writeFile(path, original + "undistortion_projection: \"incorrect\"\n");
    CollectionSettingsTestAccess::reload(projectionSettings);
    check(projectionSettings.undistortionProjection().isEmpty(),
          "invalid production config must not silently select historical projection");
    check(apply(projectionSettings, {{"undistortionProjection", "square_pixels_v1"}}).isEmpty(),
          "valid projection must repair the invalid config");
    CollectionSettingsTestAccess::reload(projectionSettings);
    check(projectionSettings.undistortionProjection() == "square_pixels_v1", "saved projection did not reload");
    projectionSettings.setDualCameraMode(true);
    const QString calibration = temporary.filePath("projection_calibration.yaml");
    writeFile(calibration,
        "%YAML:1.0\n---\nimage_width: 320\nimage_height: 240\n"
        "camera_matrix: !!opencv-matrix\n   rows: 3\n   cols: 3\n   dt: d\n"
        "   data: [180., 0., 160., 0., 190., 120., 0., 0., 1.]\n"
        "distortion_coefficients: !!opencv-matrix\n   rows: 1\n   cols: 5\n   dt: d\n"
        "   data: [-0.24, 0.03, 0., 0., 0.]\n");
    CollectionSettingsTestAccess::setupProjectionCameras(projectionSettings, calibration);
    projectionSettings.toggleUndistortion();
    projectionSettings.capturePhoto();
    check(projectionSettings.statusMessage().contains("先启用两路相机的去畸变") &&
          !projectionSettings.collectionBusy(), "production accepted a pair with one raw preview");
    projectionSettings.toggleUndistortion2();
    const auto oldMaps = CollectionSettingsTestAccess::maps(projectionSettings);
    check(oldMaps.first && oldMaps.second &&
          oldMaps.first->geometry().projection == collection::PixelProjection::SquarePixels &&
          oldMaps.second->geometry().projection == collection::PixelProjection::SquarePixels,
          "both camera toggles must use the same production mode");
    const auto beforeFailedProjection = readFile(path);
    CollectionSettingsTestAccess::setSecondCameraWidth(projectionSettings, 321);
    check(!apply(projectionSettings, {{"undistortionProjection", "opencv_alpha1_v1"}}).isEmpty() &&
          readFile(path) == beforeFailedProjection && CollectionSettingsTestAccess::maps(projectionSettings) == oldMaps,
          "a bad map must not persist a new mode or partially replace a camera map");
    CollectionSettingsTestAccess::setSecondCameraWidth(projectionSettings, 320);
    check(apply(projectionSettings, {{"undistortionProjection", "opencv_alpha1_v1"}}).isEmpty(),
          "switching projection with active maps failed");
    const auto newMaps = CollectionSettingsTestAccess::maps(projectionSettings);
    check(newMaps.first && newMaps.second && newMaps.first != oldMaps.first && newMaps.second != oldMaps.second &&
          newMaps.first->geometry().projection == collection::PixelProjection::Alpha1 &&
          newMaps.second->geometry().projection == collection::PixelProjection::Alpha1,
          "successful save must replace both maps with the new projection");

    // m_appPath 保存的是 UTF-8（生产里来自 exePath.toStdString()）。Unicode 目录必须按
    // UTF-8 解码读取；按 ANSI 代码页（此机器 936/GBK）解码会指向另一个不存在的名字，
    // 于是启动读取错误地回退到默认值。
    const QString unicodeName = QStringLiteral("测试配置_手眼目录");
    const QString unicodeDir = temporary.filePath(unicodeName);
    check(QDir().mkpath(unicodeDir), "unicode settings directory");
    const QString unicodeCfg = unicodeDir + QStringLiteral("/config.yaml");
    const QByteArray unicodeOriginal = "%YAML:1.0\n---\n# 保留注释\nrobot_speed_mm_s: 250\ncam_ip: \"10.0.0.70\"\n";
    writeFile(unicodeCfg, unicodeOriginal);

    // 同一 UTF-8 名字按 ANSI 代码页解码后的诱饵：保存与读取都不能碰它。
    const QString ansiName = QString::fromLocal8Bit(unicodeName.toUtf8());
    const bool ansiDecoy = ansiName != unicodeName;
    QString ansiCfg;
    bool decoyWritten = false;
    if (ansiDecoy) {
        const QString ansiDir = temporary.filePath(ansiName);
        ansiCfg = ansiDir + QStringLiteral("/config.yaml");
        if (QDir().mkpath(ansiDir)) {
            QFile decoy(ansiCfg);
            if (decoy.open(QIODevice::WriteOnly) &&
                decoy.write("%YAML:1.0\n---\n# ansi\nrobot_speed_mm_s: 300\n") > 0) {
                decoy.close();
                decoyWritten = true;
            }
        }
    }

    CollectionController unicode;
    CollectionSettingsTestAccess::initialize(unicode, unicodeDir);
    check(QString::fromUtf8(CollectionSettingsTestAccess::appPath(unicode).c_str()) ==
          unicodeDir + QStringLiteral("/"), "m_app_path_round_trips_as_utf8");
    CollectionSettingsTestAccess::reload(unicode);
    check(unicode.robotSpeedMmS() == 250 && unicode.cameraIp() == "10.0.0.70",
          "unicode config read through utf8 path");

    check(apply(unicode, {{"robotSpeedMmS", 400}}).isEmpty(),
          "unicode config save through utf8 path");
    const QByteArray unicodeSaved = readFile(unicodeCfg);
    check(unicodeSaved.contains("robot_speed_mm_s: 400") &&
          unicodeSaved.contains("# 保留注释") &&
          unicodeSaved.contains("cam_ip: \"10.0.0.70\""),
          "unicode save keeps utf8 name, comment and other keys");
    if (decoyWritten)
        check(readFile(ansiCfg).contains("robot_speed_mm_s: 300"),
              "unicode save does not write the ansi decoded name");

    CollectionController restarted;
    CollectionSettingsTestAccess::initialize(restarted, unicodeDir);
    CollectionSettingsTestAccess::reload(restarted);
    check(restarted.robotSpeedMmS() == 400, "unicode saved speed read back after restart");

    // 缺失与格式非法仍然回退默认值，不能因为改用内存解析而改变。
    check(QFile::remove(unicodeCfg), "remove unicode config for fallback");
    CollectionController missing;
    CollectionSettingsTestAccess::initialize(missing, unicodeDir);
    CollectionSettingsTestAccess::reload(missing);
    check(missing.robotSpeedMmS() == 10 && missing.robotIp() == "192.168.66.14",
          "unicode missing config falls back to defaults");
    writeFile(unicodeCfg, "not a YAML file : [");
    CollectionController malformed;
    CollectionSettingsTestAccess::initialize(malformed, unicodeDir);
    CollectionSettingsTestAccess::reload(malformed);
    check(malformed.robotSpeedMmS() == 10 && malformed.robotIp() == "192.168.66.14",
          "unicode malformed config falls back to defaults");
}
