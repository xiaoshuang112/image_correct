#include <QGuiApplication>
#include <QDir>
#include <QQmlApplicationEngine>
#include <QQmlComponent>
#include <QQmlContext>
#include <QQuickImageProvider>
#include <QQuickItem>
#include <QQuickWindow>
#include <QtTest/QTest>
#include <QtTest/QSignalSpy>
#include <qqml.h>
#include <iostream>
#include <functional>
#include <memory>
#include <stdexcept>

// Match the real controller's shared NOTIFY signal: unrelated/busy changes
// notify savedImageCount even when its getter returns the same value.
class SharedCollectionState : public QObject {
    Q_OBJECT
    Q_PROPERTY(int savedImageCount READ savedImageCount WRITE setSavedImageCount NOTIFY collectionChanged)
    Q_PROPERTY(bool collectionBusy READ collectionBusy WRITE setCollectionBusy NOTIFY collectionChanged)
public:
    using QObject::QObject;
    int savedImageCount() const { return m_count; }
    bool collectionBusy() const { return m_busy; }
    void setSavedImageCount(int value) { if (value != m_count) { m_count = value; emit collectionChanged(); } }
    void setCollectionBusy(bool value) { if (value != m_busy) { m_busy = value; emit collectionChanged(); } }
    Q_INVOKABLE void beginCapture() { m_busy = true; emit collectionChanged(); }
    Q_INVOKABLE void finishCapture(bool saved) {
        m_busy = false;
        if (saved) ++m_count;
        emit collectionChanged();
    }
signals:
    void collectionChanged();
private:
    int m_count = 0;
    bool m_busy = false;
};

namespace {
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
class Preview : public QQuickImageProvider {
public:
    Preview() : QQuickImageProvider(QQuickImageProvider::Image) {}
    QImage requestImage(const QString&, QSize* size, const QSize&) override {
        QImage image(640, 480, QImage::Format_RGB32);
        image.fill(QColor("#141c27"));
        if (size) *size = image.size();
        return image;
    }
};
}

int main(int argc, char** argv) {
    if (qEnvironmentVariableIsEmpty("QT_QPA_PLATFORM"))
        qputenv("QT_QPA_PLATFORM", "offscreen");
    qputenv("QT_QUICK_BACKEND", "software");
    QGuiApplication app(argc, argv);
    qmlRegisterType<SharedCollectionState>("CollectTest", 1, 0, "SharedCollectionState");
    try {
        std::unique_ptr<QObject> controller;
        QQmlApplicationEngine engine;
        QQmlComponent mock(&engine);
        mock.setData(R"QML(
import QtQuick 2.15
import CollectTest 1.0
SharedCollectionState {
    property bool cameraOpened: false
    property bool previewActive: false
    property bool robotConnected: false
    property bool undistortionReady: false
    property bool undistortionEnabled: false
    property string undistortionStatus: "未找到手眼文件，去畸变不可用"
    property string cameraBackend: "mvs"
    property string cameraIp: "192.168.66.253"
    property int previewWidth: 4024
    property int previewHeight: 3036
    property string cameraModel: "MVS"
    property int cameraActualWidth: 4024
    property int cameraActualHeight: 3036
    property int cameraExposureUs: 12000
    property string outputDirectory: "/tmp/collect-ui-check"
    property string exposureList: "5000,8500,14500,25000"
    property string lastCapturePath: ""
    property int savedPoseCount: 0
    property string pointName: ""
    property bool poseFailure: false
    property string robotIp: "192.168.66.14"
    property int robotPort: 2323
    property string robotPasswd: "123"
    property string handEyePath: "/tmp/hand_eye_result.yaml"
    property string handEyeUnits: "m"
    property string statusMessage: "界面检查"
    signal frameUpdated()
    function openCamera() { cameraOpened = true }
    function closeCamera() { previewActive = false; cameraOpened = false; undistortionEnabled = false }
    function startPreview() { previewActive = true; frameUpdated() }
    function stopPreview() { previewActive = false }
    property bool captureFailure: false
    property bool captureCancelled: false
    function capturePhoto() {
        beginCapture() // beginCollectionOperation notifies before increment
        finishCapture(!captureFailure && !captureCancelled) // one shared completion NOTIFY
    }
    function openCollectionFolder() {}
    function saveFlangePose(name) {
        if (collectionBusy || !robotConnected) return
        pointName = name
        if (poseFailure) { statusMessage = "法兰位姿读取失败，未保存点位文件"; return }
        savedPoseCount++
        statusMessage = "点位已保存：pose_" + (name || "20261015_123456_001") + ".json"
    }
    function adjustCameraExposure(value) { cameraExposureUs = value }
    function connectRobot() { robotConnected = true }
    function disconnectRobot() { robotConnected = false }
    function levelCamera() {}
    function cancelCollectionOperation() { collectionBusy = false }
    function toggleUndistortion() { undistortionEnabled = !undistortionEnabled }
}
)QML", QUrl());
        controller.reset(mock.create());
        require(controller != nullptr, "mock controller failed");
        engine.rootContext()->setContextProperty("collectionController", controller.get());
        engine.rootContext()->setContextProperty("appVersion", "UI test");
        engine.rootContext()->setContextProperty("appGitCommit", "123abcd");
        engine.addImageProvider("camera", new Preview);
        engine.load(QUrl("qrc:///qml/Collection.qml"));
        require(!engine.rootObjects().isEmpty(), "collection QML did not load");
        auto* window = qobject_cast<QQuickWindow*>(engine.rootObjects().front());
        require(window != nullptr, "missing collection window");
        window->requestActivate();
        require(QTest::qWaitForWindowActive(window), "collection window did not become active");
        QTest::qWait(100);
        std::function<QQuickItem*(QQuickItem*, const QString&)> findVisual =
            [&](QQuickItem* parent, const QString& name) -> QQuickItem* {
                if (parent->objectName() == name) return parent;
                for (auto* child : parent->childItems())
                    if (auto* found = findVisual(child, name)) return found;
                return nullptr;
            };
        auto item = [&](const char* name) {
            auto* result = findVisual(window->contentItem(), QString::fromLatin1(name));
            if (!result) throw std::runtime_error(std::string("missing UI action: ") + name);
            return result;
        };
        auto* imageArea = item("captureImageArea");
        auto* flashOverlay = item("shutterFlashOverlay");
        auto* flashAnimation = window->findChild<QObject*>("shutterFlashAnimation");
        require(flashAnimation != nullptr, "missing shutter animation");
        QSignalSpy flashes(flashAnimation, SIGNAL(started()));
        require(flashes.isValid(), "cannot observe shutter animation starts");
        auto notify = [&](QObject* state) {
            require(QMetaObject::invokeMethod(state, "collectionChanged"), "shared NOTIFY failed");
        };
        auto take = [&](QObject* state) {
            require(QMetaObject::invokeMethod(state, "capturePhoto"), "mock capture failed");
        };
        require(imageArea->property("seenCount").toInt() == 0 && flashes.count() == 0,
                "initial count should establish history without flashing");
        notify(controller.get());
        require(flashes.count() == 0, "unchanged shared NOTIFY flashed before first capture");
        take(controller.get());
        require(flashes.count() == 1 && flashAnimation->property("running").toBool(),
                "first successful capture did not start shutter animation");
        take(controller.get());
        require(flashes.count() == 2, "consecutive capture did not restart shutter animation");
        notify(controller.get());
        require(flashes.count() == 2, "unchanged count restarted shutter animation");
        controller->setProperty("captureFailure", true);
        take(controller.get());
        require(flashes.count() == 2 && controller->property("savedImageCount").toInt() == 2,
                "failed capture changed count or flashed");
        controller->setProperty("captureFailure", false);
        controller->setProperty("captureCancelled", true);
        take(controller.get());
        require(flashes.count() == 2 && controller->property("savedImageCount").toInt() == 2,
                "cancelled capture changed count or flashed");
        controller->setProperty("captureCancelled", false);
        controller->setProperty("savedImageCount", 0);
        require(imageArea->property("seenCount").toInt() == 0 && flashes.count() == 2 &&
                !flashAnimation->property("running").toBool() && qFuzzyIsNull(flashOverlay->opacity()),
                "count reset did not clear flash/history");
        notify(controller.get());
        take(controller.get());
        require(flashes.count() == 3, "first capture after reset did not flash exactly once");
        std::unique_ptr<QObject> replacement(mock.create());
        require(replacement != nullptr, "replacement controller failed");
        replacement->setProperty("savedImageCount", 9);
        window->setProperty("controller", QVariant::fromValue(replacement.get()));
        require(imageArea->property("seenCount").toInt() == 9 && flashes.count() == 3 &&
                !flashAnimation->property("running").toBool() && qFuzzyIsNull(flashOverlay->opacity()),
                "controller switch flashed historical captures");
        notify(replacement.get());
        take(replacement.get());
        require(flashes.count() == 4, "capture on replacement controller failed to flash");
        take(controller.get());
        require(flashes.count() == 4, "old controller changed active flash history");
        window->setProperty("controller", QVariant::fromValue(controller.get()));
        require(imageArea->property("seenCount").toInt() == 2 && flashes.count() == 4,
                "restoring controller did not reset history baseline");
        controller->setProperty("savedImageCount", 0);
        flashes.clear();
        auto count = [&]() { return controller->property("savedImageCount").toInt(); };
        auto* open = item("openCameraButton");
        auto* capture = item("captureButton");
        auto* level = item("levelButton");
        auto* input = item("exposureInput");
        auto* savePose = item("savePoseButton");
        auto* undistortion = item("undistortionButton");
        require(!undistortion->isEnabled(), "undistortion enabled without calibration or camera");
        auto* pointName = item("pointNameInput");
        require(!savePose->isEnabled(), "pose enabled without robot");
        require(item("exposureInput5")->property("text").toString().isEmpty(), "fifth exposure is not blank");
        require(findVisual(window->contentItem(), "exposureInput6") == nullptr, "more than five exposure inputs");
        require(!capture->isEnabled() && !level->isEnabled(), "actions enabled before camera preview");
        QTest::keyClick(window, Qt::Key_Space);
        require(count() == 0, "captured with closed camera");
        const QPoint openPoint = open->mapToScene(QPointF(open->width()/2, open->height()/2)).toPoint();
        QTest::mouseClick(window, Qt::LeftButton, Qt::NoModifier, openPoint);
        QTest::qWait(30);
        require(controller->property("previewActive").toBool() && capture->isEnabled(), "opening camera did not start preview");
        require(!level->isEnabled(), "levelling enabled without robot");
        QTest::keyClick(window, Qt::Key_Space);
        require(count() == 1 && controller->property("cameraOpened").toBool(), "Space failed or activated focused close button");
        QTest::keyClick(window, Qt::Key_S);
        require(count() == 2, "S shortcut failed");
        input->forceActiveFocus();
        QTest::keyClick(window, Qt::Key_Space);
        QTest::keyClick(window, Qt::Key_S);
        require(count() == 2, "typing in input triggered capture");
        QTest::keyClick(window, Qt::Key_S, Qt::ControlModifier);
        require(count() == 2, "Ctrl+S captured while editing input");
        input->setFocus(false);
        window->contentItem()->forceActiveFocus();
        QTest::qWait(20);
        QTest::keyClick(window, Qt::Key_S, Qt::ControlModifier);
        QTest::qWait(20);
        require(count() == 3, "Ctrl+S failed outside input");
        controller->setProperty("collectionBusy", true);
        QTest::qWait(10);
        QTest::keyClick(window, Qt::Key_S, Qt::ControlModifier);
        require(count() == 3 && !capture->isEnabled(), "captured during another operation");
        controller->setProperty("collectionBusy", false);
        controller->setProperty("robotConnected", true);
        QTest::qWait(10);
        require(level->isEnabled(), "level action not enabled with preview and robot");
        auto click = [&](QQuickItem* button) {
            QTest::mouseClick(window, Qt::LeftButton, Qt::NoModifier,
                button->mapToScene(QPointF(button->width()/2, button->height()/2)).toPoint());
            QTest::qWait(30);
        };
        auto* previewExposure = item("previewExposureInput");
        auto* applyExposure = item("applyPreviewExposureButton");
        previewExposure->setProperty("text", "18000");
        click(applyExposure);
        require(controller->property("cameraExposureUs").toInt() == 18000,
                "moved preview exposure action did not update controller");
        controller->setProperty("collectionBusy", true);
        QTest::qWait(10);
        require(!applyExposure->isEnabled() && !previewExposure->isEnabled() &&
                item("stopButton")->isEnabled(), "busy exposure gate or stop action changed");
        click(item("stopButton"));
        require(!controller->property("collectionBusy").toBool(), "stop action did not cancel collection");
        require(!undistortion->isEnabled(), "undistortion enabled with missing calibration");
        controller->setProperty("undistortionReady", true);
        QTest::qWait(10);
        require(undistortion->isEnabled(), "undistortion unavailable with camera and calibration");
        click(undistortion);
        require(controller->property("undistortionEnabled").toBool() &&
                undistortion->property("text").toString() == "去畸变已启用", "undistortion switch not visibly enabled");
        require(item("undistortionStatus")->property("text").toString().contains("预览 / 拍照"),
                "undistortion status does not describe capture mode");
        click(item("previewButton"));
        require(!controller->property("previewActive").toBool() && !capture->isEnabled() &&
                controller->property("undistortionEnabled").toBool(),
                "pause lost correction state or left capture enabled");
        click(item("previewButton"));
        require(capture->isEnabled(), "preview did not resume after pause");
        controller->setProperty("collectionBusy", true);
        QTest::qWait(10);
        require(!undistortion->isEnabled(), "undistortion can change during capture");
        controller->setProperty("collectionBusy", false);
        QTest::qWait(10);
        click(undistortion);
        require(!controller->property("undistortionEnabled").toBool(), "undistortion cannot be disabled");
        click(savePose);
        require(window->property("poseEditorOpen").toBool() && pointName->property("text").toString().isEmpty(),
                "pose editor did not open with an empty name");
        QTest::keyClick(window, Qt::Key_S, Qt::ControlModifier);
        require(count() == 3, "pose name editing triggered photo");
        pointName->setProperty("text", "工位_A");
        QTest::keyClick(window, Qt::Key_Return);
        QTest::qWait(20);
        require(controller->property("savedPoseCount").toInt() == 1 && count() == 3 &&
                controller->property("pointName").toString() == "工位_A", "named pose saved wrong data or captured image");
        require(!window->property("poseEditorOpen").toBool(), "successful point save left editor open");
        click(savePose);
        QTest::keyClick(window, Qt::Key_Return);
        QTest::qWait(20);
        require(controller->property("savedPoseCount").toInt() == 2 &&
                controller->property("pointName").toString().isEmpty(), "empty name was not passed for timestamp naming");
        click(savePose);
        pointName->setProperty("text", "保留失败点位");
        controller->setProperty("poseFailure", true);
        QTest::keyClick(window, Qt::Key_Return);
        QTest::qWait(20);
        require(window->property("poseEditorOpen").toBool() &&
                pointName->property("text").toString() == "保留失败点位" &&
                controller->property("savedPoseCount").toInt() == 2 && count() == 3,
                "failed pose save lost name, counted a point, or captured an image");
        QTest::keyClick(window, Qt::Key_Escape);
        require(!window->property("poseEditorOpen").toBool(), "Escape did not close point editor");
        window->resize(1024, 740);
        QTest::qWait(50);
        for (auto* action : {open, capture, level, savePose, undistortion, previewExposure, applyExposure,
                             item("previewButton"), item("exposureInput5")}) {
            const QRectF bounds(action->mapToScene(QPointF(0, 0)), QSizeF(action->width(), action->height()));
            if (!QRectF(0, 0, window->width(), window->height()).contains(bounds))
                std::cerr << action->objectName().toStdString() << " bounds " << bounds.x() << ',' << bounds.y()
                          << ' ' << bounds.width() << 'x' << bounds.height() << '\n';
            require(QRectF(0, 0, window->width(), window->height()).contains(bounds), "action clipped at minimum window size");
        }
        auto bounds = [](QQuickItem* target) {
            return QRectF(target->mapToScene(QPointF()), QSizeF(target->width(), target->height()));
        };
        require(!bounds(undistortion).intersects(bounds(item("previewButton"))),
                "undistortion and preview controls overlap at minimum window size");
        require(!bounds(capture).intersects(bounds(savePose)) &&
                !bounds(item("exposureInput5")).intersects(bounds(capture)),
                "capture controls overlap at minimum window size");
        require(qAbs(bounds(open).left() - bounds(item("connectRobotButton")).left()) < 1.0,
                "camera and robot connect buttons are not aligned");
        auto checkCards = [&]() {
            const auto device = bounds(item("deviceCard"));
            const auto calibration = bounds(item("calibrationCard"));
            const auto collection = bounds(item("captureCard"));
            require(device.right() < calibration.left() && calibration.right() < collection.left(),
                    "top cards do not form three separate columns");
            require(qAbs(device.height() - calibration.height()) < 1 &&
                    qAbs(device.height() - collection.height()) < 1, "top cards have different heights");
            require(calibration.contains(bounds(previewExposure)) && calibration.contains(bounds(applyExposure)) &&
                    calibration.contains(bounds(undistortion)), "calibration action is outside calibration card");
            require(qAbs(bounds(open).top() - bounds(previewExposure).top()) < 1 &&
                    qAbs(bounds(open).top() - bounds(input).top()) < 1,
                    "first control rows are not aligned");
            require(qAbs(bounds(level).top() - bounds(undistortion).top()) < 1 &&
                    qAbs(bounds(level).top() - bounds(capture).top()) < 1,
                    "second control rows are not aligned");
            // Qt rounds fractional device-independent pixels in layouts.
            require(qAbs(capture->width() - savePose->width()) <= 1,
                    "capture and pose buttons have different widths");
            require(qAbs(bounds(capture).left() - bounds(input).left()) < 1 &&
                    qAbs(bounds(savePose).right() - bounds(item("exposureInput5")).right()) < 1,
                    "capture actions do not align with exposure group edges");
            require(bounds(input).bottom() < bounds(capture).top(),
                    "capture actions should be below exposure inputs");
            require(previewExposure->width() <= 76 && item("resolutionWidthInput")->width() <= 58 &&
                    item("resolutionHeightInput")->width() <= 58, "compact fields expanded unexpectedly");
            for (const auto& names : {
                     QStringList{"cameraIpInput", "resolutionWidthInput", "resolutionHeightInput", "openCameraButton"},
                     QStringList{"previewExposureInput", "applyPreviewExposureButton"},
                     QStringList{"exposureInput", "exposureInput2", "exposureInput3", "exposureInput4", "exposureInput5"}}) {
                for (int i = 1; i < names.size(); ++i)
                    require(!bounds(item(names[i-1].toUtf8().constData())).intersects(
                                bounds(item(names[i].toUtf8().constData()))), "control fields overlap");
            }
            for (const auto* name : {"sessionCard", "recentPhotoCard", "logCard"})
                require(QRectF(0,0,window->width(),window->height()).contains(bounds(item(name))),
                        "sidebar card clipped");
        };
        checkCards();
        const qreal compactHeight = item("controlDeck")->height();
        window->resize(1280, 850);
        QTest::qWait(50);
        checkCards();
        window->resize(1840, 1080);
        QTest::qWait(50);
        checkCards();
        require(qAbs(item("controlDeck")->height() - compactHeight) < 1.0,
                "top cards stretch vertically on a large window");
        require(bounds(item("deviceCard")).bottom() - bounds(item("connectRobotButton")).bottom() <= 16,
                "unnecessary whitespace below device controls");
        click(savePose);
        require(item("controlDeck")->height() > compactHeight &&
                bounds(item("captureCard")).contains(bounds(pointName)),
                "point editor did not expand compact top cards");
        QTest::keyClick(window, Qt::Key_Escape);
        QTest::qWait(30);
        require(qAbs(item("controlDeck")->height() - compactHeight) < 1.0,
                "closing point editor left empty vertical space");
        if (argc > 1) {
            controller->setProperty("cameraOpened", false);
            controller->setProperty("previewActive", false);
            controller->setProperty("robotConnected", false);
            controller->setProperty("savedImageCount", 0);
            controller->setProperty("statusMessage", "就绪 — 打开相机即可预览和拍照");
            controller->setProperty("outputDirectory", QDir::homePath() + "/Pictures/caitu-collect");
            controller->setProperty("handEyePath", QCoreApplication::applicationDirPath() + "/param/hand_eye_result.yaml");
            input->setProperty("text", "5000");
            input->setFocus(false);
            window->contentItem()->forceActiveFocus();
            window->resize(1280, 850);
            QTest::qWait(250);
            require(window->grabWindow().save(QString::fromLocal8Bit(argv[1])), "UI screenshot failed");
            if (argc > 2) {
                controller->setProperty("cameraOpened", true);
                controller->setProperty("previewActive", true);
                controller->setProperty("robotConnected", true);
                controller->setProperty("savedImageCount", 12);
                controller->setProperty("savedPoseCount", 3);
                controller->setProperty("undistortionEnabled", true);
                QTest::qWait(300);
                QMetaObject::invokeMethod(controller.get(), "frameUpdated");
                QTest::qWait(80);
                require(window->grabWindow().save(QString::fromLocal8Bit(argv[2])), "live UI screenshot failed");
                if (argc > 3) {
                    window->resize(1024, 740);
                    QTest::qWait(150);
                    require(window->grabWindow().save(QString::fromLocal8Bit(argv[3])), "compact live UI screenshot failed");
                }
            }
        }
        std::cout << "Collection QML, shared-NOTIFY shutter history, shortcuts, focus, busy state and layout checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

#include "test_collection_ui.moc"
