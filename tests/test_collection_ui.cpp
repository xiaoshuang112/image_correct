#include <QGuiApplication>
#include <QDir>
#include <QQmlApplicationEngine>
#include <QQmlComponent>
#include <QQmlContext>
#include <QQuickImageProvider>
#include <QQuickItem>
#include <QQuickWindow>
#include <QtTest/QTest>
#include <iostream>
#include <functional>
#include <memory>
#include <stdexcept>

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
    try {
        std::unique_ptr<QObject> controller;
        QQmlApplicationEngine engine;
        QQmlComponent mock(&engine);
        mock.setData(R"QML(
import QtQuick 2.15
QtObject {
    property bool running: false
    property bool collectionBusy: false
    property bool cameraOpened: false
    property bool previewActive: false
    property bool robotConnected: false
    property bool undistortionReady: false
    property bool undistortionEnabled: false
    property string undistortionStatus: "未找到手眼文件，去畸变不可用"
    property string cameraBackend: "mvs"
    property string cameraIp: "192.168.66.253"
    property string cameraSn: ""
    property int previewWidth: 4024
    property int previewHeight: 3036
    property string cameraModel: "MVS"
    property int cameraActualWidth: 4024
    property int cameraActualHeight: 3036
    property int cameraExposureUs: 12000
    property string outputDirectory: "/tmp/collect-ui-check"
    property string exposureList: "5000,8500,14500,25000"
    property string lastCapturePath: ""
    property int savedImageCount: 0
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
    function capturePhoto() { savedImageCount++ }
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
        engine.rootContext()->setContextProperty("calibrationController", controller.get());
        engine.rootContext()->setContextProperty("appVersion", "UI test");
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
        require(!undistortion->isEnabled(), "undistortion enabled with missing calibration");
        controller->setProperty("undistortionReady", true);
        QTest::qWait(10);
        require(undistortion->isEnabled(), "undistortion unavailable with camera and calibration");
        click(undistortion);
        require(controller->property("undistortionEnabled").toBool() &&
                undistortion->property("text").toString() == "去畸变已启用", "undistortion switch not visibly enabled");
        require(item("undistortionStatus")->property("text").toString().contains("预览 / 拍照"),
                "undistortion status does not describe capture mode");
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
        for (auto* action : {open, capture, level, savePose, undistortion, item("previewButton"), item("exposureInput5")}) {
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
        require(qAbs(bounds(capture).bottom() - bounds(item("exposureInput5")).bottom()) < 2.0,
                "capture buttons and exposure fields are not on the same row");
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
            QTest::qWait(50);
            require(window->grabWindow().save(QString::fromLocal8Bit(argv[1])), "UI screenshot failed");
        }
        std::cout << "Collection QML, shortcuts, focus, busy state and layout checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
