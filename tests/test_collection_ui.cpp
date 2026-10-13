#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlComponent>
#include <QQmlContext>
#include <QQuickImageProvider>
#include <QQuickItem>
#include <QQuickWindow>
#include <QtTest/QTest>
#include <iostream>
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
    property string cameraBackend: "mvs"
    property string cameraIp: "192.168.0.10"
    property string cameraSn: ""
    property int previewWidth: 2448
    property int previewHeight: 2048
    property string cameraModel: "MVS"
    property int cameraActualWidth: 2448
    property int cameraActualHeight: 2048
    property int cameraExposureUs: 30000
    property string outputDirectory: "/tmp/collect-ui-check"
    property string exposureList: "7500,15000,30000,60000"
    property string lastCapturePath: ""
    property int savedImageCount: 0
    property string robotIp: "192.168.0.6"
    property int robotPort: 2323
    property string robotPasswd: "123"
    property string handEyePath: "/tmp/hand_eye_result.yaml"
    property string handEyeUnits: "m"
    property string statusMessage: "界面检查"
    signal frameUpdated()
    function openCamera() { cameraOpened = true }
    function closeCamera() { previewActive = false; cameraOpened = false }
    function startPreview() { previewActive = true; frameUpdated() }
    function stopPreview() { previewActive = false }
    function capturePhoto() { savedImageCount++ }
    function adjustCameraExposure(value) { cameraExposureUs = value }
    function connectRobot() { robotConnected = true }
    function disconnectRobot() { robotConnected = false }
    function levelCamera() {}
    function cancelCollectionOperation() { collectionBusy = false }
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
        auto item = [&](const char* name) {
            auto* result = window->findChild<QQuickItem*>(name);
            require(result != nullptr, "missing UI action");
            return result;
        };
        auto count = [&]() { return controller->property("savedImageCount").toInt(); };
        auto* open = item("openCameraButton");
        auto* capture = item("captureButton");
        auto* level = item("levelButton");
        auto* input = item("exposureInput");
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
        require(count() == 3, "Ctrl+S failed in input");
        controller->setProperty("collectionBusy", true);
        QTest::qWait(10);
        QTest::keyClick(window, Qt::Key_S, Qt::ControlModifier);
        require(count() == 3 && !capture->isEnabled(), "captured during another operation");
        controller->setProperty("collectionBusy", false);
        controller->setProperty("robotConnected", true);
        QTest::qWait(10);
        require(level->isEnabled(), "level action not enabled with preview and robot");
        window->resize(1000, 740);
        QTest::qWait(50);
        for (auto* action : {open, capture, level}) {
            const QRectF bounds(action->mapToScene(QPointF(0, 0)), QSizeF(action->width(), action->height()));
            if (!QRectF(0, 0, window->width(), window->height()).contains(bounds))
                std::cerr << action->objectName().toStdString() << " bounds " << bounds.x() << ',' << bounds.y()
                          << ' ' << bounds.width() << 'x' << bounds.height() << '\n';
            require(QRectF(0, 0, window->width(), window->height()).contains(bounds), "action clipped at minimum window size");
        }
        if (argc > 1) {
            input->setProperty("text", "7500,15000,30000,60000");
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
