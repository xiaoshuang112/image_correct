#include <QGuiApplication>
#include <QClipboard>
#include <QDir>
#include <QQmlApplicationEngine>
#include <QQmlComponent>
#include <QQmlContext>
#include <QQuickImageProvider>
#include <QQuickItem>
#include <QQuickItemGrabResult>
#include <QQuickWindow>
#include <QtTest/QTest>
#include <QtTest/QSignalSpy>
#include <qqml.h>
#include <iostream>
#include <functional>
#include <memory>
#include <limits>
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
    Q_INVOKABLE void copyText(const QString& value) { QGuiApplication::clipboard()->setText(value); }
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
    id: mockController
    property bool cameraOpened: false
    property bool previewActive: false
    property bool previewVisible: true
    property size previewDisplaySize: Qt.size(1280, 960)
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
    property int cameraActualExposureUs: 12000
    property string handEyePath: "/tmp/calibration.yaml"
    property string outputDirectory: "/tmp/collect-ui-check"
    property string exposureList: "5000,8500,14500,25000"
    property string lastCapturePath: ""
    property string lastCaptureTime: ""
    property int savedPoseCount: 0
    property string pointName: ""
    property bool poseFailure: false
    property var tcpNames: []
    property string selectedTcp: ""
    signal tcpToolsChanged()
    onSelectedTcpChanged: tcpToolsChanged()
    property string ninePointProgress: ""
    readonly property var ninePointDefaultYawOffsets: [0,15,-15,-20,0,20,15,-15,0]
    property bool motionFault: false
    property int ninePointCalls: 0
    property real lastNinePointX: 0
    property real lastNinePointY: 0
    property var lastNinePointAngles: []
    property string currentTcp: "探针A"
    property bool tcpListFailure: false
    property string robotIp: "192.168.66.14"
    property int robotSpeedMmS: 10
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
        var saved = !captureFailure && !captureCancelled
        if (saved) lastCaptureTime = Qt.formatTime(new Date(), "hh:mm:ss")
        finishCapture(saved) // one shared completion NOTIFY
    }
    function openCollectionFolder() {}
    function refreshTcpTools() {
        if (collectionBusy || !robotConnected) return
        var previous = selectedTcp
        tcpNames = tcpListFailure ? [] : ["探针A", "探针B", "夹具中心"]
        selectedTcp = tcpNames.indexOf(previous) >= 0 ? previous :
            previous === "" && tcpNames.indexOf(currentTcp) >= 0 ? currentTcp : ""
        statusMessage = tcpListFailure ? "TCP 列表读取失败" : "已加载 TCP 列表"
    }
    function saveTcpPose(name) {
        if (collectionBusy || !robotConnected || !selectedTcp) return
        pointName = name
        if (poseFailure) { statusMessage = "TCP 位姿读取失败，未保存点位文件"; return }
        savedPoseCount++
        statusMessage = "TCP 坐标已保存：tcp_" + selectedTcp + "_" + (name || "20261015_123456_001") + ".json"
    }
    function connectRobot() { robotConnected = true; refreshTcpTools(); tcpPoseTool = selectedTcp; tcpPoseValid = true; tcpPoseStatus = "位姿已更新" }
    function disconnectRobot() { robotConnected = false; tcpNames = []; selectedTcp = ""; tcpPoseValid = false; tcpPoseStatus = "机器人未连接" }
    property var tcpPoseValues: [100.125, -20.0, 300, 0, 179.5, 0]
    property bool tcpPoseValid: false
    property string tcpPoseStatus: "机器人未连接"
    property string tcpPoseTimestamp: "12:34:56.100"
    property string tcpPoseTool: ""
    property bool tcpPoseFresh: true
    property string orientationMode: "axis"
    readonly property var tcpDisplayPoseValues: tcpPoseValues.map(function(v,i) { return i >= 3 && orientationMode === "euler" ? v + 1 : v })
    property real manualLinearSpeedMmS: 10
    property real manualAngularSpeedDegS: 5
    property bool jogActive: false
    property int jogCalls: 0
    property int jogStops: 0
    property int jogHeartbeats: 0
    property int activeLease: 0
    property string previewPositiveX: "none"
    property string previewPositiveY: "none"
    function setManualSpeeds(linear, angular) { manualLinearSpeedMmS = linear; manualAngularSpeedDegS = angular }
    function beginTcpJog(axis, sign, mode) { jogCalls++; activeLease = jogCalls; jogActive = true; return activeLease }
    function renewTcpJog(token) { if (token === activeLease && jogActive) jogHeartbeats++ }
    function stopTcpJog(token) { if (token === activeLease && jogActive) { jogStops++; jogActive = false } }
    property bool conversionFailure: false
    function convertTcpPose(text, from, to) {
        if (conversionFailure) return {valid: false, error: "模拟欧拉奇异点：转换失败", values: []}
        var result = parseTcpTarget(text)
        if (result.valid && from !== to) result.values = result.values.map(function(v,i) { return i < 3 ? v : v + (to === "euler" ? 1 : -1) })
        return result
    }
    function copyCurrentTcpCoordinates(mode) { copyText(tcpDisplayPoseValues.map(function(v) { return Number(v).toFixed(2) }).join(",")) }
    function setPreviewDirections(x,y) {
        previewPositiveX = x
        var allowed = x === "none" ? ["none"] : x === "left" || x === "right" ? ["up", "down"] : ["left", "right"]
        previewPositiveY = allowed.indexOf(y) >= 0 ? y : allowed[0]
    }
    property bool manualMoveActive: false
    property string manualMoveProgress: ""
    property int manualMoveCalls: 0
    property string lastTarget: ""
    property string settingsFailure: ""
    property int settingsCalls: 0
    property bool diagnosticsOpened: false
    function parseTcpTarget(text) {
        var parts = text.replace(/，/g, ",").split(",")
        if (parts.length !== 6) return {valid: false, error: "请输入六个逗号分隔的数值", values: []}
        var values = []
        for (var i = 0; i < 6; ++i) {
            if (parts[i].trim() === "" || !isFinite(Number(parts[i])))
                return {valid: false, error: "目标坐标必须是有限数值", values: []}
            values.push(Number(parts[i]))
        }
        return {valid: true, error: "", values: values}
    }
    function moveTcpToDisplay(text, mode) { manualMoveCalls++; lastTarget = text; manualMoveActive = true; collectionBusy = true; manualMoveProgress = "正在移动" }
    property bool settingsSaving: false
    property bool coordinatesImporting: false
    property int coordinateDialogCalls: 0
    signal coordinatesImported(string coordinates, string tcpName, string error)
    function chooseTcpCoordinateFile() { coordinateDialogCalls++ }
    property int settingsDelay: 0
    property var pendingSettings
    signal settingsFinished(string error)
    property Timer settingsTimer: Timer { interval: mockController.settingsDelay; onTriggered: mockController.finishSettings() }
    function applySettings(values) {
        settingsCalls++
        settingsSaving = true; collectionBusy = true
        pendingSettings = JSON.parse(JSON.stringify(values))
        settingsTimer.start()
    }
    function finishSettings() {
        var values = pendingSettings
        var error = settingsFailure
        if (!error && (!/^\d+$/.test(String(values.robotSpeedMmS)) || Number(values.robotSpeedMmS) < 1 || Number(values.robotSpeedMmS) > 1000))
            error = "移动速度须为 1～1000 mm/s 范围内的整数"
        if (!error) {
            cameraIp = values.cameraIp; robotIp = values.robotIp
            robotSpeedMmS = Number(values.robotSpeedMmS)
            previewWidth = Number(values.previewWidth); previewHeight = Number(values.previewHeight)
            cameraExposureUs = Number(values.cameraExposureUs); exposureList = values.exposureList
        }
        settingsSaving = false; collectionBusy = false
        settingsFinished(error)
    }
    function openDiagnosticFolder() { diagnosticsOpened = true }
    function captureNinePoints(x, y, angles) {
        lastNinePointAngles = angles.slice()
        ninePointCalls++; lastNinePointX = x; lastNinePointY = y
        collectionBusy = true
        ninePointProgress = "第 1/9 点 · X− / Y+ · 正在采集及保存"
    }
    function cancelCollectionOperation() { if (!motionFault) { collectionBusy = false; manualMoveActive = false; jogActive = false } }
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
        auto click = [&](QQuickItem* action) {
            require(action->isVisible() && action->isEnabled(), "attempted to click unavailable control");
            QTest::mouseClick(window, Qt::LeftButton, Qt::NoModifier,
                              action->mapToScene(QPointF(action->width()/2, action->height()/2)).toPoint());
            QTest::qWait(20);
        };
        auto bounds = [](QQuickItem* action) {
            return QRectF(action->mapToScene(QPointF()), QSizeF(action->width(), action->height()));
        };
        auto invoke = [&](const char* method) { require(QMetaObject::invokeMethod(controller.get(), method), "mock invocation failed"); QTest::qWait(10); };
        auto text = [&](const char* name) { return item(name)->property("text").toString(); };
        auto setText = [&](const char* name, const QString& value) {
            item(name)->setProperty("text", value); QTest::qWait(20);
        };
        auto screenshot = [&](const char* path) {
            const auto result = window->contentItem()->grabToImage();
            require(bool(result), "screenshot could not start");
            QSignalSpy completed(result.data(), &QQuickItemGrabResult::ready);
            if (result->image().isNull()) completed.wait(2000);
            return result->image().save(QString::fromLocal8Bit(path));
        };
        auto pageIs = [&](const char* value) { return window->property("page").toString() == value; };
        auto count = [&]() { return controller->property("savedImageCount").toInt(); };
        auto* imageArea = item("captureImageArea");
        auto* animation = window->findChild<QObject*>("shutterFlashAnimation");
        require(animation, "missing shutter feedback");
        QSignalSpy flashes(animation, SIGNAL(started()));
        require(flashes.isValid(), "cannot observe shutter feedback");
        invoke("collectionChanged");
        require(flashes.count() == 0, "unchanged count flashed");
        invoke("capturePhoto");
        require(flashes.count() == 1, "successful capture did not flash once");
        const auto lastCaptureTime = controller->property("lastCaptureTime").toString();
        require(!lastCaptureTime.isEmpty() && text("savedCountSummary").endsWith(lastCaptureTime), "saved summary did not show backend capture time");
        invoke("collectionChanged");
        require(flashes.count() == 1, "shared NOTIFY flashed unchanged count");
        controller->setProperty("captureFailure", true); invoke("capturePhoto");
        require(flashes.count() == 1 && controller->property("lastCaptureTime").toString() == lastCaptureTime, "failed capture changed saved history");
        controller->setProperty("captureFailure", false);
        controller->setProperty("savedImageCount", 0);
        require(imageArea->property("seenCount").toInt() == 0 && !animation->property("running").toBool(), "reset retained shutter history");
        std::unique_ptr<QObject> replacement(mock.create());
        replacement->setProperty("savedImageCount", 9);
        window->setProperty("controller", QVariant::fromValue(replacement.get()));
        require(imageArea->property("seenCount").toInt() == 9 && flashes.count() == 1, "replacement flashed historical count");
        invoke("capturePhoto");
        require(flashes.count() == 1, "old controller flashed after replacement");
        window->setProperty("controller", QVariant::fromValue(controller.get()));
        controller->setProperty("savedImageCount", 0);

        require(!item("captureButton")->isEnabled(), "capture available without preview");
        require(!findVisual(window->contentItem(), "levelButton"), "removed levelling action remains");
        click(item("openCameraButton"));
        click(item("connectRobotButton"));
        require(item("captureButton")->isEnabled(), "camera connect did not start preview");
        require(controller->property("previewVisible").toBool(), "main page did not enable preview processing");
        click(item("captureButton"));
        require(count() == 1, "capture action failed");
        window->contentItem()->forceActiveFocus();
        QTest::keyClick(window, Qt::Key_Space); QTest::qWait(20);
        require(count() == 2, "main capture shortcut failed");
        require(!findVisual(window->contentItem(), "manualPageButton") && !findVisual(window->contentItem(), "operationLog"), "duplicate page or full log remains");
        auto* teach = item("tcpTeachCard");
        require(text("tcpAxis0") == "100.13" && item("copyPoseButton")->isEnabled(), "actual pose unavailable");
        click(item("copyPoseButton"));
        require(QGuiApplication::clipboard()->text() == "100.13,-20.00,300.00,0.00,179.50,0.00", "current pose copy order changed");
        controller->setProperty("tcpPoseFresh", false); QTest::qWait(10);
        require(!item("jog0Plus")->isEnabled() && !item("copyPoseButton")->isEnabled() && text("tcpAxis0") == "—", "stale telemetry permits teaching");
        controller->setProperty("tcpPoseFresh", true);
        controller->setProperty("selectedTcp", "探针B"); QTest::qWait(10);
        require(!item("copyPoseButton")->isEnabled(), "previous tool remains copyable");
        controller->setProperty("tcpPoseTool", "探针B"); QTest::qWait(10);
        auto editAxis = [&](int axis, const QString& value) {
            auto* field = item(("tcpAxis" + std::to_string(axis)).c_str());
            field->forceActiveFocus(); QTest::keyClick(window, Qt::Key_A, Qt::ControlModifier);
            for (const auto ch : value) QTest::keyClick(window, ch.toLatin1()); QTest::qWait(20);
        };
        editAxis(0, "111.25");
        const auto frozenTarget = teach->property("targetText").toString();
        controller->setProperty("tcpPoseValues", QVariantList{101.0, -21.0, 301.0, 2.0, 178.5, 3.0}); QTest::qWait(20);
        require(teach->property("targetDraft").toBool() && teach->property("targetText").toString() == frozenTarget && text("tcpAxis1").toDouble() == -20.0, "telemetry mutated frozen target");
        invoke("tcpToolsChanged");
        require(teach->property("targetDraft").toBool() && teach->property("targetText").toString() == frozenTarget, "same-tool refresh erased target draft");
        require(item("actualPoseWhileEditing")->isVisible() && text("actualPoseWhileEditing").contains("101.00") && !item("jog0Plus")->isEnabled(), "draft does not isolate target and actual");
        click(item("copyPoseButton"));
        require(QGuiApplication::clipboard()->text().startsWith("101.00,-21.00,301.00"), "copy copied draft instead of actual");
        auto* modeSelector = item("orientationModeSelector");
        controller->setProperty("conversionFailure", true);
        modeSelector->forceActiveFocus();
        QTest::keyClick(window, Qt::Key_Down); QTest::qWait(20);
        require(modeSelector->property("currentIndex").toInt() == 0 &&
            controller->property("orientationMode").toString() == "axis" &&
            teach->property("targetText").toString() == frozenTarget && teach->property("feedback").toString().contains("转换失败") &&
            controller->property("manualMoveCalls").toInt() == 0 && controller->property("jogCalls").toInt() == 0,
            "failed conversion left selector, interpretation or target out of sync");
        controller->setProperty("conversionFailure", false);
        QTest::keyClick(window, Qt::Key_Down); QTest::qWait(20);
        require(modeSelector->property("currentIndex").toInt() == 1 && controller->property("orientationMode").toString() == "euler" && text("tcpAxis3").toDouble() == 1.0 && teach->property("feedback").toString().isEmpty() && controller->property("manualMoveCalls").toInt() == 0, "representation switch did not convert frozen target and clear previous error without motion");
        item("tcpAxis0")->forceActiveFocus(); QTest::keyClick(window, Qt::Key_Return); QTest::keyClick(window, Qt::Key_Space);
        require(controller->property("manualMoveCalls").toInt() == 0 && count() == 2, "target keyboard activated motion/capture");
        click(item("useCurrentPoseButton"));
        require(!teach->property("targetDraft").toBool() && text("tcpAxis0") == "101.00", "discard did not resume telemetry");
        auto press = [&](QQuickItem* target) { QTest::mousePress(window, Qt::LeftButton, Qt::NoModifier, target->mapToScene(QPointF(target->width()/2,target->height()/2)).toPoint()); };
        auto release = [&](QQuickItem* target) { QTest::mouseRelease(window, Qt::LeftButton, Qt::NoModifier, target->mapToScene(QPointF(target->width()/2,target->height()/2)).toPoint()); QTest::qWait(20); };
        auto* jog = item("jog0Plus");
        press(jog); QTest::qWait(120); release(jog);
        require(controller->property("jogCalls").toInt() == 0, "short click started jog");
        press(jog); QTest::qWait(380);
        require(controller->property("jogCalls").toInt() == 1 && controller->property("jogHeartbeats").toInt() > 0, "hold did not acquire and renew motion lease");
        release(jog);
        require(!controller->property("jogActive").toBool() && controller->property("jogStops").toInt() == 1, "release did not stop jog");
        press(jog); QTest::qWait(280); QTest::mouseMove(window, QPoint(1,1)); QTest::qWait(30); release(jog);
        require(!controller->property("jogActive").toBool() && controller->property("jogStops").toInt() == 2, "pointer leave did not stop jog");
        press(jog); QTest::qWait(280); window->hide(); QTest::qWait(30);
        require(!controller->property("jogActive").toBool(), "hidden window did not stop jog");
        window->show(); window->requestActivate(); require(QTest::qWaitForWindowActive(window), "window did not reactivate"); release(jog);
        press(jog); QTest::qWait(280);
        QQuickWindow otherWindow; otherWindow.show(); otherWindow.requestActivate();
        require(QTest::qWaitForWindowActive(&otherWindow), "cannot test window blur"); QTest::qWait(30);
        require(!controller->property("jogActive").toBool(), "window blur did not stop jog");
        otherWindow.hide(); window->requestActivate(); require(QTest::qWaitForWindowActive(window), "main window did not reactivate after blur"); release(jog);
        press(jog); QTest::qWait(280);
        require(QMetaObject::invokeMethod(item("jogSurface0Plus"), "canceled"), "cannot cancel pointer gesture"); QTest::qWait(20);
        require(!controller->property("jogActive").toBool(), "canceled pointer gesture did not stop jog"); release(jog);
        press(jog); QTest::qWait(100); window->setProperty("page", "settings"); QTest::qWait(220); release(jog);
        require(controller->property("jogCalls").toInt() == 5, "page change did not cancel pending hold");
        window->setProperty("page", "main"); QTest::qWait(20);
        require(QMetaObject::invokeMethod(controller.get(), "setPreviewDirections", Q_ARG(QVariant, QVariant("right")), Q_ARG(QVariant, QVariant("right"))), "direction update failed"); QTest::qWait(10);
        require(item("previewDirectionY")->property("choices").toList().size() == 2 && controller->property("previewPositiveY").toString() == "up", "directions not perpendicular");
        invoke("frameUpdated");
        require(item("previewDirectionGizmo")->isVisible(), "preview direction gizmo missing");
        controller->setProperty("cameraExposureUs", 33333);
        controller->setProperty("cameraActualExposureUs", 33333); invoke("frameUpdated");
        require(text("frameMetadata").contains("33333"), "frame metadata not updated");
        click(item("settingsButton"));
        require(pageIs("settings"), "settings navigation failed");
        require(!controller->property("previewVisible").toBool() && controller->property("previewActive").toBool(), "settings did not suspend processing while preserving preview intent");
        const auto hiddenFrameLabel = text("frameMetadata");
        controller->setProperty("cameraActualExposureUs", 44444);
        invoke("frameUpdated");
        require(text("frameMetadata") == hiddenFrameLabel, "hidden preview still refreshed the image");
        auto* exposureField = item("exposureInput");
        require(QMetaObject::invokeMethod(item("settingsPage"), "reload"), "settings reload failed");
        require(item("exposureInput") == exposureField, "settings reload recreated exposure controls");
        auto* settingsPage = item("settingsPage");
        require(text("robotSpeedInput") == "10", "speed input did not show current speed");
        setText("robotSpeedInput", "1000");
        require(controller->property("robotSpeedMmS").toInt() == 10 && settingsPage->property("dirty").toBool(), "draft speed applied before save");
        controller->setProperty("collectionBusy", true); QTest::qWait(10);
        require(!item("robotSpeedInput")->isEnabled(), "motion did not lock speed input");
        controller->setProperty("collectionBusy", false);
        require(!item("cameraIpInput")->isEnabled() && !item("robotIpInput")->isEnabled(), "connected device IP remains editable");
        click(item("settingsTab1"));
        setText("previewExposureInput", "15000");
        require(controller->property("cameraExposureUs").toInt() == 33333 && settingsPage->property("dirty").toBool(), "draft changed backend before explicit save");
        item("previewExposureInput")->forceActiveFocus();
        const int beforeSettings = count();
        QTest::keyClick(window, Qt::Key_Return);
        QTest::keyClick(window, Qt::Key_S, Qt::ControlModifier);
        require(count() == beforeSettings && controller->property("settingsCalls").toInt() == 0, "settings keyboard triggered capture or saving");
        controller->setProperty("settingsFailure", "配置保存失败：只读目录");
        controller->setProperty("settingsDelay", 150);
        click(item("saveSettingsButton"));
        require(controller->property("settingsSaving").toBool() && text("saveSettingsButton") == "保存中…" &&
                !item("settingsBackButton")->isEnabled() && !item("robotSpeedInput")->isEnabled(), "saving did not show progress and lock conflicting actions");
        QTest::qWait(170);
        controller->setProperty("settingsDelay", 0);
        require(settingsPage->property("dirty").toBool() && text("previewExposureInput") == "15000" && text("settingsFeedback").contains("只读目录"), "failed save discarded draft or hid backend error");
        controller->setProperty("settingsFailure", "");
        setText("exposureInput", "25000");
        setText("exposureInput2", "5000");
        setText("exposureInput3", "8500");
        setText("exposureInput4", "");
        setText("exposureInput5", "");
        click(item("saveSettingsButton"));
        require(!settingsPage->property("dirty").toBool() && controller->property("exposureList").toString() == "25000,5000,8500", "settings save sorted exposures or retained dirty draft");
        require(controller->property("robotSpeedMmS").toInt() == 1000 && text("robotSpeedInput") == "1000", "saved speed not reflected in input");
        setText("exposureInput2", "25000");
        const int savedCalls = controller->property("settingsCalls").toInt();
        click(item("saveSettingsButton"));
        require(controller->property("settingsCalls").toInt() == savedCalls && text("settingsFeedback").contains("不同的正整数"), "duplicate exposure validation failed");
        setText("exposureInput2", "NaN");
        require(settingsPage->property("exposureError").toString().contains("不同的正整数"), "non-finite exposure accepted");
        click(item("settingsBackButton"));
        require(pageIs("settings"), "dirty settings left without confirmation");
        click(item("keepSettingsButton"));
        require(pageIs("settings") && settingsPage->property("dirty").toBool(), "continue editing lost draft");
        click(item("settingsBackButton"));
        click(item("discardSettingsButton"));
        require(pageIs("main"), "discard did not return to main");
        require(controller->property("previewVisible").toBool(), "returning to main did not resume preview processing");

        click(item("importCoordinatesButton"));
        require(controller->property("coordinateDialogCalls").toInt() == 1, "JSON chooser missing from main");
        require(QMetaObject::invokeMethod(controller.get(), "coordinatesImported", Q_ARG(QString, QString("7,8,9,10,11,12")), Q_ARG(QString, QString("fixture-tool")), Q_ARG(QString, QString{})), "cannot import legacy coordinates"); QTest::qWait(20);
        require(text("tcpAxis0").toDouble() == 7 && text("tcpAxis3").toDouble() == 11 && teach->property("feedback").toString().contains("fixture-tool"), "legacy axis import not converted into selected mode");
        const QString preciseJson = "7.123456789012345,8,9,10.123456789012345,11,12";
        require(QMetaObject::invokeMethod(controller.get(), "coordinatesImported", Q_ARG(QString, preciseJson), Q_ARG(QString, QString("fixture-tool")), Q_ARG(QString, QString{})), "cannot import precise coordinates"); QTest::qWait(20);
        require(teach->property("targetText").toString().split(',').front() == "7.123456789012345", "JSON import rounded away source double precision");
        setText("manualLinearSpeedInput", "12"); setText("manualAngularSpeedInput", "7.5");
        item("manualMoveButton")->forceActiveFocus(); QTest::keyClick(window, Qt::Key_Return); QTest::keyClick(window, Qt::Key_Space);
        require(controller->property("manualMoveCalls").toInt() == 0, "keyboard activated explicit target movement");
        click(item("manualMoveButton"));
        require(controller->property("manualLinearSpeedMmS").toDouble() == 12 && controller->property("manualAngularSpeedDegS").toDouble() == 7.5, "explicit target move did not commit visible manual speeds");
        require(controller->property("manualMoveCalls").toInt() == 1 && !item("tcpAxis0")->isEnabled() && !item("tcpSelector")->isEnabled(), "manual move did not lock controls");
        controller->setProperty("motionFault", true); controller->setProperty("collectionBusy", false); controller->setProperty("manualMoveActive", false); QTest::qWait(20);
        require(text("teachStopButton") == "重试停止" && !item("settingsButton")->isEnabled(), "fault released teaching locks");
        click(item("teachStopButton"));
        require(!window->close(), "window closed before stop confirmed");
        controller->setProperty("motionFault", false); QTest::qWait(20);
        click(item("ninePointModeButton"));
        const QStringList defaults{"0", "+15", "-15", "-20", "0", "+20", "+15", "-15", "0"};
        const QStringList angles{"1.25", "2.5", "-3.75", "-10", "0.1", "10", "7.5", "-15", "0.01"};
        for (int point = 0; point < 9; ++point) {
            const auto name = "ninePointAngle" + std::to_string(point);
            require(text(name.c_str()) == defaults[point], "nine-point default angle differs from requested order");
            for (const QString& invalid : {QString(), QString("21"), QString("nan"), QString("0.001")}) {
                setText(name.c_str(), invalid);
                require(!item("ninePointButton")->isEnabled(), "invalid angle did not prevent collection");
            }
            setText(name.c_str(), angles[point]);
        }
        click(item("singleModeButton")); click(item("ninePointModeButton"));
        for (int point = 0; point < 9; ++point)
            require(text(("ninePointAngle" + std::to_string(point)).c_str()) == angles[point], "mode change discarded edited angles");
        item("ninePointAngle0")->forceActiveFocus();
        QTest::keyClick(window, Qt::Key_Return); QTest::qWait(10);
        require(controller->property("ninePointCalls").toInt() == 0, "editing angles triggered robot collection");
        setText("ninePointOffsetXInput", "40"); setText("ninePointOffsetYInput", "40");
        require(!item("ninePointButton")->isEnabled() && item("ninePointOffsetError")->isVisible(), "unsafe diagonal offset allowed");
        setText("ninePointOffsetXInput", "7.5"); setText("ninePointOffsetYInput", "12.5");
        click(item("ninePointButton"));
        require(controller->property("lastNinePointX").toDouble() == 7.5 && controller->property("lastNinePointY").toDouble() == 12.5, "nine-point offsets not passed");
        const auto submitted = controller->property("lastNinePointAngles").toList();
        require(submitted.size() == 9, "not all nine edited angles were submitted");
        for (int point = 0; point < 9; ++point) {
            require(submitted[point].toDouble() == angles[point].toDouble(), "collector used defaults instead of edited angles");
            require(!item(("ninePointAngle" + std::to_string(point)).c_str())->isEnabled(), "angle editor remained enabled during capture");
        }
        require(!item("openCameraButton")->isEnabled() && !item("connectRobotButton")->isEnabled() && item("stopButton")->isEnabled(), "nine-point operation left connections enabled");
        controller->setProperty("motionFault", true); controller->setProperty("collectionBusy", false); QTest::qWait(10);
        require(text("stopButton") == "重试停止" && !item("settingsButton")->isEnabled(), "main fault did not preserve stop/connection lock");
        controller->setProperty("motionFault", false); QTest::qWait(10);

        for (const QSize size : {QSize(1280,850), QSize(1280,800), QSize(1024,740), QSize(760,600)}) {
            window->resize(size); QTest::qWait(100);
            const QRectF viewport(0,0,size.width(),size.height());
            for (const char* name : {"openCameraButton", "connectRobotButton", "settingsButton", "ninePointPlan", "ninePointButton", "tcpSelector", "savePoseButton", "previewButton", "tcpTeachCard", "manualMoveButton", "tcpAxis0", "tcpAxis5", "previewDirectionX", "previewDirectionY", "operationBrief"}) {
                if (!viewport.contains(bounds(item(name)))) std::cerr << name << " outside viewport at " << size.width() << "x" << size.height() << '\n';
                require(viewport.contains(bounds(item(name))), "action clipped at supported size");
            }
            require(qAbs(bounds(item("mainControls")).bottom() - bounds(item("mainRightColumn")).bottom()) < 1, "main columns not bottom aligned");
            require(bounds(item("captureImageArea")).height() >= 160, "preview too small");
            require(!bounds(item("ninePointButton")).intersects(bounds(item("tcpSelector"))), "capture overlaps TCP selector");
            require(item("tcpTeachCard")->height() <= 212, "teaching card exceeds compact four rows");
            require(bounds(item("tcpAxis0")).width() >= 32 && bounds(item("tcpAxis0")).height() >= 40, "target field unreadable");
            editAxis(0, "123.45");
            require(viewport.contains(bounds(item("actualPoseWhileEditing"))), "actual pose during drafting is clipped");
            require(bounds(item("actualPoseWhileEditing")).bottom() <= bounds(item("previewButton")).top() - 2, "draft actual pose overlaps preview button");
            require(viewport.contains(bounds(item("previewButton"))), "draft pushed preview button outside viewport");
            require(bounds(item("operationBrief")).bottom() <= bounds(item("previewButton")).top() - 2, "brief spills into bottom controls");
            if (argc > 6 && size == QSize(760,600)) require(screenshot(argv[6]), "draft screenshot failed");
            click(item("useCurrentPoseButton"));
            if (argc > 1 && size == QSize(1280,850)) require(screenshot(argv[1]), "main screenshot failed");
            if (argc > 4 && size == QSize(760,600)) require(screenshot(argv[4]), "compact screenshot failed");
            click(item("settingsButton"));
            for (int tab = 0; tab < 3; ++tab) {
                click(item(("settingsTab" + std::to_string(tab)).c_str()));
                require(viewport.contains(bounds(item("saveSettingsButton"))) && viewport.contains(bounds(item("settingsFooter"))), "settings footer clipped");
                if (tab == 2) { click(item("diagnosticFolderButton")); require(controller->property("diagnosticsOpened").toBool(), "log directory unavailable"); }
            }
            if (argc > 2 && size == QSize(1280,850)) { click(item("settingsTab1")); require(screenshot(argv[2]), "settings screenshot failed"); }
            if (argc > 5 && size == QSize(760,600)) { click(item("settingsTab1")); require(screenshot(argv[5]), "compact settings screenshot failed"); }
            click(item("settingsBackButton"));
        }
        window->hide(); QTest::qWait(20);
        require(!controller->property("previewVisible").toBool(), "hidden window still processes preview");
        std::cout << "Collection teaching leases, frozen targets, imports, angle modes, preview annotations, nine point preservation, settings and responsive layouts passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

#include "test_collection_ui.moc"
