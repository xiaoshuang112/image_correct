#include <QGuiApplication>
#include <QClipboard>
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
    property bool previewVisible: true
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
    property string logText: ""
    property string handEyePath: "/tmp/calibration.yaml"
    property string outputDirectory: "/tmp/collect-ui-check"
    property string exposureList: "5000,8500,14500,25000"
    property string lastCapturePath: ""
    property int savedPoseCount: 0
    property string pointName: ""
    property bool poseFailure: false
    property var tcpNames: []
    property string selectedTcp: ""
    property string fivePointProgress: ""
    property bool motionFault: false
    property int fivePointCalls: 0
    property real lastFivePointX: 0
    property real lastFivePointY: 0
    property string currentTcp: "探针A"
    property bool tcpListFailure: false
    property string robotIp: "192.168.66.14"
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
    function moveTcpTo(text) { manualMoveCalls++; lastTarget = text; manualMoveActive = true; collectionBusy = true; manualMoveProgress = "正在移动" }
    function applySettings(values) {
        settingsCalls++
        if (settingsFailure) return settingsFailure
        cameraIp = values.cameraIp; robotIp = values.robotIp
        previewWidth = Number(values.previewWidth); previewHeight = Number(values.previewHeight)
        cameraExposureUs = Number(values.cameraExposureUs); exposureList = values.exposureList
        return ""
    }
    function openDiagnosticFolder() { diagnosticsOpened = true }
    function captureFivePoints(x, y) {
        fivePointCalls++; lastFivePointX = x; lastFivePointY = y
        collectionBusy = true
        fivePointProgress = "第 1/5 点 · 基准位 · 正在采集及保存"
    }
    function cancelCollectionOperation() { if (!motionFault) { collectionBusy = false; manualMoveActive = false } }
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
        invoke("collectionChanged");
        require(flashes.count() == 1, "shared NOTIFY flashed unchanged count");
        controller->setProperty("captureFailure", true); invoke("capturePhoto");
        require(flashes.count() == 1, "failed capture flashed");
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
        const QString liveCoordinates = "100.13,-20.00,300.00,0.00,179.50,0.00";
        auto* mainPose = item("livePoseValues");
        auto* mainCoordinates = findVisual(mainPose, "poseCoordinates");
        auto* mainCopy = findVisual(mainPose, "copyPoseButton");
        require(mainCoordinates && mainCopy && mainCoordinates->property("readOnly").toBool(), "missing read-only pose and copy action");
        require(mainCoordinates->property("text").toString() == liveCoordinates && text("poseAxes") == "X,Y,Z,Rx,Ry,Rz", "live pose does not use ordered comma-separated values");
        click(mainCopy);
        require(QGuiApplication::clipboard()->text() == liveCoordinates && mainCopy->property("text").toString() == "已复制", "main pose copy did not write displayed coordinates to clipboard");
        require(count() == 2 && controller->property("manualMoveCalls").toInt() == 0, "copy triggered capture or motion");
        controller->setProperty("selectedTcp", "探针B"); QTest::qWait(10);
        require(mainCoordinates->property("text").toString() == "—" && !mainCopy->isEnabled() && text("livePoseStatus").contains("工具已切换"), "previous tool pose remained visible or copyable after selection");
        controller->setProperty("tcpPoseTool", "探针B"); QTest::qWait(10);
        require(mainCoordinates->property("text").toString() == liveCoordinates && mainCopy->isEnabled(), "new selected tool pose not shown or copyable");
        controller->setProperty("tcpPoseValid", false); QTest::qWait(10);
        require(mainCoordinates->property("text").toString() == "—" && !mainCopy->isEnabled(), "stale pose remained visible or copyable");
        controller->setProperty("tcpPoseValid", true);
        controller->setProperty("tcpPoseValues", QVariantList{100.125, -20.0, 300.0, 0.0, std::numeric_limits<double>::infinity(), 0.0});
        QTest::qWait(10);
        require(!mainCopy->isEnabled() && mainCoordinates->property("text").toString() == "—", "non-finite pose remained copyable");
        controller->setProperty("tcpPoseValues", QVariantList{100.125, -20.0, 300.0, 0.0, 179.5, 0.0});
        QTest::qWait(10);
        invoke("frameUpdated");
        const auto frameLabel = text("frameMetadata");
        controller->setProperty("cameraExposureUs", 33333);
        require(text("frameMetadata") == frameLabel, "old image relabelled by settings change");
        controller->setProperty("cameraActualExposureUs", 33333);
        invoke("frameUpdated");
        require(text("frameMetadata").contains("33333"), "frame update did not snapshot metadata");
        click(item("savePoseButton"));
        require(window->property("poseEditorOpen").toBool(), "TCP naming dialog did not open");
        setText("pointNameInput", "失败点位");
        controller->setProperty("poseFailure", true);
        click(item("confirmPoseButton"));
        require(window->property("poseEditorOpen").toBool() && text("pointNameInput") == "失败点位", "failed TCP save discarded naming input");
        controller->setProperty("poseFailure", false);
        click(item("confirmPoseButton"));
        require(!window->property("poseEditorOpen").toBool() && controller->property("savedPoseCount").toInt() == 1, "successful TCP save did not close dialog");

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
        click(item("saveSettingsButton"));
        require(settingsPage->property("dirty").toBool() && text("previewExposureInput") == "15000" && text("settingsFeedback").contains("只读目录"), "failed save discarded draft or hid backend error");
        controller->setProperty("settingsFailure", "");
        setText("exposureInput", "25000");
        setText("exposureInput2", "5000");
        setText("exposureInput3", "8500");
        setText("exposureInput4", "");
        setText("exposureInput5", "");
        click(item("saveSettingsButton"));
        require(!settingsPage->property("dirty").toBool() && controller->property("exposureList").toString() == "25000,5000,8500", "settings save sorted exposures or retained dirty draft");
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

        click(item("manualPageButton"));
        require(pageIs("manual"), "manual page entry failed");
        require(!controller->property("previewVisible").toBool(), "manual page still processes hidden preview");
        auto* manualLive = item("manualLiveCard");
        auto* manualCoordinates = findVisual(manualLive, "poseCoordinates");
        auto* manualCopy = findVisual(manualLive, "copyPoseButton");
        require(manualCoordinates && manualCopy && manualCoordinates->property("text").toString() == liveCoordinates, "manual page current pose format differs from main page");
        QGuiApplication::clipboard()->setText("sentinel");
        click(manualCopy);
        require(QGuiApplication::clipboard()->text() == liveCoordinates, "manual page did not copy the current pose");
        controller->setProperty("robotConnected", false); QTest::qWait(10);
        require(!manualCopy->isEnabled() && manualCoordinates->property("text").toString() == "—", "disconnected pose remained copyable");
        controller->setProperty("robotConnected", true); QTest::qWait(10);
        auto* targetCopy = findVisual(item("parsedTargetValues"), "copyPoseButton");
        require(targetCopy && !targetCopy->isEnabled(), "empty target remained copyable");
        setText("tcpTargetInput", "100，20,300,0,180,0");
        require(item("manualMoveButton")->isEnabled(), "valid Chinese comma target rejected");
        click(targetCopy);
        require(QGuiApplication::clipboard()->text() == "100.00,20.00,300.00,0.00,180.00,0.00", "target copy did not normalize six ordered values");
        setText("tcpTargetInput", "100,20,300");
        require(!targetCopy->isEnabled(), "invalid target remained copyable");
        setText("tcpTargetInput", "100，20,300,0,180,0");
        item("tcpTargetInput")->forceActiveFocus();
        QTest::keyClick(window, Qt::Key_Return);
        QTest::keyClick(window, Qt::Key_S, Qt::ControlModifier);
        QTest::keyClick(window, Qt::Key_Space);
        QTest::qWait(20);
        require(controller->property("manualMoveCalls").toInt() == 0 && count() == beforeSettings, "manual input keyboard triggered motion or capture");
        setText("tcpTargetInput", "100,20,300,0,180,0");
        item("manualMoveButton")->forceActiveFocus();
        QTest::keyClick(window, Qt::Key_Return); QTest::keyClick(window, Qt::Key_Space);
        require(controller->property("manualMoveCalls").toInt() == 0, "keyboard activated manual motion button");
        click(item("manualMoveButton"));
        require(controller->property("manualMoveCalls").toInt() == 1 && !item("tcpTargetInput")->isEnabled() && !item("manualBackButton")->isEnabled() && !item("manualTcpSelector")->isEnabled(), "manual move did not lock conflicting controls");
        controller->setProperty("motionFault", true);
        controller->setProperty("collectionBusy", false);
        controller->setProperty("manualMoveActive", false);
        QTest::qWait(10);
        require(text("manualMoveButton") == "重试停止" && !item("manualBackButton")->isEnabled() && !item("tcpTargetInput")->isEnabled(), "fault alone released motion locks");
        click(item("manualMoveButton"));
        require(!item("manualBackButton")->isEnabled(), "failed stop released UI");
        require(!window->close(), "window closed with unconfirmed stop");
        controller->setProperty("motionFault", false); QTest::qWait(10);
        require(item("manualBackButton")->isEnabled(), "safe stop did not unlock page");
        click(item("manualBackButton"));
        click(item("fivePointModeButton"));
        setText("fivePointOffsetXInput", "7.5"); setText("fivePointOffsetYInput", "12.5");
        click(item("fivePointButton"));
        require(controller->property("lastFivePointX").toDouble() == 7.5 && controller->property("lastFivePointY").toDouble() == 12.5, "five-point offsets not passed");
        require(!item("openCameraButton")->isEnabled() && !item("connectRobotButton")->isEnabled() && item("stopButton")->isEnabled(), "five-point operation left connections enabled");
        controller->setProperty("motionFault", true); controller->setProperty("collectionBusy", false); QTest::qWait(10);
        require(text("stopButton") == "重试停止" && !item("settingsButton")->isEnabled(), "main fault did not preserve stop/connection lock");
        controller->setProperty("motionFault", false); QTest::qWait(10);

        for (const QSize size : {QSize(1280,850), QSize(1024,740), QSize(760,600)}) {
            window->resize(size); QTest::qWait(80);
            const QRectF viewport(0,0,size.width(),size.height());
            for (const char* name : {"openCameraButton", "connectRobotButton", "settingsButton", "fivePointButton", "tcpSelector", "savePoseButton", "manualPageButton", "previewButton", "logCard", "livePoseCard", "copyPoseButton", "poseCoordinates"}) {
                if (!viewport.contains(bounds(item(name)))) std::cerr << name << " outside viewport at " << size.width() << "x" << size.height() << '\n';
                require(viewport.contains(bounds(item(name))), "main action clipped at supported size");
            }
            require(qAbs(bounds(item("mainControls")).bottom() - bounds(item("mainRightColumn")).bottom()) < 1, "main columns are not bottom aligned");
            require(!bounds(item("fivePointButton")).intersects(bounds(item("tcpSelector"))), "primary action overlaps TCP selector");
            click(item("settingsButton"));
            for (int tab = 0; tab < 3; ++tab) {
                click(item(("settingsTab" + std::to_string(tab)).c_str()));
                const auto left = bounds(item("settingsLeftCard")), right = bounds(item("settingsRightCard"));
                require(qAbs(left.height() - right.height()) < 1 && qAbs(left.bottom() - right.bottom()) < 1, "paired settings cards differ in height");
                require(viewport.contains(bounds(item("saveSettingsButton"))) && viewport.contains(bounds(item("settingsFooter"))), "fixed settings footer clipped");
                if (size.height() == 600 && tab == 1) {
                    auto* flick = qvariant_cast<QQuickItem*>(item("settingsScroll")->property("contentItem"));
                    require(flick, "settings scroll has no flickable content");
                    const qreal maximumY = flick->property("contentHeight").toReal() - flick->height();
                    require(maximumY > 0, "small settings content does not scroll");
                    flick->setProperty("contentY", maximumY); QTest::qWait(20);
                    require(bounds(item("exposureInput5")).bottom() <= bounds(item("settingsFooter")).top() + 1, "last exposure remains behind fixed footer after scrolling");
                    flick->setProperty("contentY", 0);
                }
            }
            click(item("settingsBackButton"));
            click(item("manualPageButton"));
            require(viewport.contains(bounds(item("manualMoveButton"))) && qAbs(item("manualTargetCard")->height() - item("manualLiveCard")->height()) < 1, "manual footer clipped or card heights differ");
            click(item("manualBackButton"));
        }
        QStringList logLines;
        for (int line = 0; line < 1005; ++line) logLines << QString("[12:00:00] real diagnostic line %1").arg(line);
        controller->setProperty("logText", logLines.join('\n'));
        QTest::qWait(20);
        require(!text("operationLog").contains("line 4\n") && text("operationLog").contains("line 1004") &&
                text("operationLog").split('\n').size() == 1000, "backend log was not trimmed to latest 1000 lines");
        controller->setProperty("logText", "[12:00:00] 相机预览已启动\n[12:00:01] 当前 TCP：探针B\n[12:00:02] 等待操作");
        click(item("settingsButton"));
        const auto hiddenLog = text("operationLog");
        controller->setProperty("logText", "[12:00:03] 子页面期间的诊断");
        require(text("operationLog") == hiddenLog, "hidden log still rebuilt its text layout");
        click(item("settingsBackButton"));
        require(text("operationLog") == controller->property("logText").toString(), "returning to main lost deferred diagnostic lines");
        window->hide(); QTest::qWait(20);
        require(!controller->property("previewVisible").toBool(), "hidden window still processes preview");
        window->show(); QTest::qWait(20);
        require(controller->property("previewVisible").toBool(), "shown window did not restore preview visibility");
        if (argc > 1) {
            controller->setProperty("statusMessage", "就绪 · 预览中，等待采集");
            controller->setProperty("manualMoveProgress", "已安全停止 · 可编辑新的目标");
            window->resize(1280,850); QTest::qWait(100);
            require(window->grabWindow().save(QString::fromLocal8Bit(argv[1])), "main screenshot failed");
            if (argc > 2) {
                click(item("settingsButton")); click(item("settingsTab1")); QTest::qWait(100);
                require(window->grabWindow().save(QString::fromLocal8Bit(argv[2])), "settings screenshot failed");
                click(item("settingsBackButton"));
            }
            if (argc > 3) {
                click(item("manualPageButton")); QTest::qWait(100);
                require(window->grabWindow().save(QString::fromLocal8Bit(argv[3])), "manual screenshot failed");
                click(item("manualBackButton"));
            }
            if (argc > 4) {
                window->resize(760,600); QTest::qWait(100);
                require(window->grabWindow().save(QString::fromLocal8Bit(argv[4])), "compact screenshot failed");
                if (argc > 5) {
                    click(item("settingsButton")); click(item("settingsTab1")); QTest::qWait(100);
                    require(window->grabWindow().save(QString::fromLocal8Bit(argv[5])), "compact settings screenshot failed");
                    click(item("settingsBackButton"));
                }
                if (argc > 6) {
                    click(item("manualPageButton")); QTest::qWait(100);
                    require(window->grabWindow().save(QString::fromLocal8Bit(argv[6])), "compact manual screenshot failed");
                    click(item("manualBackButton"));
                }
            }
        }
        if (argc > 7) {
            window->resize(1280,850); QTest::qWait(50);
            click(item("savePoseButton")); setText("pointNameInput", "工位_A"); QTest::qWait(50);
            require(window->grabWindow().save(QString::fromLocal8Bit(argv[7])), "TCP naming dialog screenshot failed");
            QTest::keyClick(window, Qt::Key_Escape); QTest::qWait(250);
            require(!window->property("poseEditorOpen").toBool(), "Escape did not close TCP naming dialog");
        }
        if (argc > 8) {
            click(item("tcpSelector")); QTest::qWait(50);
            require(window->grabWindow().save(QString::fromLocal8Bit(argv[8])), "TCP dropdown screenshot failed");
        }
        std::cout << "Collection HMI navigation, draft persistence, pose validity, keyboard safety, motion faults, shutter and responsive layouts passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

#include "test_collection_ui.moc"
