import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

ApplicationWindow {
    id: root
    visible: true
    width: 1280
    height: 850
    minimumWidth: 1000
    minimumHeight: 740
    title: "图像采集 · " + calibrationController.cameraBackend + " · " + appVersion
    color: "#171c24"
    property var controller: calibrationController
    readonly property bool busy: controller.collectionBusy || controller.running
    readonly property bool ready: controller.cameraOpened && controller.previewActive && !busy
    readonly property bool editing: activeFocusItem !== null && activeFocusItem.cursorPosition !== undefined
    function takePhoto() {
        controller.outputDirectory = outputField.text
        controller.exposureList = exposuresField.text
        controller.capturePhoto()
    }
    Shortcut { sequence: "Space"; autoRepeat: false; context: Qt.WindowShortcut; enabled: root.ready && !root.editing; onActivated: root.takePhoto() }
    Shortcut { sequence: "S"; autoRepeat: false; context: Qt.WindowShortcut; enabled: root.ready && !root.editing; onActivated: root.takePhoto() }
    Shortcut { sequence: "Ctrl+S"; autoRepeat: false; context: Qt.WindowShortcut; enabled: root.ready; onActivated: root.takePhoto() }
    Connections {
        target: root.controller
        function onFrameUpdated() { cameraImage.source = "image://camera/current#" + Date.now() }
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 18
        spacing: 12
        RowLayout {
            Layout.fillWidth: true
            Label { text: "图像采集"; color: "#f1f5fb"; font.pixelSize: 28; font.bold: true }
            Label { text: "预览 · 调平 · 多曝光拍照"; color: "#94a3b8"; font.pixelSize: 16 }
            Item { Layout.fillWidth: true }
            Label { text: "本次已保存 " + root.controller.savedImageCount + " 张"; color: "#75c3ff"; font.pixelSize: 18 }
        }

        Pane {
            Layout.fillWidth: true
            background: Rectangle { color: "#222b38"; radius: 8 }
            RowLayout {
                anchors.fill: parent
                Label { text: "相机 IP"; color: "#ced8e6" }
                TextField {
                    text: root.controller.cameraIp; Layout.preferredWidth: 150
                    enabled: !root.controller.cameraOpened && !root.busy
                    onEditingFinished: root.controller.cameraIp = text
                }
                Label { text: "序列号"; color: "#ced8e6" }
                TextField {
                    text: root.controller.cameraSn; placeholderText: "可选"; Layout.preferredWidth: 155
                    enabled: !root.controller.cameraOpened && !root.busy
                    onEditingFinished: root.controller.cameraSn = text
                }
                Label { text: "分辨率"; color: "#ced8e6" }
                TextField {
                    text: root.controller.previewWidth; Layout.preferredWidth: 70
                    validator: IntValidator { bottom: 1; top: 32768 }
                    enabled: !root.controller.cameraOpened && !root.busy
                    onEditingFinished: if (acceptableInput) root.controller.previewWidth = parseInt(text)
                }
                Label { text: "×"; color: "#ced8e6" }
                TextField {
                    text: root.controller.previewHeight; Layout.preferredWidth: 70
                    validator: IntValidator { bottom: 1; top: 32768 }
                    enabled: !root.controller.cameraOpened && !root.busy
                    onEditingFinished: if (acceptableInput) root.controller.previewHeight = parseInt(text)
                }
                Item { Layout.fillWidth: true }
                Button {
                    objectName: "openCameraButton"
                    text: root.controller.cameraOpened ? "关闭相机" : "打开相机并预览"
                    enabled: !root.busy
                    onClicked: {
                        if (root.controller.cameraOpened) root.controller.closeCamera()
                        else {
                            root.controller.openCamera()
                            if (root.controller.cameraOpened) root.controller.startPreview()
                        }
                    }
                }
                Button {
                    text: root.controller.previewActive ? "暂停预览" : "恢复预览"
                    enabled: root.controller.cameraOpened && !root.busy
                    onClicked: root.controller.previewActive ? root.controller.stopPreview() : root.controller.startPreview()
                }
            }
        }

        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 12
            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: "#0c1119"; radius: 8
                clip: true
                Image {
                    id: cameraImage
                    anchors.fill: parent
                    anchors.margins: 6
                    fillMode: Image.PreserveAspectFit
                    cache: false
                    source: "image://camera/current"
                }
                Label {
                    anchors.centerIn: parent
                    visible: !root.controller.cameraOpened
                    text: "打开相机开始预览"; color: "#94a3b8"; font.pixelSize: 22
                }
                Rectangle {
                    anchors.centerIn: parent; width: 42; height: 1; color: "#88ffffff"
                    visible: root.controller.previewActive
                }
                Rectangle {
                    anchors.centerIn: parent; width: 1; height: 42; color: "#88ffffff"
                    visible: root.controller.previewActive
                }
                Label {
                    anchors.left: parent.left; anchors.bottom: parent.bottom; anchors.margins: 12
                    text: root.controller.cameraModel + "  " + root.controller.cameraActualWidth + "×" + root.controller.cameraActualHeight
                    color: "#ced8e6"; visible: root.controller.cameraOpened
                }
                BusyIndicator { anchors.centerIn: parent; running: root.busy }
            }
            ScrollView {
                id: photoSettings
                Layout.preferredWidth: 310
                Layout.fillHeight: true
                clip: true
                contentWidth: availableWidth
                ColumnLayout {
                width: photoSettings.availableWidth
                Label { text: "拍照设置"; font.pixelSize: 20; font.bold: true; color: "#f1f5fb" }
                Label { text: "预览曝光（μs）"; color: "#ced8e6" }
                RowLayout {
                    TextField {
                        id: previewExposure
                        text: root.controller.cameraExposureUs; Layout.fillWidth: true
                        validator: IntValidator { bottom: 1; top: 1000000 }
                        enabled: !root.busy
                    }
                    Button {
                        text: "应用"; enabled: !root.busy && previewExposure.acceptableInput
                        onClicked: root.controller.adjustCameraExposure(parseInt(previewExposure.text))
                    }
                }
                Label { text: "拍照曝光列表（μs，逗号分隔）"; color: "#ced8e6" }
                TextField {
                    id: exposuresField
                    objectName: "exposureInput"
                    Layout.fillWidth: true
                    text: root.controller.exposureList
                    enabled: !root.busy
                    onEditingFinished: root.controller.exposureList = text
                }
                Label { text: "每次采齐全部曝光后，融合为一张 PNG。\n采集时请保持相机和被拍物体静止。"; color: "#94a3b8"; wrapMode: Text.Wrap; Layout.fillWidth: true }
                Label { text: "保存目录"; color: "#ced8e6" }
                TextField {
                    id: outputField
                    Layout.fillWidth: true
                    text: root.controller.outputDirectory
                    enabled: !root.busy
                    onEditingFinished: root.controller.outputDirectory = text
                }
                Button {
                    objectName: "captureButton"
                    Layout.fillWidth: true
                    text: "拍照保存  ·  空格 / S / Ctrl+S"
                    enabled: root.ready
                    highlighted: true
                    onClicked: root.takePhoto()
                }
                Button {
                    text: "打开保存目录"; Layout.fillWidth: true
                    onClicked: Qt.openUrlExternally("file://" + root.controller.outputDirectory)
                }
                Label { text: "最近保存"; color: "#ced8e6" }
                Image {
                    Layout.fillWidth: true; Layout.preferredHeight: 120
                    source: root.controller.lastCapturePath ? "file://" + root.controller.lastCapturePath : ""
                    fillMode: Image.PreserveAspectFit
                }
                Label {
                    text: root.controller.lastCapturePath ? root.controller.lastCapturePath.split("/").pop() : "尚未拍照"
                    color: "#94a3b8"; elide: Text.ElideMiddle; Layout.fillWidth: true
                }
                }
            }
        }

        Pane {
            Layout.fillWidth: true
            background: Rectangle { color: "#222b38"; radius: 8 }
            ColumnLayout {
                anchors.fill: parent
                RowLayout {
                    Layout.fillWidth: true
                    Label { text: "机器人调平"; color: "#f1f5fb"; font.pixelSize: 18; font.bold: true }
                    Label { text: "IP"; color: "#ced8e6" }
                    TextField {
                        text: root.controller.robotIp; Layout.preferredWidth: 140
                        enabled: !root.controller.robotConnected && !root.busy
                        onEditingFinished: root.controller.robotIp = text
                    }
                    Label { text: "端口"; color: "#ced8e6" }
                    TextField {
                        text: root.controller.robotPort; Layout.preferredWidth: 70
                        validator: IntValidator { bottom: 1; top: 65535 }
                        enabled: !root.controller.robotConnected && !root.busy
                        onEditingFinished: if (acceptableInput) root.controller.robotPort = parseInt(text)
                    }
                    Label { text: "密码"; color: "#ced8e6" }
                    TextField {
                        text: root.controller.robotPasswd; echoMode: TextInput.Password; Layout.preferredWidth: 90
                        enabled: !root.controller.robotConnected && !root.busy
                        onEditingFinished: root.controller.robotPasswd = text
                    }
                    Button {
                        text: root.controller.robotConnected ? "断开机器人" : "连接机器人"; enabled: !root.busy
                        onClicked: root.controller.robotConnected ? root.controller.disconnectRobot() : root.controller.connectRobot()
                    }
                    Item { Layout.fillWidth: true }
                }
                RowLayout {
                    Layout.fillWidth: true
                    Label { text: "手眼文件"; color: "#ced8e6" }
                    TextField {
                        id: handEyeField; text: root.controller.handEyePath; Layout.fillWidth: true
                        enabled: !root.busy
                        onEditingFinished: root.controller.handEyePath = text
                    }
                    Label { text: "矩阵平移单位"; color: "#ced8e6" }
                    ComboBox {
                        id: units
                        model: ["m", "mm"]
                        currentIndex: root.controller.handEyeUnits === "mm" ? 1 : 0
                        enabled: !root.busy
                        onActivated: root.controller.handEyeUnits = currentText
                    }
                    Button {
                        objectName: "levelButton"
                        text: "调平 · 相机向下"
                        enabled: root.ready && root.controller.robotConnected
                        onClicked: {
                            root.controller.handEyePath = handEyeField.text
                            root.controller.handEyeUnits = units.currentText
                            root.controller.levelCamera()
                        }
                    }
                    Button {
                        text: "停止操作"; enabled: root.controller.collectionBusy
                        onClicked: root.controller.cancelCollectionOperation()
                    }
                }
                Label {
                    text: "调平会移动机器人，使镜头平面水平；目标姿态保留镜头中心位置。手眼文件使用 T_flange_camera。"
                    color: "#94a3b8"; Layout.fillWidth: true; wrapMode: Text.Wrap
                }
            }
        }
        Label {
            Layout.fillWidth: true
            text: root.controller.statusMessage
            color: "#d4e9ff"; font.pixelSize: 16
            wrapMode: Text.Wrap; maximumLineCount: 2; elide: Text.ElideMiddle
        }
    }
}
