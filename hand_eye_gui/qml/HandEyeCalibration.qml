import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

ApplicationWindow {
    id: root
    visible: true
    width: 1280
    height: 800
    title: "手眼标定 — " + (calibrationController.calibrationType === "eye_in_hand"
           ? "眼在手内" : "眼在手外")
    color: "#1a1e24"
    visibility: "Maximized"

    // 启动后最大化
    Timer {
        interval: 20
        running: true
        repeat: false
        onTriggered: root.showMaximized()
    }

    // =================================================================
    //  颜色常量
    // =================================================================
    readonly property color bgDark:    "#1a1e24"
    readonly property color bgPanel:   "#21262d"
    readonly property color borderColor: "#30363d"
    readonly property color accent:    "#1e88e5"
    readonly property color accentRed: "#da3633"
    readonly property color textPrimary: "#e6edf3"
    readonly property color textDim:   "#8b949e"

    // =================================================================
    //  与 C++ 控制器的连接
    // =================================================================
    // QML 侧定时刷新图像（独立于 C++ 抓帧，避免事件循环阻塞导致不刷新）
    Timer {
        id: refreshTimer
        interval: 50
        running: calibrationController.previewActive || calibrationController.running
        repeat: true
        onTriggered: {
            cameraImage.source = "image://camera/current#" + Date.now()
        }
    }

    Connections {
        target: calibrationController
        function onCalibrationFinished(success, message) {
            console.log("Calibration finished:", success, message)
        }
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 16
        spacing: 12

        // =============================================================
        //  顶部：连接配置 + 相机配置 + 模式选择（双行自适应）
        // =============================================================
        Rectangle {
            Layout.fillWidth: true
            implicitHeight: topColumn.implicitHeight + 20
            radius: 8
            color: bgPanel
            border.color: borderColor

            ColumnLayout {
                id: topColumn
                anchors.fill: parent
                anchors.margins: 10
                spacing: 6

                // ---- 第 1 行：机器人连接 + 标定模式 ----
                RowLayout {
                    Layout.fillWidth: true
                    spacing: 10

                    Label { text: "机器人IP:"; color: textDim; font.pixelSize: 22 }
                    TextField {
                        text: calibrationController.robotIp
                        font.pixelSize: 22; color: textPrimary
                        Layout.preferredWidth: 130
                        background: Rectangle { radius: 4; color: "#0d1117"; border.color: borderColor }
                        onEditingFinished: calibrationController.robotIp = text
                    }

                    Label { text: "端口:"; color: textDim; font.pixelSize: 22 }
                    TextField {
                        text: calibrationController.robotPort
                        font.pixelSize: 22; color: textPrimary
                        Layout.preferredWidth: 65
                        background: Rectangle { radius: 4; color: "#0d1117"; border.color: borderColor }
                        onEditingFinished: {
                            var v = parseInt(text)
                            if (!isNaN(v)) calibrationController.robotPort = v
                        }
                    }

                    Label { text: "密码:"; color: textDim; font.pixelSize: 22 }
                    TextField {
                        text: calibrationController.robotPasswd
                        font.pixelSize: 22; color: textPrimary
                        Layout.preferredWidth: 80
                        background: Rectangle { radius: 4; color: "#0d1117"; border.color: borderColor }
                        onEditingFinished: calibrationController.robotPasswd = text
                    }

                    Button {
                        text: calibrationController.robotConnected ? "断开" : "连接"
                        font.pixelSize: 21; font.bold: true
                        enabled: !calibrationController.running
                        background: Rectangle {
                            radius: 6
                            color: parent.down ? "#1565c0" : parent.hovered ? "#1976d2" : accent
                        }
                        contentItem: Text {
                            text: parent.text; color: "#ffffff"; font: parent.font
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                        }
                        onClicked: {
                            if (calibrationController.robotConnected)
                                calibrationController.disconnectRobot()
                            else
                                calibrationController.connectRobot()
                        }
                    }

                    Item { Layout.fillWidth: true }

                    // 标定模式
                    ButtonGroup { buttons: [btnEyeInHand, btnEyeToHand] }

                    Button {
                        id: btnEyeInHand
                        checkable: true
                        enabled: !calibrationController.running
                        checked: calibrationController.calibrationType === "eye_in_hand"
                        text: "眼在手内"
                        font.pixelSize: 21; font.bold: true
                        background: Rectangle {
                            radius: 6
                            color: btnEyeInHand.checked ? accent
                                 : parent.down ? "#1565c0" : parent.hovered ? "#1976d2" : "#30363d"
                        }
                        contentItem: Text {
                            text: parent.text; color: "#ffffff"; font: parent.font
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                        }
                        onClicked: calibrationController.calibrationType = "eye_in_hand"
                    }

                    Button {
                        id: btnEyeToHand
                        checkable: true
                        enabled: !calibrationController.running
                        checked: calibrationController.calibrationType === "eye_to_hand"
                        text: "眼在手外"
                        font.pixelSize: 21; font.bold: true
                        background: Rectangle {
                            radius: 6
                            color: btnEyeToHand.checked ? accent
                                 : parent.down ? "#1565c0" : parent.hovered ? "#1976d2" : "#30363d"
                        }
                        contentItem: Text {
                            text: parent.text; color: "#ffffff"; font: parent.font
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                        }
                        onClicked: calibrationController.calibrationType = "eye_to_hand"
                    }
                }

                // ---- 第 2 行：相机配置 ----
                RowLayout {
                    Layout.fillWidth: true
                    spacing: 10

                    Label { text: "相机IP:"; color: textDim; font.pixelSize: 22 }
                    TextField {
                        text: calibrationController.cameraIp
                        font.pixelSize: 22; color: textPrimary
                        Layout.preferredWidth: 170
                        background: Rectangle { radius: 4; color: "#0d1117"; border.color: borderColor }
                        onEditingFinished: calibrationController.cameraIp = text
                    }

                    Label { text: ":"; color: textDim; font.pixelSize: 22 }
                    TextField {
                        text: calibrationController.cameraPort
                        font.pixelSize: 22; color: textPrimary
                        Layout.preferredWidth: 65
                        background: Rectangle { radius: 4; color: "#0d1117"; border.color: borderColor }
                        onEditingFinished: {
                            var v = parseInt(text)
                            if (!isNaN(v)) calibrationController.cameraPort = v
                        }
                    }

                    Label { text: "分辨率:"; color: textDim; font.pixelSize: 22 }
                    TextField {
                        text: calibrationController.previewWidth
                        font.pixelSize: 22; color: textPrimary
                        Layout.preferredWidth: 75
                        background: Rectangle { radius: 4; color: "#0d1117"; border.color: borderColor }
                        onEditingFinished: {
                            var v = parseInt(text)
                            if (!isNaN(v)) calibrationController.previewWidth = v
                        }
                    }
                    Label { text: "×"; color: textDim; font.pixelSize: 22 }
                    TextField {
                        text: calibrationController.previewHeight
                        font.pixelSize: 22; color: textPrimary
                        Layout.preferredWidth: 75
                        background: Rectangle { radius: 4; color: "#0d1117"; border.color: borderColor }
                        onEditingFinished: {
                            var v = parseInt(text)
                            if (!isNaN(v)) calibrationController.previewHeight = v
                        }
                    }

                    Item { Layout.fillWidth: true }
                }
            }
        }

        // =============================================================
        //  操作按钮行
        // =============================================================
        Rectangle {
            Layout.fillWidth: true
            height: 56
            radius: 8
            color: bgPanel
            border.color: borderColor

            RowLayout {
                anchors.fill: parent
                anchors.margins: 10
                spacing: 12

                Item { Layout.fillWidth: true }

                // 打开/关闭相机
                Button {
                    text: calibrationController.cameraOpened ? "关闭相机" : "打开相机"
                    font.pixelSize: 27
                    font.bold: true
                    enabled: !calibrationController.running
                    background: Rectangle {
                        radius: 6
                        color: parent.down ? "#1565c0"
                             : parent.hovered ? "#1976d2"
                             : accent
                    }
                    contentItem: Text {
                        text: parent.text
                        color: "#ffffff"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    onClicked: {
                        if (calibrationController.cameraOpened)
                            calibrationController.closeCamera()
                        else
                            calibrationController.openCamera()
                    }
                }

                // 开始/停止预览
                Button {
                    text: calibrationController.previewActive ? "停止预览" : "开始预览"
                    font.pixelSize: 27
                    font.bold: true
                    enabled: calibrationController.cameraOpened && !calibrationController.running
                    background: Rectangle {
                        radius: 6
                        color: parent.down ? "#b0880a"
                             : parent.hovered ? "#c49b0c"
                             : "#d2991d"
                    }
                    contentItem: Text {
                        text: parent.text
                        color: "#ffffff"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    onClicked: {
                        if (calibrationController.previewActive)
                            calibrationController.stopPreview()
                        else
                            calibrationController.startPreview()
                    }
                }

                // 开始标定
                Button {
                    text: "开始标定"
                    font.pixelSize: 27
                    font.bold: true
                    enabled: calibrationController.robotConnected
                             && calibrationController.cameraOpened
                             && !calibrationController.running
                    background: Rectangle {
                        radius: 6
                        color: parent.down ? "#2a7d2f"
                             : parent.hovered ? "#2e8b33"
                             : "#2ea043"
                    }
                    contentItem: Text {
                        text: parent.text
                        color: "#ffffff"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    onClicked: calibrationController.startCalibration()
                }

                // 取消标定
                Button {
                    text: "取消标定"
                    font.pixelSize: 27
                    font.bold: true
                    visible: calibrationController.running
                    background: Rectangle {
                        radius: 6
                        color: parent.down ? "#b62324"
                             : parent.hovered ? "#d12426"
                             : accentRed
                    }
                    contentItem: Text {
                        text: parent.text
                        color: "#ffffff"
                        font: parent.font
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                    onClicked: calibrationController.stopCalibration()
                }

                Item { Layout.fillWidth: true }
            }
        }

        // =============================================================
        //  状态栏 + 进度条
        // =============================================================
        Rectangle {
            Layout.fillWidth: true
            height: 42
            radius: 6
            color: "#161b22"
            border.color: borderColor

            RowLayout {
                anchors.fill: parent
                anchors.margins: 4
                spacing: 8

                BusyIndicator {
                    id: busyInd
                    Layout.preferredWidth: 24
                    Layout.preferredHeight: 24
                    running: calibrationController.running
                    visible: running
                }

                Label {
                    Layout.fillWidth: true
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    wrapMode: Text.WordWrap
                    text: calibrationController.statusMessage
                    color: calibrationController.statusMessage.includes("失败")
                           ? accentRed : textPrimary
                    font.pixelSize: 27
                    font.bold: true
                }
            }
        }

        // 进度条
        Rectangle {
            Layout.fillWidth: true
            height: 36
            radius: 6
            color: "#161b22"
            border.color: borderColor
            visible: calibrationController.running

            ProgressBar {
                anchors.fill: parent
                anchors.margins: 8
                from: 0
                to: 100
                value: calibrationController.progress
                background: Rectangle {
                    radius: 4
                    color: bgPanel
                }
                contentItem: Item {
                    Rectangle {
                        width: parent.parent.visualPosition * parent.parent.width
                        height: parent.height
                        radius: 4
                        color: accent
                    }
                    Text {
                        anchors.centerIn: parent
                        text: calibrationController.progress + "%"
                        color: textPrimary
                        font.pixelSize: 24
                    }
                }
            }
        }

        // =============================================================
        //  主区域：视频画面 + 结果
        // =============================================================
        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 12

            // 视频画面（占 2/3）
            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.preferredWidth: 2
                radius: 8
                color: "#0d1117"
                border.color: borderColor

                Image {
                    id: cameraImage
                    anchors.fill: parent
                    anchors.margins: 4
                    source: "image://camera/current#0"
                    cache: false
                    fillMode: Image.PreserveAspectFit
                    asynchronous: false
                }

                Label {
                    anchors.centerIn: parent
                    visible: cameraImage.status !== Image.Ready
                             || !calibrationController.cameraOpened
                    text: calibrationController.cameraOpened
                          ? "等待视频流..."
                          : "相机未打开"
                    color: accentRed
                    font.pixelSize: 33
                    font.bold: true
                }
            }

            // 结果 + 日志面板（占 1/3）
            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.preferredWidth: 1
                radius: 8
                color: "#0d1117"
                border.color: borderColor

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 12
                    spacing: 6

                    // ---- 标定结果（紧凑） ----
                    Label {
                        text: "标定结果"
                        color: textPrimary
                        font.pixelSize: 30
                        font.bold: true
                    }

                    Rectangle {
                        Layout.fillWidth: true
                        height: 1
                        color: borderColor
                    }

                    // TCP 偏移（仅眼在手内模式）
                    RowLayout {
                        visible: calibrationController.calibrationType === "eye_in_hand"
                        spacing: 8
                        Label {
                            text: "TCP 偏移 (mm):"
                            color: textDim
                            font.pixelSize: 21
                        }
                        TextField {
                            id: tcpOffsetField
                            text: calibrationController.cameraToTcpEnd
                            font.pixelSize: 21
                            color: textPrimary
                            Layout.preferredWidth: 60
                            background: Rectangle {
                                radius: 4; color: "#161b22"; border.color: borderColor
                            }
                            onEditingFinished: {
                                var v = parseInt(text)
                                if (!isNaN(v)) calibrationController.cameraToTcpEnd = v
                            }
                        }
                    }

                    Flickable {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 60
                        clip: true
                        contentWidth: resultText.paintedWidth
                        contentHeight: resultText.paintedHeight
                        TextArea {
                            id: resultText
                            readOnly: true
                            text: calibrationController.resultText || "等待标定完成..."
                            color: textDim
                            font.pixelSize: 18
                            font.family: "monospace"
                            background: null
                            wrapMode: TextArea.NoWrap
                            selectByMouse: true
                        }
                    }

                    // ---- 分隔 ----
                    Rectangle {
                        Layout.fillWidth: true
                        height: 1
                        color: borderColor
                    }

                    // ---- 持久化诊断日志 ----
                    Label {
                        text: "诊断日志"
                        color: textPrimary
                        font.pixelSize: 24
                        font.bold: true
                    }

                    TextField {
                        Layout.fillWidth: true
                        readOnly: true
                        selectByMouse: true
                        text: calibrationController.diagnosticLogPath
                        color: textDim
                        font.pixelSize: 14
                        font.family: "monospace"
                        background: Rectangle {
                            radius: 4
                            color: "#161b22"
                            border.color: borderColor
                        }
                        ToolTip.visible: hovered
                        ToolTip.text: "远程分析时请提供该文件及 .1～.5 归档"
                    }

                    Flickable {
                        id: logFlickable
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        clip: true
                        contentWidth: logTextArea.paintedWidth
                        contentHeight: logTextArea.paintedHeight
                        boundsBehavior: Flickable.StopAtBounds

                        function scrollToBottom() {
                            var maxY = contentHeight - height
                            if (maxY > 0) contentY = maxY
                        }

                        TextArea {
                            id: logTextArea
                            readOnly: true
                            text: calibrationController.logText
                            color: "#c9d1d9"
                            font.pixelSize: 18
                            font.family: "monospace"
                            background: null
                            wrapMode: TextArea.Wrap
                            selectByMouse: true
                            onPaintedHeightChanged: { logFlickable.contentHeight = paintedHeight; logFlickable.scrollToBottom() }
                            onPaintedWidthChanged:  { logFlickable.contentWidth = paintedWidth }
                        }
                    }

                    // 当日志文本变化时，延迟一帧滚动（确保布局完成）
                    Connections {
                        target: calibrationController
                        function onLogTextChanged() {
                            Qt.callLater(function() { logFlickable.scrollToBottom() })
                        }
                    }
                }
            }
        }
    }
}
