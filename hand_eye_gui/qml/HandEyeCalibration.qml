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

    function fmtMetric(v) {
        if (v === undefined || v === null)
            return "--"
        if (typeof v === "number" && (isNaN(v) || !isFinite(v)))
            return "--"
        return Number(v).toFixed(3)
    }

    function fmtCount(v) {
        if (v === undefined || v === null)
            return "--"
        if (typeof v !== "number" || !isFinite(v) || v < 0)
            return "--"
        return v
    }

    // =================================================================
    //  与 C++ 控制器的连接
    // =================================================================
    // 预览最多 10 fps。标定期间不再定时重绘同一张图，而只响应新采集图像。
    Timer {
        id: refreshTimer
        interval: 100
        running: calibrationController.previewActive && !calibrationController.running
        repeat: true
        onTriggered: {
            cameraImage.source = "image://camera/current#" + Date.now()
        }
    }

    Connections {
        target: calibrationController
        function onFrameUpdated() {
            if (calibrationController.running)
                cameraImage.source = "image://camera/current#" + Date.now()
        }
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
            enabled: !calibrationController.running
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

                Rectangle {
                    Layout.preferredWidth: 24
                    Layout.preferredHeight: 24
                    radius: 12
                    color: calibrationController.running ? accent : borderColor
                    visible: calibrationController.running
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
                    anchors.margins: 8
                    spacing: 6

                    TabBar {
                        id: rightTabs
                        Layout.fillWidth: true

                        TabButton {
                            text: "运行状态"
                            font.pixelSize: 18
                            width: implicitWidth
                        }
                        TabButton {
                            text: "标定结果"
                            font.pixelSize: 18
                            width: implicitWidth
                        }
                        TabButton {
                            text: "诊断日志"
                            font.pixelSize: 18
                            width: implicitWidth
                        }
                    }

                    // 运行中自动看“运行状态”，结束后自动切到“标定结果”
                    Connections {
                        target: calibrationController
                        function onRunningChanged() {
                            if (calibrationController.running)
                                rightTabs.currentIndex = 0
                        }
                        function onCalibrationRuntimeChanged() {
                            var outcome = calibrationController.calibrationOutcome
                            if (outcome === "PASS" || outcome === "FAIL" || outcome === "CANCELLED")
                                rightTabs.currentIndex = 1
                        }
                        function onCalibrationFinished(success, message) {
                            rightTabs.currentIndex = 1
                        }
                    }

                    StackLayout {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        currentIndex: rightTabs.currentIndex

                        // -------- Tab 0: 运行状态 --------
                        Flickable {
                            clip: true
                            contentWidth: width
                            contentHeight: runtimeCol.implicitHeight
                            boundsBehavior: Flickable.StopAtBounds

                            ColumnLayout {
                                id: runtimeCol
                                width: parent.width
                                spacing: 6

                                Label {
                                    text: "阶段: " + (calibrationController.calibrationStageText || "空闲")
                                    color: textPrimary
                                    font.pixelSize: 22
                                    font.bold: true
                                }
                                Label {
                                    text: "候选姿态: " +
                                          root.fmtCount(calibrationController.candidateProcessed) + " / " +
                                          root.fmtCount(calibrationController.candidateReachable) +
                                          "  (planned " + root.fmtCount(calibrationController.candidatePlanned) + ")"
                                    color: textDim
                                    font.pixelSize: 18
                                }
                                Label {
                                    text: "成功采集: " + root.fmtCount(calibrationController.capturedFrames) +
                                          "    跳过: " + root.fmtCount(calibrationController.skippedFrames)
                                    color: textDim
                                    font.pixelSize: 18
                                }
                                Label {
                                    text: "PnP有效: " + root.fmtCount(calibrationController.pnpValidFrames) +
                                          "    MAD剔除: " + root.fmtCount(calibrationController.madRejectedFrames) +
                                          "    最终内点: " + root.fmtCount(calibrationController.finalInliers)
                                    color: textDim
                                    font.pixelSize: 18
                                }
                                Label {
                                    text: "平移 RMS/P95/MAX: " +
                                          root.fmtMetric(calibrationController.translationRmsMm) + " / " +
                                          root.fmtMetric(calibrationController.translationP95Mm) + " / " +
                                          root.fmtMetric(calibrationController.translationMaxMm) + " mm"
                                    color: textDim
                                    font.pixelSize: 17
                                }
                                Label {
                                    text: "旋转 RMS/P95/MAX: " +
                                          root.fmtMetric(calibrationController.rotationRmsDeg) + " / " +
                                          root.fmtMetric(calibrationController.rotationP95Deg) + " / " +
                                          root.fmtMetric(calibrationController.rotationMaxDeg) + " deg"
                                    color: textDim
                                    font.pixelSize: 17
                                }
                                Label {
                                    text: "Solver: " + (calibrationController.selectedSolver || "--") +
                                          "  共识: " + (calibrationController.consensusAvailable
                                              ? (calibrationController.consensusPassed ? "PASS" : "FAIL")
                                              : "--")
                                    color: textDim
                                    font.pixelSize: 17
                                }
                                Label {
                                    text: "发布: " + (calibrationController.publicationState || "not_started")
                                    color: textDim
                                    font.pixelSize: 17
                                }
                            }
                        }

                        // -------- Tab 1: 标定结果 --------
                        Flickable {
                            clip: true
                            contentWidth: width
                            contentHeight: resultCol.implicitHeight
                            boundsBehavior: Flickable.StopAtBounds
                            ScrollBar.vertical: ScrollBar { policy: ScrollBar.AsNeeded }

                            ColumnLayout {
                                id: resultCol
                                width: parent.width
                                spacing: 10

                                Label {
                                    Layout.fillWidth: true
                                    horizontalAlignment: Text.AlignHCenter
                                    text: {
                                        var o = calibrationController.calibrationOutcome
                                        if (o === "PASS" || o === "FAIL" || o === "CANCELLED")
                                            return o
                                        return calibrationController.running ? "RUNNING" : "—"
                                    }
                                    color: {
                                        var o = calibrationController.calibrationOutcome
                                        if (o === "PASS") return "#3fb950"
                                        if (o === "FAIL") return accentRed
                                        if (o === "CANCELLED") return "#d2991d"
                                        return textDim
                                    }
                                    font.pixelSize: 36
                                    font.bold: true
                                }

                                // TCP 偏移
                                RowLayout {
                                    visible: calibrationController.calibrationType === "eye_in_hand"
                                    spacing: 8
                                    Label {
                                        text: "TCP 偏移 (mm):"
                                        color: textDim
                                        font.pixelSize: 18
                                    }
                                    TextField {
                                        enabled: !calibrationController.running
                                        text: calibrationController.cameraToTcpEnd
                                        font.pixelSize: 18
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

                                // A. 数据质量
                                Label { text: "数据质量"; color: textPrimary; font.pixelSize: 20; font.bold: true }
                                Label {
                                    text: "候选 planned / reachable: " +
                                          root.fmtCount(calibrationController.candidatePlanned) + " / " +
                                          root.fmtCount(calibrationController.candidateReachable)
                                    color: textDim; font.pixelSize: 17
                                }
                                Label {
                                    text: "PnP有效: " + root.fmtCount(calibrationController.pnpValidFrames)
                                    color: textDim; font.pixelSize: 17
                                }
                                Label {
                                    text: "MAD剔除: " + root.fmtCount(calibrationController.madRejectedFrames)
                                    color: textDim; font.pixelSize: 17
                                }
                                Label {
                                    text: "最终内点: " + root.fmtCount(calibrationController.finalInliers)
                                    color: textDim; font.pixelSize: 17
                                }

                                Rectangle { Layout.fillWidth: true; height: 1; color: borderColor }

                                // B. 平移一致性
                                Label { text: "平移一致性 (mm)"; color: textPrimary; font.pixelSize: 20; font.bold: true }
                                Label {
                                    text: "RMS: " + root.fmtMetric(calibrationController.translationRmsMm)
                                    color: textDim; font.pixelSize: 17
                                }
                                Label {
                                    text: "P95: " + root.fmtMetric(calibrationController.translationP95Mm)
                                    color: textDim; font.pixelSize: 17
                                }
                                Label {
                                    text: "MAX: " + root.fmtMetric(calibrationController.translationMaxMm)
                                    color: textDim; font.pixelSize: 17
                                }

                                Rectangle { Layout.fillWidth: true; height: 1; color: borderColor }

                                // C. 旋转一致性
                                Label { text: "旋转一致性 (deg)"; color: textPrimary; font.pixelSize: 20; font.bold: true }
                                Label {
                                    text: "RMS: " + root.fmtMetric(calibrationController.rotationRmsDeg)
                                    color: textDim; font.pixelSize: 17
                                }
                                Label {
                                    text: "P95: " + root.fmtMetric(calibrationController.rotationP95Deg)
                                    color: textDim; font.pixelSize: 17
                                }
                                Label {
                                    text: "MAX: " + root.fmtMetric(calibrationController.rotationMaxDeg)
                                    color: textDim; font.pixelSize: 17
                                }

                                Rectangle { Layout.fillWidth: true; height: 1; color: borderColor }

                                // D. Solver 共识
                                Label { text: "Solver 共识"; color: textPrimary; font.pixelSize: 20; font.bold: true }
                                Label {
                                    text: "Medoid: " + (calibrationController.selectedSolver || "--")
                                    color: textDim; font.pixelSize: 17
                                }
                                Label {
                                    text: "Max Δt: " + root.fmtMetric(calibrationController.consensusMaxTranslationMm) + " mm"
                                    color: textDim; font.pixelSize: 17
                                }
                                Label {
                                    text: "Max ΔR: " + root.fmtMetric(calibrationController.consensusMaxRotationDeg) + " deg"
                                    color: textDim; font.pixelSize: 17
                                }
                                Label {
                                    text: "Consensus: " + (calibrationController.consensusAvailable
                                          ? (calibrationController.consensusPassed ? "PASS" : "FAIL")
                                          : "--")
                                    color: calibrationController.consensusAvailable
                                           ? (calibrationController.consensusPassed ? "#3fb950" : accentRed)
                                           : textDim
                                    font.pixelSize: 17
                                    font.bold: true
                                }

                                Rectangle { Layout.fillWidth: true; height: 1; color: borderColor }

                                // E. 生产发布
                                Label { text: "生产发布"; color: textPrimary; font.pixelSize: 20; font.bold: true }
                                Label {
                                    text: "production handeye.yaml: " + (
                                        calibrationController.publicationState === "published" ? "已发布" :
                                        calibrationController.publicationState === "publishing" ? "发布中" :
                                        calibrationController.publicationState === "failed" ? "发布失败" :
                                        calibrationController.publicationState === "not_published" ? "未发布" :
                                        "未开始")
                                    color: textDim; font.pixelSize: 17
                                }
                                Label {
                                    text: "Session ID: " + (calibrationController.calibrationSessionId || "--")
                                    color: textDim; font.pixelSize: 15
                                    wrapMode: Text.WrapAnywhere
                                    Layout.fillWidth: true
                                }
                                Label {
                                    text: "Session目录: " + (calibrationController.calibrationSessionDir || "--")
                                    color: textDim; font.pixelSize: 15
                                    wrapMode: Text.WrapAnywhere
                                    Layout.fillWidth: true
                                }

                                Rectangle { Layout.fillWidth: true; height: 1; color: borderColor }

                                // 矩阵：可展开
                                Button {
                                    id: matrixToggle
                                    text: matrixExpanded.checked ? "收起手眼矩阵" : "查看手眼矩阵"
                                    font.pixelSize: 18
                                    checkable: false
                                    onClicked: matrixExpanded.checked = !matrixExpanded.checked
                                    background: Rectangle {
                                        radius: 4
                                        color: parent.down ? "#30363d" : "#21262d"
                                        border.color: borderColor
                                    }
                                    contentItem: Text {
                                        text: parent.text; color: textPrimary; font: parent.font
                                        horizontalAlignment: Text.AlignHCenter
                                        verticalAlignment: Text.AlignVCenter
                                    }
                                }
                                CheckBox {
                                    id: matrixExpanded
                                    visible: false
                                    checked: false
                                }
                                TextArea {
                                    visible: matrixExpanded.checked
                                    Layout.fillWidth: true
                                    readOnly: true
                                    selectByMouse: true
                                    wrapMode: TextArea.NoWrap
                                    text: calibrationController.handEyeMatrixText ||
                                          calibrationController.resultText ||
                                          "尚无矩阵"
                                    color: textDim
                                    font.pixelSize: 15
                                    font.family: "monospace"
                                    background: Rectangle {
                                        color: "#161b22"
                                        border.color: borderColor
                                        radius: 4
                                    }
                                }
                            }
                        }

                        // -------- Tab 2: 诊断日志 --------
                        ColumnLayout {
                            spacing: 6

                            RowLayout {
                                Layout.fillWidth: true
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
                                CheckBox {
                                    text: "自动滚动"
                                    checked: calibrationController.logAutoScroll
                                    font.pixelSize: 16
                                    onCheckedChanged: calibrationController.logAutoScroll = checked
                                    contentItem: Text {
                                        text: parent.text
                                        color: textDim
                                        font: parent.font
                                        leftPadding: parent.indicator.width + parent.spacing
                                        verticalAlignment: Text.AlignVCenter
                                    }
                                }
                            }

                            Flickable {
                                id: logFlickable
                                Layout.fillWidth: true
                                Layout.fillHeight: true
                                clip: true
                                contentWidth: logTextArea.paintedWidth
                                contentHeight: logTextArea.paintedHeight
                                boundsBehavior: Flickable.StopAtBounds

                                property bool userBrowsing: false

                                function scrollToBottom() {
                                    if (!calibrationController.logAutoScroll || userBrowsing)
                                        return
                                    var maxY = contentHeight - height
                                    if (maxY > 0) contentY = maxY
                                }

                                onMovementStarted: {
                                    // 用户上翻查看历史时暂停自动滚底
                                    var nearBottom = (contentHeight - height - contentY) < 40
                                    userBrowsing = !nearBottom
                                    if (userBrowsing)
                                        calibrationController.logAutoScroll = false
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
                                    onPaintedHeightChanged: {
                                        logFlickable.contentHeight = paintedHeight
                                        logFlickable.scrollToBottom()
                                    }
                                    onPaintedWidthChanged: logFlickable.contentWidth = paintedWidth
                                }
                            }

                            Connections {
                                target: calibrationController
                                function onLogTextChanged() {
                                    if (calibrationController.logAutoScroll)
                                        Qt.callLater(function() { logFlickable.scrollToBottom() })
                                }
                                function onLogAutoScrollChanged() {
                                    if (calibrationController.logAutoScroll) {
                                        logFlickable.userBrowsing = false
                                        Qt.callLater(function() { logFlickable.scrollToBottom() })
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
