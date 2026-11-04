import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import "."
Item {
    id: page
    property var controller
    property bool locked: false
    signal leave()
    property int tab: 0
    property string cameraIp: ""
    property string robotIp: ""
    property string robotSpeed: ""
    property string previewWidth: ""
    property string previewHeight: ""
    property string previewExposure: ""
    property var savedDraft: ({})
    property int revision: 0
    property bool loaded: false
    property string feedback: ""
    readonly property var draft: {
        var dependency = revision
        var exposures = []
        for (var i = 0; i < exposureSlots.count; ++i) {
            var value = exposureSlots.get(i).value.trim()
            if (value !== "") exposures.push(value)
        }
        return {cameraIp: cameraIp, robotIp: robotIp, robotSpeedMmS: robotSpeed, previewWidth: previewWidth,
            previewHeight: previewHeight, cameraExposureUs: previewExposure, exposureList: exposures.join(",")}
    }
    readonly property int dirtyCount: {
        var count = 0
        for (var key in draft) if (String(draft[key]) !== String(savedDraft[key])) ++count
        return loaded ? count : 0
    }
    readonly property bool dirty: dirtyCount > 0
    readonly property string exposureError: {
        var values = draft.exposureList.split(",").filter(function(v) { return v !== "" })
        if (values.length < 2 || values.length > 5) return "融合曝光应包含 2～5 个数值，未写入配置"
        var seen = {}
        for (var i = 0; i < values.length; ++i) {
            var n = Number(values[i])
            if (!/^\d+$/.test(values[i]) || !isFinite(n) || n < 1 || n > 1000000 || seen[n])
                return "融合曝光应为不同的正整数（1～1000000 μs），未写入配置"
            seen[n] = true
        }
        return ""
    }
    function reload() {
        loaded = false
        cameraIp = controller.cameraIp; robotIp = controller.robotIp
        robotSpeed = String(controller.robotSpeedMmS)
        previewWidth = String(controller.previewWidth); previewHeight = String(controller.previewHeight)
        previewExposure = String(controller.cameraExposureUs)
        var values = controller.exposureList.split(/[,，;\s]+/).filter(function(v) { return v !== "" })
        for (var i = 0; i < exposureSlots.count; ++i) exposureSlots.setProperty(i, "value", i < values.length ? values[i] : "")
        ++revision
        savedDraft = JSON.parse(JSON.stringify(draft))
        loaded = true; feedback = "配置已载入，编辑后点击「保存配置」"
    }
    function requestLeave() { if (locked) return; if (dirty) leaveConfirm.open(); else leave() }
    function save() {
        if (locked) return
        if (exposureError) { feedback = exposureError; return }
        feedback = "正在保存配置…"
        controller.applySettings(draft)
    }
    Connections {
        target: page.controller
        function onSettingsFinished(error) {
            if (error) { page.feedback = error; return }
            page.reload(); page.feedback = "配置已保存"
        }
    }
    Component.onCompleted: reload()
    ListModel {
        id: exposureSlots
        ListElement { value: "" }
        ListElement { value: "" }
        ListElement { value: "" }
        ListElement { value: "" }
        ListElement { value: "" }
    }
    Timer {
        id: measure; interval: 0
        onTriggered: pair.height = Math.max(leftContent.implicitHeight + 40, rightContent.implicitHeight + 40,
            Math.min(Theme.pageCardMinHeight, scroll.availableHeight - 32))
    }
    ColumnLayout {
        anchors.fill: parent; spacing: 0
        Rectangle {
            color: Theme.surface; Layout.fillWidth: true; Layout.preferredHeight: 64
            RowLayout {
                anchors.fill: parent; anchors.margins: 12; spacing: 16
                CollectButton { objectName: "settingsBackButton"; text: "← 返回"; quiet: true; enabled: !page.locked; onClicked: page.requestLeave() }
                HmiText { text: "采集 › 设置"; color: Theme.ink; font.pixelSize: Theme.title; Layout.fillWidth: true }
                HmiText { text: page.dirty ? page.dirtyCount + " 项未保存" : "配置"; color: page.dirty ? Theme.warning : Theme.ink3 }
            }
        }
        Item {
            Layout.fillWidth: true; Layout.preferredHeight: 76
            RowLayout {
                width: Math.min(parent.width - 32, Theme.pageContentWidth); height: Theme.controlHeight
                anchors.centerIn: parent; spacing: 8
                Repeater {
                    model: ["设备", "成像", "存储与诊断"]
                    CollectButton { objectName: "settingsTab" + index; text: modelData; selected: page.tab === index; onClicked: page.tab = index }
                }
                Item { Layout.fillWidth: true }
            }
        }
        ScrollView {
            id: scroll
            onAvailableHeightChanged: measure.restart()
            objectName: "settingsScroll"
            Layout.fillWidth: true; Layout.fillHeight: true
            clip: true; contentWidth: availableWidth
            ScrollBar.horizontal.policy: ScrollBar.AlwaysOff
            Item {
                width: scroll.availableWidth
                implicitHeight: Math.max(scroll.availableHeight, pair.height + 32)
                Item {
                    id: pair
                    width: Math.min(parent.width - 32, Theme.pageContentWidth)
                    x: Math.round((parent.width - width) / 2)
                    y: Math.round((parent.height - height) / 2)
                    height: 400
                    onWidthChanged: measure.restart()
                    HmiPanel {
                        objectName: "settingsLeftCard"; width: (parent.width - 16) / 2; height: parent.height; x: 0
                        ColumnLayout {
                            id: leftContent; width: parent.width; height: implicitHeight; onImplicitHeightChanged: measure.restart(); spacing: 16
                            HmiText { text: ["相机", "预览与校正", "数据存储"][page.tab]; color: Theme.ink; font.pixelSize: Theme.appTitle }
                            ColumnLayout {
                                visible: page.tab === 0; Layout.fillWidth: true; spacing: 8
                                HmiText { text: "相机 IP" }
                                CollectField { objectName: "cameraIpInput"; Layout.fillWidth: true; text: page.cameraIp; enabled: !page.locked && !page.controller.cameraOpened; onTextChanged: page.cameraIp = text }
                                HmiText { visible: page.controller.cameraOpened; text: "相机已连接，断开后可编辑 IP 与分辨率"; font.pixelSize: Theme.meta }
                                HmiText { text: "预览分辨率 · px"; Layout.topMargin: 8 }
                                RowLayout {
                                    Layout.fillWidth: true; spacing: 8
                                    CollectField { objectName: "resolutionWidthInput"; Layout.fillWidth: true; Layout.minimumWidth: 0; text: page.previewWidth; enabled: !page.locked && !page.controller.cameraOpened; onTextChanged: page.previewWidth = text }
                                    HmiText { text: "×" }
                                    CollectField { objectName: "resolutionHeightInput"; Layout.fillWidth: true; Layout.minimumWidth: 0; text: page.previewHeight; enabled: !page.locked && !page.controller.cameraOpened; onTextChanged: page.previewHeight = text }
                                }
                                HmiText { Layout.fillWidth: true; text: "后端：" + page.controller.cameraBackend + " · " + page.controller.cameraModel; font.pixelSize: Theme.meta }
                            }
                            ColumnLayout {
                                visible: page.tab === 1; Layout.fillWidth: true; spacing: 8
                                HmiText { text: "预览曝光 · μs" }
                                CollectField { objectName: "previewExposureInput"; Layout.fillWidth: true; text: page.previewExposure; enabled: !page.locked; onTextChanged: page.previewExposure = text }
                                HmiText { Layout.fillWidth: true; text: "预览曝光独立于融合序列；保存后生效。范围 1～10000000 μs。"; font.pixelSize: Theme.meta }
                                CollectButton {
                                    objectName: "undistortionButton"; Layout.fillWidth: true; Layout.topMargin: 16
                                    text: page.controller.undistortionEnabled ? "去畸变已启用" : "启用去畸变"
                                    enabled: !page.locked && page.controller.cameraOpened && (page.controller.undistortionReady || page.controller.undistortionEnabled)
                                    onClicked: page.controller.toggleUndistortion()
                                }
                                HmiText { objectName: "undistortionStatus"; Layout.fillWidth: true; text: page.controller.undistortionStatus; font.pixelSize: Theme.meta }
                                HmiText { Layout.fillWidth: true; text: "校正开关即时生效于预览 / 拍照；曝光配置需要保存。"; font.pixelSize: Theme.meta }
                            }
                            ColumnLayout {
                                visible: page.tab === 2; Layout.fillWidth: true; spacing: 16
                                HmiText { text: "图片 " + page.controller.savedImageCount + "   TCP 点位 " + page.controller.savedPoseCount; color: Theme.ink; font.family: Theme.mono }
                                HmiText { Layout.fillWidth: true; text: page.controller.outputDirectory; font.family: Theme.mono; font.pixelSize: Theme.meta; wrapMode: Text.WrapAnywhere }
                                CollectButton { objectName: "collectionFolderButton"; text: "打开图片目录"; onClicked: page.controller.openCollectionFolder() }
                                HmiText { Layout.fillWidth: true; text: page.controller.lastCapturePath || "尚未保存图片"; font.pixelSize: Theme.meta; wrapMode: Text.WrapAnywhere }
                                HmiText { Layout.fillWidth: true; text: "保存仍保留现有原图、融合图、校正图及位姿数据。预览十字线不会写入图片。"; font.pixelSize: Theme.meta }
                            }
                        }
                    }
                    HmiPanel {
                        objectName: "settingsRightCard"; width: (parent.width - 16) / 2; height: parent.height; x: width + 16
                        ColumnLayout {
                            id: rightContent; width: parent.width; height: implicitHeight; onImplicitHeightChanged: measure.restart(); spacing: 16
                            HmiText { text: ["机器人", "融合曝光序列", "诊断"][page.tab]; color: Theme.ink; font.pixelSize: Theme.appTitle }
                            ColumnLayout {
                                visible: page.tab === 0; Layout.fillWidth: true; spacing: 8
                                HmiText { text: "机器人 IP" }
                                CollectField { objectName: "robotIpInput"; Layout.fillWidth: true; text: page.robotIp; enabled: !page.locked && !page.controller.robotConnected; onTextChanged: page.robotIp = text }
                                HmiText { visible: page.controller.robotConnected; Layout.fillWidth: true; text: "机器人已连接，断开后可编辑 IP"; font.pixelSize: Theme.meta }
                                HmiText { text: "移动速度 · mm/s"; Layout.topMargin: 8 }
                                CollectField {
                                    objectName: "robotSpeedInput"; Layout.fillWidth: true
                                    text: page.robotSpeed; enabled: !page.locked
                                    validator: IntValidator { bottom: 1; top: 1000 }
                                    onTextChanged: page.robotSpeed = text
                                }
                                HmiText { Layout.fillWidth: true; text: "范围 1～1000 mm/s。用于手动 TCP 移动与五点采图，保存后对下一次移动生效。"; font.pixelSize: Theme.meta }
                                HmiText { Layout.fillWidth: true; text: "TCP 工具列表与实时位姿在主页面查看。移动前请确认所选工具和目标坐标。"; font.pixelSize: Theme.meta }
                            }
                            ColumnLayout {
                                visible: page.tab === 1; Layout.fillWidth: true; spacing: 8
                                HmiText { Layout.fillWidth: true; text: "按输入顺序采集 2～5 档；留空的档位不参与融合。单位 μs。"; font.pixelSize: Theme.meta }
                                Repeater {
                                    model: exposureSlots
                                    RowLayout {
                                        Layout.fillWidth: true; spacing: 8
                                        HmiText { text: String(index + 1); font.family: Theme.mono; Layout.preferredWidth: 20 }
                                        CollectField {
                                            objectName: index === 0 ? "exposureInput" : "exposureInput" + (index + 1)
                                            Layout.fillWidth: true; Layout.minimumWidth: 0; text: value; placeholderText: "可选"; enabled: !page.locked; invalid: page.exposureError !== ""
                                            onTextChanged: { if (page.loaded) { exposureSlots.setProperty(index, "value", text); ++page.revision } }
                                        }
                                    }
                                }
                                HmiText { objectName: "exposureError"; Layout.fillWidth: true; text: page.exposureError; color: Theme.danger; font.pixelSize: Theme.meta }
                            }
                            ColumnLayout {
                                visible: page.tab === 2; Layout.fillWidth: true; spacing: 16
                                HmiText { text: "版本 " + appVersion + " · " + appGitCommit; font.family: Theme.mono; Layout.fillWidth: true }
                                HmiText { Layout.fillWidth: true; text: "校正参数\n" + page.controller.handEyePath; font.family: Theme.mono; font.pixelSize: Theme.meta; wrapMode: Text.WrapAnywhere }
                                CollectButton { objectName: "diagnosticFolderButton"; text: "打开诊断目录"; onClicked: page.controller.openDiagnosticFolder() }
                                HmiText { Layout.fillWidth: true; text: "运行日志保留在诊断目录；主页面显示最近 1000 行，可选择复制。"; font.pixelSize: Theme.meta }
                            }
                        }
                    }
                }
            }
        }
        Rectangle {
            objectName: "settingsFooter"; Layout.fillWidth: true; Layout.preferredHeight: 72; color: Theme.surface
            RowLayout {
                anchors.fill: parent; anchors.margins: 16; spacing: 12
                HmiText { objectName: "settingsFeedback"; Layout.fillWidth: true; text: page.controller.settingsSaving ? "正在保存配置，请稍候…" : page.locked ? "操作进行中，设置已锁定" : page.feedback + (page.dirty ? " · " + page.dirtyCount + " 项未保存" : ""); color: page.dirty ? Theme.warning : Theme.ink2 }
                CollectButton { objectName: "resetSettingsButton"; text: "还原编辑"; quiet: true; enabled: page.dirty && !page.locked; onClicked: page.reload() }
                CollectButton { objectName: "saveSettingsButton"; text: page.controller.settingsSaving ? "保存中…" : "保存配置"; primary: true; Layout.preferredWidth: 144; enabled: !page.locked && page.dirty; onClicked: page.save() }
            }
        }
    }
    Dialog {
        id: leaveConfirm; objectName: "dirtySettingsDialog"; anchors.centerIn: parent; width: Math.min(480, page.width - 32); modal: true
        header: HmiText { objectName: "dirtyDialogTitle"; text: leaveConfirm.title; color: Theme.ink; font.pixelSize: Theme.appTitle; padding: 20 }
        title: "配置尚未保存"; closePolicy: Popup.CloseOnEscape
        background: Rectangle { color: Theme.surface; radius: Theme.radiusCard; border.color: Theme.hairline }
        contentItem: ColumnLayout {
            spacing: 16
            HmiText { text: "离开将放弃本次编辑。"; Layout.fillWidth: true }
            RowLayout {
                Item { Layout.fillWidth: true }
                CollectButton { objectName: "keepSettingsButton"; text: "继续编辑"; onClicked: leaveConfirm.close() }
                CollectButton { objectName: "discardSettingsButton"; text: "放弃并返回"; danger: true; onClicked: { leaveConfirm.close(); page.reload(); page.leave() } }
            }
        }
    }
}
