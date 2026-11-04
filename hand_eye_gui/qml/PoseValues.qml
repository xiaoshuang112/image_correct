import QtQuick 2.15
import QtQuick.Layouts 1.15
import "."
ColumnLayout {
    id: pose
    property var values: []
    property bool valid: false
    readonly property string coordinateText: {
        if (!valid || !values || values.length !== 6) return ""
        var parts = []
        for (var i = 0; i < 6; ++i) {
            if (typeof values[i] !== "number" || !isFinite(values[i])) return ""
            parts.push(values[i].toFixed(2))
        }
        return parts.join(",")
    }
    readonly property bool hasCoordinates: coordinateText !== ""
    onHasCoordinatesChanged: if (!hasCoordinates) copied.stop()
    spacing: 8
    RowLayout {
        Layout.fillWidth: true; spacing: 8
        HmiText { objectName: "poseAxes"; text: "X,Y,Z,Rx,Ry,Rz"; Layout.fillWidth: true; font.family: Theme.mono; font.pixelSize: Theme.meta }
        CollectButton {
            objectName: "copyPoseButton"; small: true; Layout.preferredWidth: 80
            text: copied.running ? "已复制" : "复制"; enabled: pose.hasCoordinates
            onClicked: {
                coordinates.selectAll()
                coordinates.copy()
                coordinates.deselect()
                coordinates.cursorPosition = 0
                copied.restart()
            }
        }
    }
    CollectField {
        id: coordinates; objectName: "poseCoordinates"
        Layout.fillWidth: true; Layout.minimumWidth: 0
        text: pose.coordinateText || "—"; readOnly: true; selectByMouse: true
        font.pixelSize: width < 400 ? Theme.body : Theme.title
        onTextChanged: cursorPosition = 0
    }
    Timer { id: copied; interval: 1500 }
}
