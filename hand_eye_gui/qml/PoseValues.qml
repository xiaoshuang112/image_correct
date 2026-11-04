import QtQuick 2.15
import QtQuick.Layouts 1.15
import "."
GridLayout {
    id: grid
    property var values: []
    property bool valid: false
    property int valueSize: Theme.appTitle
    columns: width < 560 ? 3 : 6
    columnSpacing: 12; rowSpacing: 8
    Repeater {
        model: ["X · mm", "Y · mm", "Z · mm", "Rx · °", "Ry · °", "Rz · °"]
        ColumnLayout {
            Layout.fillWidth: true; Layout.minimumWidth: 0; spacing: 4
            HmiText { text: modelData; font.family: Theme.mono; font.pixelSize: Theme.meta }
            HmiText {
                objectName: "poseValue" + index
                Layout.fillWidth: true
                text: grid.valid && grid.values.length === 6 && isFinite(Number(grid.values[index])) ? Number(grid.values[index]).toFixed(2) : "—"
                color: Theme.ink; font.family: Theme.mono; font.pixelSize: grid.valueSize
                elide: Text.ElideRight; wrapMode: Text.NoWrap
            }
        }
    }
}
