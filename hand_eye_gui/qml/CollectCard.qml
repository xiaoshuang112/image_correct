import QtQuick 2.15
import QtQuick.Controls 2.15

Pane {
    property color surfaceColor: "#161c25"
    property color outlineColor: "#2b3441"
    padding: 12
    background: Rectangle {
        radius: 9
        color: surfaceColor
        border.width: 1
        border.color: outlineColor
    }
}
