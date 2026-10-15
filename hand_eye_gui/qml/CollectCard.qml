import QtQuick 2.15
import QtQuick.Controls 2.15

Pane {
    property color surfaceColor: "#151820"
    property color outlineColor: "#252a34"
    padding: 14
    background: Rectangle {
        radius: 10
        color: surfaceColor
        border.width: 1
        border.color: outlineColor
    }
}
