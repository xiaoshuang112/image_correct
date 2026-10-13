import QtQuick 2.15
import QtQuick.Controls 2.15

TextField {
    id: control
    property bool mono: false
    implicitHeight: 36
    implicitWidth: 140
    leftPadding: 11
    rightPadding: 11
    font.family: mono ? "DejaVu Sans Mono" : "Noto Sans CJK SC"
    font.pixelSize: 12
    color: enabled ? "#303b4b" : "#899098"
    placeholderTextColor: "#a0a6ad"
    selectionColor: "#d9e5ff"
    selectedTextColor: "#234fc9"
    background: Rectangle {
        radius: 7
        color: control.enabled ? "#f6f7f5" : "#f1f2f0"
        border.width: 1
        border.color: control.activeFocus ? "#3b70f1" : "#e3e6e2"
        Behavior on border.color { ColorAnimation { duration: 100 } }
    }
}
