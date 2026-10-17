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
    color: enabled ? "#dce8f7" : "#8393a8"
    placeholderTextColor: "#7e91a8"
    selectionColor: "#344b68"
    selectedTextColor: "#ecf1f8"
    background: Rectangle {
        radius: 6
        color: control.enabled ? "#0f151e" : "#181e28"
        border.width: 1
        border.color: control.activeFocus ? "#a6d5ff" : "#354357"
        Behavior on border.color { ColorAnimation { duration: 100 } }
    }
}
