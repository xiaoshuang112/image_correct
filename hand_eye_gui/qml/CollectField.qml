import QtQuick 2.15
import QtQuick.Controls 2.15

TextField {
    id: control
    property bool mono: false
    implicitHeight: 34
    implicitWidth: 140
    leftPadding: 11
    rightPadding: 11
    font.family: mono ? "DejaVu Sans Mono" : "Noto Sans CJK SC"
    font.pixelSize: 12
    color: enabled ? "#d8dfe8" : "#687383"
    placeholderTextColor: "#6e7989"
    selectionColor: "#344b68"
    selectedTextColor: "#ecf1f8"
    background: Rectangle {
        radius: 7
        color: control.enabled ? "#151a21" : "#181d24"
        border.width: 1
        border.color: control.activeFocus ? "#7796bd" : "#323a46"
        Behavior on border.color { ColorAnimation { duration: 100 } }
    }
}
