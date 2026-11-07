import QtQuick 2.15
import QtQuick.Controls 2.15
import "."
CompactSelector {
    id: control
    property string direction: "none"
    property string perpendicularTo: "none"
    property bool restrictPerpendicular: false
    signal chosen(string value)
    readonly property var choices: !restrictPerpendicular ? ["none", "up", "down", "left", "right"] : perpendicularTo === "none" ? ["none"] : perpendicularTo === "up" || perpendicularTo === "down" ? ["left", "right"] : ["up", "down"]
    function label(value) { return ({none: "未设置", up: "↑ 上", down: "↓ 下", left: "← 左", right: "→ 右"})[value] }
    implicitWidth: 88; implicitHeight: 30
    font.family: Theme.sans; font.pixelSize: Theme.meta
    model: choices.map(label)
    currentIndex: Math.max(0, choices.indexOf(direction))
    enabled: !restrictPerpendicular || perpendicularTo !== "none"
    onActivated: chosen(choices[currentIndex])
    ToolTip.visible: hovered
    ToolTip.text: restrictPerpendicular ? "Y+ 始终与 X+ 垂直，仅改变预览标记" : "人工指定画面方向，仅改变预览标记"
}
