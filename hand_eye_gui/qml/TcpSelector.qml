import QtQuick 2.15
import QtQuick.Controls 2.15
import "."
ComboBox {
    id: control
    property var controller
    implicitHeight: Theme.controlHeight; implicitWidth: 180
    model: controller.tcpNames
    currentIndex: controller.tcpNames.indexOf(controller.selectedTcp)
    onActivated: controller.selectedTcp = currentText
    opacity: enabled ? 1 : 0.4
    font.family: Theme.sans; font.pixelSize: Theme.title
    contentItem: Text {
        leftPadding: 12; rightPadding: 32
        text: control.currentIndex < 0 ? "请选择 TCP 工具" : control.currentText
        font: control.font; color: Theme.ink; verticalAlignment: Text.AlignVCenter; elide: Text.ElideRight
    }
    background: Rectangle { color: Theme.bg; radius: Theme.radiusControl; border.color: control.activeFocus ? Theme.accent : Theme.hairline }
    delegate: ItemDelegate {
        width: control.width; text: modelData
        font: control.font
        contentItem: Text { text: modelData; color: Theme.ink; font: control.font; elide: Text.ElideRight; verticalAlignment: Text.AlignVCenter }
        background: Rectangle { color: highlighted ? Theme.raised : Theme.surface; border.color: highlighted ? Theme.accentLine : "transparent"; radius: Theme.radiusControl }
        highlighted: control.highlightedIndex === index
    }
    popup: Popup {
        y: control.height; width: control.width; implicitHeight: Math.min(contentItem.implicitHeight + 8, 240); padding: 4
        contentItem: ListView { clip: true; implicitHeight: contentHeight; model: control.popup.visible ? control.delegateModel : null; currentIndex: control.highlightedIndex; ScrollIndicator.vertical: ScrollIndicator {} }
        background: Rectangle { color: Theme.surface; border.color: Theme.hairline; radius: Theme.radiusControl }
    }
}
