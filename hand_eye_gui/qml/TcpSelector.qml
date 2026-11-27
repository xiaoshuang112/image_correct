import QtQuick 2.15
import QtQuick.Controls 2.15
import "."
ComboBox {
    id: control
    property var controller
    implicitHeight: 32; implicitWidth: 140
    model: controller.tcpNames
    currentIndex: controller.tcpNames.indexOf(controller.selectedTcp)
    onActivated: controller.selectedTcp = currentText
    opacity: enabled ? 1 : 0.42
    font.family: Theme.sans; font.pixelSize: Theme.meta
    contentItem: Text {
        leftPadding: 10; rightPadding: 24
        text: control.currentIndex < 0 ? "选择 TCP" : control.currentText
        font: control.font
        color: control.currentIndex < 0 ? Theme.ink3 : Theme.ink
        verticalAlignment: Text.AlignVCenter
        elide: Text.ElideRight
    }
    indicator: Text {
        x: control.width - width - 7
        anchors.verticalCenter: parent.verticalCenter
        text: "⌄"; font.pixelSize: 14; color: Theme.ink2
    }
    background: Rectangle {
        color: Theme.surfaceAlt
        radius: Theme.radiusControl
        border.width: 1
        border.color: control.activeFocus ? Theme.accent : Theme.hairline
    }
    delegate: ItemDelegate {
        width: control.width; implicitHeight: 32; text: modelData
        font: control.font
        contentItem: Text { text: modelData; color: Theme.ink; font: control.font; elide: Text.ElideRight; verticalAlignment: Text.AlignVCenter }
        background: Rectangle { color: highlighted ? Theme.raisedHover : Theme.surface; border.color: highlighted ? Theme.accentLine : "transparent"; radius: 4 }
        highlighted: control.highlightedIndex === index
    }
    popup: Popup {
        y: control.height + 2; width: Math.max(control.width, 160); implicitHeight: Math.min(contentItem.implicitHeight + 8, 240); padding: 4
        contentItem: ListView { clip: true; implicitHeight: contentHeight; model: control.popup.visible ? control.delegateModel : null; currentIndex: control.highlightedIndex; ScrollIndicator.vertical: ScrollIndicator {} }
        background: Rectangle { color: Theme.surface; border.color: Theme.hairlineStrong; radius: Theme.radiusControl }
    }
}
