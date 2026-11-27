import QtQuick 2.15
import QtQuick.Controls 2.15
import "."
ComboBox {
    id: control
    implicitHeight: 30; implicitWidth: 100
    opacity: enabled ? 1 : 0.42
    font.family: Theme.sans; font.pixelSize: Theme.meta
    contentItem: Text { text: control.currentText; leftPadding: 8; rightPadding: 20; color: Theme.ink; font: control.font; verticalAlignment: Text.AlignVCenter; elide: Text.ElideRight }
    indicator: Text { x: control.width - width - 6; anchors.verticalCenter: parent.verticalCenter; text: "⌄"; font.pixelSize: 14; color: Theme.ink2 }
    background: Rectangle { color: Theme.surfaceAlt; radius: Theme.radiusControl; border.width: 1; border.color: control.activeFocus ? Theme.accent : Theme.hairline }
    delegate: ItemDelegate {
        width: control.width; implicitHeight: 30; highlighted: control.highlightedIndex === index
        contentItem: Text { text: modelData; font: control.font; color: Theme.ink; verticalAlignment: Text.AlignVCenter }
        background: Rectangle { color: highlighted ? Theme.raisedHover : Theme.surface; radius: 4 }
    }
    popup: Popup {
        y: control.height + 2; width: control.width; implicitHeight: Math.min(contentItem.implicitHeight + 8, 220); padding: 4
        contentItem: ListView { clip: true; implicitHeight: contentHeight; model: control.popup.visible ? control.delegateModel : null; currentIndex: control.highlightedIndex; ScrollIndicator.vertical: ScrollIndicator {} }
        background: Rectangle { color: Theme.surface; border.color: Theme.hairlineStrong; radius: Theme.radiusControl }
    }
}
