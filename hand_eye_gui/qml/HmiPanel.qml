import QtQuick 2.15
import "."
Rectangle {
    id: panel
    default property alias contents: content.data
    property int padding: Theme.padding
    color: Theme.surface; border.color: Theme.hairline; radius: Theme.radiusCard
    Item { id: content; x: panel.padding; y: panel.padding; width: panel.width - panel.padding * 2; height: panel.height - panel.padding * 2 }
}
