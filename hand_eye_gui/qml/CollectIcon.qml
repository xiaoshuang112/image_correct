import QtQuick 2.15
import "."
Canvas {
    id: icon
    property color ink: Theme.ink2
    implicitWidth: 24; implicitHeight: 24
    onInkChanged: requestPaint()
    onWidthChanged: requestPaint()
    onHeightChanged: requestPaint()
    onPaint: {
        var ctx = getContext("2d")
        ctx.reset(); ctx.scale(width / 24, height / 24)
        ctx.strokeStyle = ink; ctx.lineWidth = 1.5; ctx.lineCap = "round"; ctx.lineJoin = "round"
        ctx.beginPath(); ctx.arc(12, 12, 9, 0, Math.PI * 2); ctx.stroke()
        ctx.beginPath()
        for (var i = 0; i < 6; ++i) {
            var a = i * Math.PI / 3
            var x = 12 + Math.cos(a) * 4, y = 12 + Math.sin(a) * 4
            if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
        }
        ctx.closePath(); ctx.stroke()
        for (var j = 0; j < 6; ++j) {
            var b = j * Math.PI / 3, distance = Math.sqrt(65)
            var x0 = 12 + Math.cos(b) * 4, y0 = 12 + Math.sin(b) * 4
            ctx.beginPath(); ctx.moveTo(x0, y0)
            ctx.lineTo(x0 - Math.sin(b) * distance, y0 + Math.cos(b) * distance); ctx.stroke()
        }
    }
}
