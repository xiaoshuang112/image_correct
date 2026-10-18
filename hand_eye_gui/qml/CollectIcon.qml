import QtQuick 2.15
import "."

Canvas {
    id: icon
    property string name: "camera"
    property color ink: Theme.ink2
    implicitWidth: 20
    implicitHeight: 20
    onNameChanged: requestPaint()
    onInkChanged: requestPaint()
    onWidthChanged: requestPaint()
    onHeightChanged: requestPaint()
    onPaint: {
        var ctx = getContext("2d")
        ctx.reset()
        ctx.scale(width / 24, height / 24)
        ctx.strokeStyle = ink
        ctx.fillStyle = ink
        ctx.lineWidth = 1.5
        ctx.lineCap = "round"
        ctx.lineJoin = "round"
        function line(x1, y1, x2, y2) {
            ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke()
        }
        function circle(x, y, r) {
            ctx.beginPath(); ctx.arc(x, y, r, 0, Math.PI * 2); ctx.stroke()
        }
        if (name === "camera") {
            ctx.beginPath(); ctx.moveTo(3, 7); ctx.lineTo(7, 7); ctx.lineTo(9, 4)
            ctx.lineTo(15, 4); ctx.lineTo(17, 7); ctx.lineTo(21, 7)
            ctx.lineTo(21, 20); ctx.lineTo(3, 20); ctx.closePath(); ctx.stroke()
            circle(12, 13, 4)
        } else if (name === "shutter") {
            circle(12, 12, 9); circle(12, 12, 3.6)
            for (var i = 0; i < 6; ++i) {
                var a = i * Math.PI / 3
                line(12 + Math.cos(a) * 3.6, 12 + Math.sin(a) * 3.6,
                     12 + Math.cos(a - 0.35) * 8.9, 12 + Math.sin(a - 0.35) * 8.9)
            }
        } else if (name === "aperture") {
            circle(12, 12, 9)
            var t = Math.sqrt(81 - 16)
            ctx.beginPath()
            for (var j = 0; j < 6; ++j) {
                var b = j * Math.PI / 3
                var px = 12 + Math.cos(b) * 4, py = 12 + Math.sin(b) * 4
                if (j === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py)
            }
            ctx.closePath(); ctx.stroke()
            for (var k = 0; k < 6; ++k) {
                var c = k * Math.PI / 3
                var x0 = 12 + Math.cos(c) * 4, y0 = 12 + Math.sin(c) * 4
                line(x0, y0, x0 - Math.sin(c) * t, y0 + Math.cos(c) * t)
            }
        } else if (name === "spinner") {
            ctx.beginPath(); ctx.arc(12, 12, 8, 0, Math.PI * 1.5); ctx.stroke()
        } else if (name === "save") {
            ctx.beginPath(); ctx.moveTo(4, 3); ctx.lineTo(17, 3); ctx.lineTo(21, 7)
            ctx.lineTo(21, 21); ctx.lineTo(4, 21); ctx.closePath(); ctx.stroke()
            ctx.strokeRect(8, 3, 8, 6); ctx.strokeRect(8, 14, 9, 7)
        } else if (name === "folder") {
            ctx.beginPath(); ctx.moveTo(3, 6); ctx.lineTo(10, 6); ctx.lineTo(12, 9)
            ctx.lineTo(21, 9); ctx.lineTo(21, 20); ctx.lineTo(3, 20)
            ctx.closePath(); ctx.stroke(); line(3, 11, 21, 11)
        } else if (name === "pause") {
            line(8, 5, 8, 19); line(16, 5, 16, 19)
        } else if (name === "play") {
            ctx.beginPath(); ctx.moveTo(8, 5); ctx.lineTo(19, 12); ctx.lineTo(8, 19)
            ctx.closePath(); ctx.stroke()
        } else if (name === "level") {
            line(3, 18, 21, 18); line(12, 3, 12, 13)
            line(8, 9, 12, 13); line(16, 9, 12, 13)
            line(5, 21, 5, 18); line(19, 21, 19, 18)
        } else if (name === "undistort") {
            line(3, 8, 3, 3); line(3, 3, 8, 3)
            line(16, 3, 21, 3); line(21, 3, 21, 8)
            line(21, 16, 21, 21); line(21, 21, 16, 21)
            line(8, 21, 3, 21); line(3, 21, 3, 16)
            line(7, 12, 17, 12)
        }
    }
}
