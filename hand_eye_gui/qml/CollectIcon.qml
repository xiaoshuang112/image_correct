import QtQuick 2.15

Canvas {
    id: icon
    property string name: "camera"
    property color ink: "#9ba8bc"
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
        ctx.lineWidth = 1.6
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
        } else if (name === "stop") {
            ctx.strokeRect(6, 6, 12, 12)
        } else if (name === "connect") {
            line(9, 3, 9, 8); line(15, 3, 15, 8)
            ctx.beginPath(); ctx.moveTo(6, 8); ctx.lineTo(18, 8); ctx.lineTo(18, 12)
            ctx.arc(12, 12, 6, 0, Math.PI); ctx.lineTo(6, 8); ctx.stroke()
            line(12, 18, 12, 22)
        } else if (name === "check") {
            line(5, 12, 10, 17); line(10, 17, 19, 7)
        } else if (name === "arrow") {
            line(4, 12, 20, 12); line(14, 6, 20, 12); line(14, 18, 20, 12)
        }
    }
}
