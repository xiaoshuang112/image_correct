pragma Singleton
import QtQuick 2.15
QtObject {
    readonly property color bg: "#0B0E13"
    readonly property color surface: "#131720"
    readonly property color surfaceAlt: "#0D1119"
    readonly property color raised: "#1A202C"
    readonly property color raisedHover: "#232B3B"
    readonly property color hairline: "#262E3D"
    readonly property color hairlineStrong: "#354054"
    readonly property color ink: "#EDF1F7"
    readonly property color ink2: "#94A0B4"
    readonly property color ink3: "#637085"
    readonly property color accent: "#3B82F6"
    readonly property color accentHover: "#2563EB"
    readonly property color accentLine: "#993B82F6"
    readonly property color accentSoft: "#283B82F6"
    readonly property color accentSelection: "#663B82F6"
    readonly property color ok: "#22C55E"
    readonly property color okSoft: "#1F22C55E"
    readonly property color okLine: "#6622C55E"
    readonly property color okText: "#86EFAC"
    readonly property color warning: "#F59E0B"
    readonly property color warningSoft: "#22F59E0B"
    readonly property color warningLine: "#80F59E0B"
    readonly property color warningText: "#FCD34D"
    readonly property color danger: "#EF4444"
    readonly property color dangerHover: "#DC2626"
    readonly property color viewfinder: "#05070A"
    readonly property color overlay: "#D60B0E13"
    readonly property color chip: "#CC0B0E13"
    readonly property color white: "#FFFFFF"
    readonly property color axisX: "#F87171"
    readonly property color axisY: "#34D399"
    readonly property var axisColors: ["#F87171", "#34D399", "#60A5FA", "#FB923C", "#A78BFA", "#22D3EE"]
    readonly property string sans: "Noto Sans CJK SC"
    readonly property string mono: "Noto Sans Mono CJK SC"
    readonly property int micro: 11
    readonly property int meta: 12
    readonly property int body: 14
    readonly property int title: 16
    readonly property int appTitle: 19
    readonly property int radiusCard: 10
    readonly property int radiusControl: 6
    readonly property int radiusSmall: 6
    readonly property int controlHeight: 40
    readonly property int padding: 18
    readonly property int pageContentWidth: 1120
    readonly property int pageCardMinHeight: 340
}
