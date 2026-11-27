#!/bin/bash
set -e

# ============================================================
#  collect-mvs 目标机交付校验脚本
#
#  用法（在目标机上、发布包目录内执行）:
#    ./check_mvs_target.sh [发布包目录]
#
#  校验内容:
#    1. 主程序存在、可执行，架构与当前机器一致；
#    2. ldd 无 not found（优先使用包内 lib/mvs，其次 /opt/MVS）；
#    3. 未捆绑运行库时，目标机必须已安装 /opt/MVS，否则明确失败；
#    4. 打印 MVS 运行库版本、相机默认配置是否就位。
# ============================================================

PKG_DIR="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
BIN="$PKG_DIR/collect-mvs"
MVS_SDK_DIR="/opt/MVS"

echo "========================================"
echo "  collect-mvs 目标机交付校验"
echo "  目录: $PKG_DIR"
echo "========================================"

fail() { echo "[失败] $1"; exit 1; }

[ -x "$BIN" ] || fail "未找到可执行文件 $BIN"

# ---- 1. 架构 ----
HOST_ARCH="$(uname -m)"
case "$HOST_ARCH" in
    x86_64|amd64) ELF_ARCH="x86-64"; MVS_LIB_DIR="$MVS_SDK_DIR/lib/64" ;;
    aarch64|arm64) ELF_ARCH="ARM aarch64"; MVS_LIB_DIR="$MVS_SDK_DIR/lib/aarch64" ;;
    *) fail "不支持的目标机架构: $HOST_ARCH" ;;
esac
check_arch() {
    local path="$1"
    local name="$2"
    local description
    description="$(LC_ALL=C file -Lb -- "$path")"
    [[ "$description" == ELF\ 64-bit*,\ "$ELF_ARCH",* ]] \
        || fail "$name 架构与目标机($HOST_ARCH)不符: $description"
}
check_arch "$BIN" "collect-mvs"
echo "1. 架构: $HOST_ARCH OK（collect-mvs）"

# ---- 2. 运行库解析 ----
shopt -s nullglob
bundled_core=("$PKG_DIR/lib/mvs/"libMvCameraControl.so*)
if [ "${#bundled_core[@]}" -gt 0 ]; then
    echo "2. 使用包内 MVS 运行库: $PKG_DIR/lib/mvs"
    LDD_LIBS="$PKG_DIR/lib/mvs"
elif [ -f "$MVS_LIB_DIR/libMvCameraControl.so" ]; then
    echo "2. 包内无运行库，使用目标机 $MVS_SDK_DIR"
    LDD_LIBS="$MVS_LIB_DIR"
else
    fail "包内无 MVS 运行库，目标机缺少 $MVS_LIB_DIR/libMvCameraControl.so"
fi
for core in "$LDD_LIBS/"libMvCameraControl.so*; do
    check_arch "$core" "$(basename "$core")"
done

if ! LDD_OUTPUT="$(LC_ALL=C LD_LIBRARY_PATH="$LDD_LIBS" ldd "$BIN" 2>&1)"; then
    echo "$LDD_OUTPUT"
    fail "collect-mvs ldd 执行失败"
fi
if [[ "$LDD_OUTPUT" == *'not found'* ]]; then
    grep 'not found' <<< "$LDD_OUTPUT"
    fail "collect-mvs 存在未解析的动态库"
fi
echo "   collect-mvs ldd: 无缺失库"

# ---- 3. MVS 运行库版本 ----
versioned_core=("$LDD_LIBS/"libMvCameraControl.so.*)
if [ "${#versioned_core[@]}" -gt 0 ]; then
    echo "3. MVS 运行库: $(basename "${versioned_core[0]}")"
else
    echo "3. MVS 运行库: libMvCameraControl.so (无版本后缀)"
fi

# ---- 4. 配置与参数目录 ----
[ -f "$PKG_DIR/config.yaml" ] \
    && echo "4. config.yaml: 就位" \
    || echo "4. config.yaml: 缺失（将使用编译期默认值）"
[ -d "$PKG_DIR/param" ] || mkdir -p "$PKG_DIR/param"
if [ -f "$PKG_DIR/param/cam1_hand_eye_result.yaml" ]; then
    echo "   相机 1 param/cam1_hand_eye_result.yaml: 就位（有身份和尺寸字段时校验）"
elif [ -f "$PKG_DIR/param/hand_eye_result.yaml" ]; then
    echo "   相机 1 param/hand_eye_result.yaml: 就位（兼容旧文件，有身份和尺寸字段时校验）"
else
    echo "   相机 1 缺少 param/cam1_hand_eye_result.yaml 和 param/hand_eye_result.yaml（仍可原图采集）"
fi
if [ -f "$PKG_DIR/param/cam2_hand_eye_result.yaml" ]; then
    echo "   相机 2 param/cam2_hand_eye_result.yaml: 就位（有身份和尺寸字段时校验）"
else
    echo "   相机 2 param/cam2_hand_eye_result.yaml: 缺失（可选相机，启用后仍可原图采集）"
fi

echo ""
echo "校验通过"
