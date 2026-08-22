#!/bin/bash
set -e

# ============================================================
#  hand-eye-mvs 目标机交付校验脚本
#
#  用法（在目标机上、发布包目录内执行）:
#    ./check_mvs_target.sh [发布包目录]
#
#  校验内容:
#    1. 二进制架构与当前机器一致；
#    2. ldd 无 not found（优先使用包内 lib/mvs，其次 /opt/MVS）；
#    3. 未捆绑运行库时，目标机必须已安装 /opt/MVS，否则明确失败；
#    4. 打印 MVS 运行库版本、相机默认配置是否就位。
# ============================================================

PKG_DIR="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
BIN="$PKG_DIR/hand-eye-mvs"
MVS_SDK_DIR="/opt/MVS"

echo "========================================"
echo "  hand-eye-mvs 目标机交付校验"
echo "  目录: $PKG_DIR"
echo "========================================"

fail() { echo "[失败] $1"; exit 1; }

[ -x "$BIN" ] || fail "未找到可执行文件 $BIN"

# ---- 1. 架构 ----
HOST_ARCH="$(uname -m)"
file "$BIN" | grep -q "$HOST_ARCH" \
    || fail "二进制架构与目标机($HOST_ARCH)不符: $(file -b "$BIN")"
echo "1. 架构: $HOST_ARCH OK"

# ---- 2. 运行库解析 ----
if [ -d "$PKG_DIR/lib/mvs" ] && compgen -G "$PKG_DIR/lib/mvs/libMvCameraControl.so*" > /dev/null; then
    echo "2. 使用包内 MVS 运行库: $PKG_DIR/lib/mvs"
    LDD_LIBS="$PKG_DIR/lib/mvs"
elif [ -f "$MVS_SDK_DIR/lib/aarch64/libMvCameraControl.so" ] || \
     [ -f "$MVS_SDK_DIR/lib/64/libMvCameraControl.so" ]; then
    echo "2. 包内无运行库，使用目标机 $MVS_SDK_DIR"
    case "$HOST_ARCH" in
        aarch64|arm64) LDD_LIBS="$MVS_SDK_DIR/lib/aarch64" ;;
        x86_64)        LDD_LIBS="$MVS_SDK_DIR/lib/64" ;;
    esac
else
    fail "包内无 lib/mvs 且目标机未安装 $MVS_SDK_DIR，无法解析 libMvCameraControl.so"
fi

if env LD_LIBRARY_PATH="$LDD_LIBS" ldd "$BIN" | grep -q "not found"; then
    env LD_LIBRARY_PATH="$LDD_LIBS" ldd "$BIN" | grep "not found"
    fail "存在未解析的动态库"
fi
echo "   ldd: 无缺失库"

# ---- 3. MVS 运行库版本 ----
MVS_CORE="$(ls "$LDD_LIBS"/libMvCameraControl.so.* 2>/dev/null | head -1 || true)"
if [ -n "$MVS_CORE" ]; then
    echo "3. MVS 运行库: $(basename "$MVS_CORE")"
else
    echo "3. MVS 运行库: libMvCameraControl.so (无版本后缀)"
fi

# ---- 4. 配置与参数目录 ----
[ -f "$PKG_DIR/config.yaml" ] \
    && echo "4. config.yaml: 就位" \
    || echo "4. config.yaml: 缺失（将使用编译期默认值）"
[ -d "$PKG_DIR/param" ] || mkdir -p "$PKG_DIR/param"
if [ -f "$PKG_DIR/param/camera_params.yaml" ]; then
    echo "   param/camera_params.yaml: 就位（MVS 标定必须依赖此外部内参）"
else
    echo "   param/camera_params.yaml: 缺失 —— MVS 眼在手内标定前必须放置匹配该机的外参内参文件"
fi

echo ""
echo "校验通过"
