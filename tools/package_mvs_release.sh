#!/bin/bash
set -e

# ============================================================
#  collect-mvs 发布包打包脚本
#
#  用法:
#    ./tools/package_mvs_release.sh
#    ./tools/package_mvs_release.sh --build-dir build_mvs --output dist/collect-mvs
#    ./tools/package_mvs_release.sh --no-bundle-libs
#
#  产物布局:
#    <output>/collect-mvs          可执行文件（捆绑运行库时 RPATH 含 $ORIGIN/lib/mvs）
#    <output>/check_mvs_target.sh   目标机交付校验脚本
#    <output>/lib/mvs/              MVS 运行库（默认捆绑，可用 --no-bundle-libs 关闭）
#    <output>/config.yaml           配置样例（若发布目录还没有）
#    <output>/param/                参数目录（内参、手眼结果）
#    <output>/BUILD_INFO.txt        构建身份（commit / 时间 / 架构 / SHA256）
#    <output>/SHA256SUMS            文件校验清单
#
#  注意: 海康 MVS SDK 运行库通常允许随应用分发，但须遵循 SDK 许可条款。
#  若不允许随包分发，使用 --no-bundle-libs，目标机必须已安装 /opt/MVS，
#  部署时在发布包目录内执行 ./check_mvs_target.sh 校验，缺失会明确失败。
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

BUILD_DIR="$REPO_DIR/build_mvs"
OUTPUT_DIR=""
BUNDLE_LIBS=1
MVS_SDK_DIR="/opt/MVS"

while [ $# -gt 0 ]; do
    case "$1" in
        --build-dir) BUILD_DIR="$2"; shift 2 ;;
        --output)    OUTPUT_DIR="$2"; shift 2 ;;
        --no-bundle-libs) BUNDLE_LIBS=0; shift ;;
        *) echo "未知参数: $1"; exit 1 ;;
    esac
done

BIN="$BUILD_DIR/collect-mvs"
if [ ! -x "$BIN" ]; then
    echo "[错误] 未找到 $BIN，请先运行 ./build_collect.sh"
    exit 1
fi

if [ -z "$OUTPUT_DIR" ]; then
    OUTPUT_DIR="$REPO_DIR/dist/collect-mvs-$(date +%Y%m%d-%H%M%S)"
fi
mkdir -p "$OUTPUT_DIR/lib/mvs" "$OUTPUT_DIR/param"
shopt -s nullglob
existing_libs=("$OUTPUT_DIR/lib/mvs/"*)
if [ "$BUNDLE_LIBS" -eq 0 ] && [ "${#existing_libs[@]}" -gt 0 ]; then
    echo "[错误] --no-bundle-libs 需要空的 lib/mvs，请使用新的发布目录: $OUTPUT_DIR"
    exit 1
fi

echo "========================================"
echo "  打包 collect-mvs → $OUTPUT_DIR"
echo "========================================"

# ---- 二进制架构与对应的 MVS SDK 目录 ----
BIN_DESCRIPTION="$(LC_ALL=C file -Lb -- "$BIN")"
case "$BIN_DESCRIPTION" in
    ELF\ 64-bit*,\ x86-64,*) BIN_ARCH=x86_64; ELF_ARCH="x86-64"; MVS_LIB_SUBDIR=64 ;;
    ELF\ 64-bit*,\ ARM\ aarch64,*) BIN_ARCH=aarch64; ELF_ARCH="ARM aarch64"; MVS_LIB_SUBDIR=aarch64 ;;
    *) echo "[错误] 不支持的 collect-mvs 二进制架构: $BIN_DESCRIPTION"; exit 1 ;;
esac
HOST_ARCH="$(uname -m)"
case "$HOST_ARCH" in
    amd64) HOST_ARCH=x86_64 ;;
    arm64) HOST_ARCH=aarch64 ;;
esac
echo "  二进制架构: $BIN_ARCH"

cp -L "$BIN" "$OUTPUT_DIR/collect-mvs"

# ---- MVS 运行库 ----
if [ -f "$BUILD_DIR/CMakeCache.txt" ]; then
    CACHED_SDK_DIR="$(sed -n 's/^MVS_SDK_DIR:PATH=//p' "$BUILD_DIR/CMakeCache.txt")"
    MVS_SDK_DIR="${CACHED_SDK_DIR:-$MVS_SDK_DIR}"
fi
MVS_LIB_DIR="$MVS_SDK_DIR/lib/$MVS_LIB_SUBDIR"

copy_runtime_file() {
    local source="$1"
    local target="$OUTPUT_DIR/lib/mvs/${source##*/}"
    if [ -L "$source" ] && [[ "$(readlink "$source")" == */* ]]; then
        # 外部链接指向 SDK 本地文件，包中需要实际内容。
        cp -L --remove-destination "$source" "$target"
    else
        cp -d --remove-destination "$source" "$target"
    fi
}

if [ "$BUNDLE_LIBS" -eq 1 ]; then
    if [ ! -f "$MVS_LIB_DIR/libMvCameraControl.so" ]; then
        echo "[错误] 本机缺少 MVS 运行库: $MVS_LIB_DIR/libMvCameraControl.so"
        exit 1
    fi
    # libMvCameraControl 自身只依赖系统库，但 GigE/USB 取流在运行时
    # dlopen 这些组件，必须一并打包。
    MVS_RUNTIME_LIBS=(
        libMvCameraControl.so
        libMVGigEVisionSDK.so
        libMvUsb3vTL.so
        libFormatConversion.so
        libMediaProcess.so
        libMVRender.so
        libMvCamLVision.so
        libMvSDKVersion.so
        'libGCBase_gcc*_v3_0.so'
        'libGenCP_gcc*_v3_0.so'
        'libLog_gcc*_v3_0.so'
        'liblog4cpp_gcc*_v3_0.so'
    )
    for lib in "${MVS_RUNTIME_LIBS[@]}"; do
        # lib 来自上面的固定列表，允许匹配不同 SDK 版本的 GenICam 库。
        matches=("$MVS_LIB_DIR/"$lib "$MVS_LIB_DIR/"$lib.*)
        found=0
        for source in "${matches[@]}"; do
            [ -e "$source" ] || [ -L "$source" ] || continue
            copy_runtime_file "$source"
            found=1
        done
        if [ "$found" -eq 0 ]; then
            echo "[警告] MVS 组件缺失，未打包: $lib"
        fi
    done
    # GenTL producer 与运行参数文件
    for extra in MvProducerGEV.cti MvProducerU3V.cti CommonParameters.ini; do
        [ -f "$MVS_LIB_DIR/$extra" ] || continue
        copy_runtime_file "$MVS_LIB_DIR/$extra"
    done
    for library in "$OUTPUT_DIR/lib/mvs/"*.so "$OUTPUT_DIR/lib/mvs/"*.so.* "$OUTPUT_DIR/lib/mvs/"*.cti; do
        description="$(LC_ALL=C file -Lb -- "$library")"
        if [[ "$description" != ELF\ 64-bit*,\ "$ELF_ARCH",* ]]; then
            echo "[错误] $library 与二进制架构 $BIN_ARCH 不符: $description"
            exit 1
        fi
    done
    echo "  已捆绑 MVS 运行库: $(ls "$OUTPUT_DIR/lib/mvs" | wc -l) 个文件"
else
    echo "  --no-bundle-libs: 不捆绑 MVS 运行库，目标机必须安装 /opt/MVS"
fi

# ---- 配置样例（不覆盖已有配置） ----
if [ ! -f "$OUTPUT_DIR/config.yaml" ]; then
    cp "$REPO_DIR/config/collect.example.yaml" "$OUTPUT_DIR/config.yaml"
    echo "  已复制配置样例 config.yaml"
fi

# ---- 目标机校验脚本随包分发 ----
cp "$REPO_DIR/tools/check_mvs_target.sh" "$OUTPUT_DIR/check_mvs_target.sh"
chmod +x "$OUTPUT_DIR/check_mvs_target.sh"

# ---- RPATH / ldd 校验 ----
PACKED_BIN="$OUTPUT_DIR/collect-mvs"
if [ "$BUNDLE_LIBS" -eq 1 ]; then
    DYNAMIC="$(LC_ALL=C readelf -d "$PACKED_BIN")"
    if ! grep -Eq '\((RPATH|RUNPATH)\).*\[(.*:)?\$ORIGIN/lib/mvs(:|\])' <<< "$DYNAMIC"; then
        echo "[错误] collect-mvs RPATH 缺少 \$ORIGIN/lib/mvs，发布包无法自包含运行"
        grep -iE "rpath|runpath" <<< "$DYNAMIC" || true
        exit 1
    fi
    echo "  collect-mvs RPATH 包含 \$ORIGIN/lib/mvs: OK"
    LDD_LIBS="$OUTPUT_DIR/lib/mvs"
else
    LDD_LIBS="$MVS_LIB_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

if [ "$BIN_ARCH" != "$HOST_ARCH" ]; then
    echo "  collect-mvs ldd: 需在 $BIN_ARCH 目标机校验（打包主机架构 $HOST_ARCH）"
else
    if ! LDD_OUTPUT="$(LC_ALL=C LD_LIBRARY_PATH="$LDD_LIBS" ldd "$PACKED_BIN" 2>&1)"; then
        echo "$LDD_OUTPUT"
        echo "[错误] collect-mvs ldd 执行失败"
        exit 1
    fi
    if [[ "$LDD_OUTPUT" == *'not found'* ]]; then
        echo "[错误] collect-mvs 存在未解析的动态库:"
        grep 'not found' <<< "$LDD_OUTPUT"
        exit 1
    fi
    echo "  collect-mvs ldd: 无缺失库"
fi

# ---- 构建身份与校验清单 ----
GIT_COMMIT="$(git -C "$REPO_DIR" rev-parse --short=12 HEAD 2>/dev/null || echo unknown)"
GIT_STATE="$(if [ -z "$(git -C "$REPO_DIR" status --porcelain 2>/dev/null)" ]; then echo clean; else echo dirty; fi)"
{
    echo "package_time_utc: $(date -u '+%Y-%m-%dT%H:%M:%SZ')"
    echo "git_commit: $GIT_COMMIT"
    echo "git_worktree_state: $GIT_STATE"
    echo "camera_backend: mvs"
    echo "binary_arch: $BIN_ARCH"
    echo "bundle_libs: $BUNDLE_LIBS"
    echo "source_binary: $BIN"
    echo "binary_sha256: $(sha256sum "$BIN" | awk '{print $1}')"
} > "$OUTPUT_DIR/BUILD_INFO.txt"

(
    cd "$OUTPUT_DIR"
    sha256sum collect-mvs check_mvs_target.sh lib/mvs/* > SHA256SUMS
)

echo ""
echo "  打包完成: $OUTPUT_DIR"
echo "  部署后进入发布包目录执行："
echo "    ./check_mvs_target.sh"
