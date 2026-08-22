#!/bin/bash
set -e

# ============================================================
#  hand-eye-mvs 发布包打包脚本
#
#  用法:
#    ./tools/package_mvs_release.sh
#    ./tools/package_mvs_release.sh --build-dir build_mvs --output dist/hand-eye-mvs
#    ./tools/package_mvs_release.sh --no-bundle-libs
#
#  产物布局:
#    <output>/hand-eye-mvs          可执行文件（RPATH 含 $ORIGIN/lib/mvs）
#    <output>/lib/mvs/              MVS 运行库（默认捆绑，可用 --no-bundle-libs 关闭）
#    <output>/config.yaml           配置样例（若发布目录还没有）
#    <output>/param/                参数目录（内参、手眼结果）
#    <output>/BUILD_INFO.txt        构建身份（commit / 时间 / 架构 / SHA256）
#    <output>/SHA256SUMS            文件校验清单
#
#  注意: 海康 MVS SDK 运行库通常允许随应用分发，但须遵循 SDK 许可条款。
#  若不允许随包分发，使用 --no-bundle-libs，目标机必须已安装 /opt/MVS，
#  部署时用 tools/check_mvs_target.sh 校验，缺失会明确失败。
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

BIN="$BUILD_DIR/hand-eye-mvs"
if [ ! -x "$BIN" ]; then
    echo "[错误] 未找到 $BIN，请先运行 ./build_hand_eye.sh mvs"
    exit 1
fi

if [ -z "$OUTPUT_DIR" ]; then
    OUTPUT_DIR="$REPO_DIR/dist/hand-eye-mvs-$(date +%Y%m%d-%H%M%S)"
fi
mkdir -p "$OUTPUT_DIR/lib/mvs" "$OUTPUT_DIR/param"

echo "========================================"
echo "  打包 hand-eye-mvs → $OUTPUT_DIR"
echo "========================================"

# ---- 架构检查 ----
if ! file "$BIN" | grep -q "aarch64"; then
    echo "[警告] 二进制不是 aarch64 架构:"
    file "$BIN"
fi

cp -d "$BIN" "$OUTPUT_DIR/hand-eye-mvs"

# ---- MVS 运行库 ----
case "$(uname -m)" in
    aarch64|arm64) MVS_LIB_DIR="$MVS_SDK_DIR/lib/aarch64" ;;
    x86_64)        MVS_LIB_DIR="$MVS_SDK_DIR/lib/64" ;;
    *)             MVS_LIB_DIR="$MVS_SDK_DIR/lib/32" ;;
esac

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
        libGCBase_gcc494_v3_0.so
        libGenCP_gcc494_v3_0.so
        libLog_gcc494_v3_0.so
        liblog4cpp_gcc494_v3_0.so
    )
    for lib in "${MVS_RUNTIME_LIBS[@]}"; do
        if compgen -G "$MVS_LIB_DIR/${lib}*" > /dev/null; then
            cp -d "$MVS_LIB_DIR/${lib}"* "$OUTPUT_DIR/lib/mvs/"
        else
            echo "[警告] MVS 组件缺失，未打包: $lib"
        fi
    done
    # GenTL producer 与运行参数文件
    for extra in MvProducerGEV.cti MvProducerU3V.cti CommonParameters.ini; do
        [ -f "$MVS_LIB_DIR/$extra" ] && cp -d "$MVS_LIB_DIR/$extra" "$OUTPUT_DIR/lib/mvs/"
    done
    echo "  已捆绑 MVS 运行库: $(ls "$OUTPUT_DIR/lib/mvs" | wc -l) 个文件"
else
    echo "  --no-bundle-libs: 不捆绑 MVS 运行库，目标机必须安装 $MVS_SDK_DIR"
fi

# ---- 配置样例（不覆盖已有配置） ----
if [ ! -f "$OUTPUT_DIR/config.yaml" ] && [ -f "$REPO_DIR/build/config.yaml" ]; then
    cp "$REPO_DIR/build/config.yaml" "$OUTPUT_DIR/config.yaml"
    echo "  已复制配置样例 config.yaml"
fi

# ---- 目标机校验脚本与真机验收工具随包分发 ----
cp "$REPO_DIR/tools/check_mvs_target.sh" "$OUTPUT_DIR/check_mvs_target.sh"
chmod +x "$OUTPUT_DIR/check_mvs_target.sh"
if [ -f "$BUILD_DIR/mvs_hw_acceptance" ]; then
    cp -d "$BUILD_DIR/mvs_hw_acceptance" "$OUTPUT_DIR/mvs_hw_acceptance"
    echo "  已打包真机验收工具 mvs_hw_acceptance"
fi

# ---- RPATH 检查 ----
if ! readelf -d "$OUTPUT_DIR/hand-eye-mvs" | grep -q '\$ORIGIN/lib/mvs'; then
    echo "[错误] 二进制 RPATH 缺少 \$ORIGIN/lib/mvs，发布包无法自包含运行"
    readelf -d "$OUTPUT_DIR/hand-eye-mvs" | grep -iE "rpath|runpath" || true
    exit 1
fi
echo "  RPATH 包含 \$ORIGIN/lib/mvs: OK"

# ---- ldd 校验 ----
if [ "$BUNDLE_LIBS" -eq 1 ]; then
    LDD_ENV="LD_LIBRARY_PATH=$OUTPUT_DIR/lib/mvs"
else
    LDD_ENV="LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}"
fi
if env $LDD_ENV ldd "$OUTPUT_DIR/hand-eye-mvs" | grep -q "not found"; then
    echo "[错误] 发布包存在未解析的动态库:"
    env $LDD_ENV ldd "$OUTPUT_DIR/hand-eye-mvs" | grep "not found"
    exit 1
fi
echo "  ldd: 无缺失库"

# ---- 构建身份与校验清单 ----
GIT_COMMIT="$(git -C "$REPO_DIR" rev-parse --short=12 HEAD 2>/dev/null || echo unknown)"
GIT_STATE="$(if [ -z "$(git -C "$REPO_DIR" status --porcelain 2>/dev/null)" ]; then echo clean; else echo dirty; fi)"
{
    echo "package_time_utc: $(date -u '+%Y-%m-%dT%H:%M:%SZ')"
    echo "git_commit: $GIT_COMMIT"
    echo "git_worktree_state: $GIT_STATE"
    echo "camera_backend: mvs"
    echo "bundle_libs: $BUNDLE_LIBS"
    echo "source_binary: $BIN"
    echo "binary_sha256: $(sha256sum "$BIN" | awk '{print $1}')"
} > "$OUTPUT_DIR/BUILD_INFO.txt"

( cd "$OUTPUT_DIR" && sha256sum hand-eye-mvs lib/mvs/* 2>/dev/null > SHA256SUMS )

echo ""
echo "  打包完成: $OUTPUT_DIR"
echo "  部署后在目标机运行 tools/check_mvs_target.sh 做交付校验"
