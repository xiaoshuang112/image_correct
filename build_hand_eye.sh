#!/bin/bash
set -e

# ============================================================
#  手眼标定 GUI 编译脚本
#  用法:
#    ./build_hand_eye.sh        # 编译 MVS 版本 → build_mvs/hand-eye-mvs
#    ./build_hand_eye.sh orb    # 编译 ORB 版本 → build_orb/hand-eye-orb
#    ./build_hand_eye.sh all    # 两个都编译
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

build_target() {
    local cam_type="$1"
    local build_dir="$2"
    local cmake_args="$3"

    echo ""
    echo "========================================"
    echo "  编译 hand-eye-${cam_type}"
    echo "========================================"

    mkdir -p "$build_dir"
    cd "$build_dir"
    cmake .. $cmake_args
    make hand-eye-app -j$(nproc)
    cd "$SCRIPT_DIR"

    echo "  完成: ${build_dir}/hand-eye-${cam_type}"
}

case "${1:-mvs}" in
    mvs)
        build_target "mvs" "build_mvs" "-DUSE_ORB=OFF"
        ;;
    orb)
        build_target "orb" "build_orb" "-DUSE_ORB=ON"
        ;;
    all)
        build_target "mvs" "build_mvs" "-DUSE_ORB=OFF"
        build_target "orb" "build_orb" "-DUSE_ORB=ON"
        ;;
    *)
        echo "用法: $0 {mvs|orb|all}"
        exit 1
        ;;
esac
