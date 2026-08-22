#!/bin/bash
set -e

# ============================================================
#  手眼标定 GUI 编译脚本
#  用法:
#    ./build_hand_eye.sh        # 编译 MVS 版本 → build_mvs/hand-eye-mvs
#    ./build_hand_eye.sh mvs    # 同上
#    ./build_hand_eye.sh orb    # 编译 ORB 版本 → build_orb/hand-eye-orb
#    ./build_hand_eye.sh all    # 两个都编译（均为 Release）
#
#  相机后端通过 -DCAMERA_BACKEND=ORB|MVS 传给 CMake；
#  旧的 -DUSE_ORB=ON/OFF 入口仍然兼容。
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

build_target() {
    local cam_type="$1"
    local build_dir="$2"
    local cmake_args="$3"

    echo ""
    echo "========================================"
    echo "  编译 hand-eye-${cam_type} (Release)"
    echo "========================================"

    mkdir -p "$build_dir"
    cd "$build_dir"
    cmake .. -DCMAKE_BUILD_TYPE=Release $cmake_args
    if [ "$cam_type" = "mvs" ]; then
        # 发布包必备：GUI 与真机验收工具必须同时产出
        make hand-eye-app mvs_hw_acceptance -j$(nproc)
    else
        make hand-eye-app -j$(nproc)
    fi
    cd "$SCRIPT_DIR"

    echo "  完成: ${build_dir}/hand-eye-${cam_type}"
}

verify_target() {
    local cam_type="$1"
    local build_dir="$2"
    local bin="${build_dir}/hand-eye-${cam_type}"

    echo ""
    echo "---- 校验 ${bin} ----"
    file "$bin"
    sha256sum "$bin"
    if ldd "$bin" | grep -q "not found"; then
        echo "  [错误] 存在未解析的动态库:"
        ldd "$bin" | grep "not found"
        exit 1
    fi
    echo "  ldd: 无缺失库"
    if [ "$cam_type" = "orb" ]; then
        if ldd "$bin" | grep -qi "MvCameraControl"; then
            echo "  [错误] ORB 产物不应依赖 MVS 运行库"
            exit 1
        fi
        echo "  未链接 libMvCameraControl.so: 符合预期"
    else
        if ! ldd "$bin" | grep -qi "MvCameraControl"; then
            echo "  [错误] MVS 产物未链接 libMvCameraControl.so"
            exit 1
        fi
        echo "  已链接 libMvCameraControl.so: 符合预期"
        local hw="${build_dir}/mvs_hw_acceptance"
        if [ ! -x "$hw" ]; then
            echo "  [错误] 缺少真机验收工具 $hw"
            exit 1
        fi
        if ldd "$hw" | grep -q "not found"; then
            echo "  [错误] mvs_hw_acceptance 存在未解析的动态库:"
            ldd "$hw" | grep "not found"
            exit 1
        fi
        if ! ldd "$hw" | grep -qi "MvCameraControl"; then
            echo "  [错误] mvs_hw_acceptance 未链接 libMvCameraControl.so"
            exit 1
        fi
        echo "  mvs_hw_acceptance: 可执行且已链接 MVS 运行库"
    fi
}

case "${1:-mvs}" in
    mvs)
        build_target "mvs" "build_mvs" "-DCAMERA_BACKEND=MVS"
        verify_target "mvs" "build_mvs"
        ;;
    orb)
        build_target "orb" "build_orb" "-DCAMERA_BACKEND=ORB"
        verify_target "orb" "build_orb"
        ;;
    all)
        build_target "mvs" "build_mvs" "-DCAMERA_BACKEND=MVS"
        build_target "orb" "build_orb" "-DCAMERA_BACKEND=ORB"
        verify_target "mvs" "build_mvs"
        verify_target "orb" "build_orb"
        ;;
    *)
        echo "用法: $0 {mvs|orb|all}"
        exit 1
        ;;
esac
