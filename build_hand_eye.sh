#!/usr/bin/env bash
set -euo pipefail

# Build the MVS GUI and hardware acceptance tool.
# Usage: ./build_hand_eye.sh [mvs]
if [[ $# -gt 1 || "${1:-mvs}" != "mvs" ]]; then
    echo "用法: $0 [mvs]（仅支持 MVS）" >&2
    exit 1
fi

build_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
build_dir="$build_root/build_mvs"
cmake -S "$build_root" -B "$build_dir" \
    -DCAMERA_BACKEND=MVS -DBUILD_QT_GUI=ON -DCMAKE_BUILD_TYPE=Release
cmake --build "$build_dir" --target hand-eye-app mvs_hw_acceptance \
    -j "${COLLECT_BUILD_JOBS:-4}"

for name in collect-mvs mvs_hw_acceptance; do
    bin="$build_dir/$name"
    if [[ ! -x "$bin" ]]; then
        echo "[错误] 缺少可执行文件 $bin" >&2
        exit 1
    fi
    file "$bin"
    sha256sum "$bin"
    dependencies="$(ldd "$bin")"
    if grep -q "not found" <<< "$dependencies"; then
        echo "[错误] $name 存在未解析的动态库:" >&2
        echo "$dependencies" >&2
        exit 1
    fi
    if ! grep -qi "MvCameraControl" <<< "$dependencies"; then
        echo "[错误] $name 未链接 MVS 运行库" >&2
        exit 1
    fi
done
printf '构建完成：%s/collect-mvs（标定界面使用 --calibration）\n' "$build_dir"
