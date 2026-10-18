#!/usr/bin/env bash
set -euo pipefail
collect_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cmake -S "$collect_root" -B "$collect_root/build_mvs" \
    -DCAMERA_BACKEND=MVS -DBUILD_QT_GUI=ON -DBUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Release "$@"
cmake --build "$collect_root/build_mvs" -j "${COLLECT_BUILD_JOBS:-4}"
ctest --test-dir "$collect_root/build_mvs" --output-on-failure
printf '采集程序：%s/build_mvs/collect-mvs\n' "$collect_root"
