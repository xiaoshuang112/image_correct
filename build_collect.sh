#!/usr/bin/env bash
set -euo pipefail
collect_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
build_jobs="${COLLECT_BUILD_JOBS:-}"
if [[ -z "$build_jobs" ]]; then
    case "$(uname -m)" in
        x86_64|amd64|i686|i386) build_jobs=32 ;;
        aarch64|arm64|arm*) build_jobs=4 ;;
        *) build_jobs=4 ;;
    esac
fi
cmake -S "$collect_root" -B "$collect_root/build_mvs" \
    -DCAMERA_BACKEND=MVS -DBUILD_QT_GUI=ON -DBUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Release "$@"
cmake --build "$collect_root/build_mvs" -j "$build_jobs"
ctest --test-dir "$collect_root/build_mvs" --output-on-failure
printf '采集程序：%s/build_mvs/collect-mvs\n' "$collect_root"
