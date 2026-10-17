#!/usr/bin/env bash
set -euo pipefail
collect_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
collect_binary="$collect_root/build_mvs/collect-mvs"
if [[ ! -x "$collect_binary" ]]; then
    echo '请先运行 ./build_collect.sh（首次配置可传入 -DCGX_SDK_DIR=...）' >&2
    exit 1
fi
# WSLg sockets may be present even when the launching shell lacks DISPLAY.
if [[ -n "${WSL_DISTRO_NAME:-}" || -e /proc/sys/fs/binfmt_misc/WSLInterop ]]; then
    if [[ -z "${DISPLAY:-}" && -S /tmp/.X11-unix/X0 ]]; then
        export DISPLAY=:0
    fi
    export QT_QUICK_BACKEND="${QT_QUICK_BACKEND:-software}"
fi
cd "$collect_root/build_mvs"
exec "$collect_binary" "$@"
