#!/bin/bash
# Start the frozen default and its existing keyboard bridge.
set -euo pipefail
if [ "$(id -u)" -ne 0 ]; then
    exec sudo -n bash "$0" "$@"
fi
export DISPLAY=:0 XAUTHORITY=/home/linaro/.Xauthority
bash /userdata/claude_qt_async_default_20261021/start_qt_async_default.sh live
bash /userdata/claude_four_capture_20261023/keyinject_ctl.sh start
