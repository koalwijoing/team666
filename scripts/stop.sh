#!/bin/bash
# Request the exact app's normal GUI exit; do not force-stop capture.
set -euo pipefail
if [ "$(id -u)" -ne 0 ]; then
    exec sudo -n bash "$0" "$@"
fi
export DISPLAY=:0 XAUTHORITY=/home/linaro/.Xauthority
exec python3 /userdata/codex_lane_interaction_release_20261028_r01/scripts/release_ctl.py stop
