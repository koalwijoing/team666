#!/bin/bash
# Qt 异步完整版（CODEX r01, ELF f8bf3b14）默认入口，CLAUDE 2026-10-21。
# 只读引用 CODEX 候选树，不修改其文件；运行输出在本目录 runs/。
#   live     : 停止原 v9（若在跑）后启动 Qt 异步默认
#   smoke    : 180 帧前台有限窗口（先停任何采集者）
#   stop     : 精确停止本入口实例（PID/exe 核验）
#   rollback : 停止本入口并恢复原 v9 默认入口
set -euo pipefail
ROOT=/userdata/claude_qt_async_default_20261021
CAND=/userdata/codex_qt_main_actor_app_20261021_r01
# [RED-MEMORY-FIX temp 2026-10-21] link-time fix: same frozen objects + patched bridge.o
APP=/userdata/codex_lane_interaction_release_20261028_r01/build/app
APP_SHA=3b1d686cb900673a92a36d9c6038da65c37c2c653e43cccb01784851319a3afd
V9=/userdata/display_upgrade_20261019_twinauth_v9
V9_ENTRY=$V9/start_twinauth_v9_default.sh
V9_APP=$V9/build/app_display_upgrade
DESKTOP=/home/linaro/Desktop/P3_ADAS_Default.desktop
DESKTOP_BAK=$DESKTOP.bak_v9_20261021
DEV=/dev/pango_pci_driver
MODE=${1:-live}
mkdir -p "$ROOT/runs"
exec 9>"$ROOT/default.lock"
flock -n 9 || { echo 'qt async default already owns the lock' >&2; exit 5; }
case "$MODE" in
  live|smoke) ;;
  stop)
    exec 9>&-
    exec python3 /userdata/codex_lane_interaction_release_20261028_r01/scripts/release_ctl.py stop ;;
  rollback)
    exec 9>&-
    exec python3 /userdata/codex_lane_interaction_release_20261028_r01/scripts/release_ctl.py rollback ;;
  *)
    echo 'usage: start_qt_async_default.sh [live|smoke|stop|rollback]'; exit 2 ;;
esac
[ "$(sha256sum "$APP" | cut -d' ' -f1)" = "$APP_SHA" ] || { echo 'REFUSE: candidate ELF SHA mismatch' >&2; exit 9; }
test -e "$DEV" || { echo 'REFUSE: /dev/pango_pci_driver missing (insmod required)' >&2; exit 10; }
test -S /tmp/.X11-unix/X0
# 停止可能正在运行的原 v9 默认（精确、经其自身入口）
if [ -x "$V9_ENTRY" ]; then
  bash "$V9_ENTRY" stop >/dev/null 2>&1 || true
fi
for p in $(ls /proc | grep -E '^[0-9]+$'); do
  [ -e "/proc/$p/fd" ] || continue
  ls -l "/proc/$p/fd" 2>/dev/null | grep -q pango_pci_driver && { echo "REFUSE: capture owner $p" >&2; exit 11; }
done
OUT=$ROOT/runs/$(date +%Y%m%d_%H%M%S)_${MODE}_qt_async
mkdir -p "$OUT"
printf '%s\n' "$OUT" > "$ROOT/latest_run.txt"
export DISPLAY=:0 XAUTHORITY=/home/linaro/.Xauthority
export LD_LIBRARY_PATH=/root/board_deploy/p3_app/app_pcie_twin_demo_20261009/hyperlpr3/lib:${LD_LIBRARY_PATH:-}
# User-selected original TwinLite + pure Pad; CAAM disabled, 2026-10-27.
TWIN_PAD_MODEL=/userdata/codex_rknn_accept_20261026_r01/models/twin_pad.rknn
TWIN_PAD_SHA=938040480f56a93c31447ea303abb7860cb45740b716968c3ec66f511d4ec0a0
[ "$(sha256sum "$TWIN_PAD_MODEL" | cut -d' ' -f1)" = "$TWIN_PAD_SHA" ] || { echo 'REFUSE: pure Pad Twin SHA mismatch' >&2; exit 14; }
export P3_TEST_TWIN_MODEL="$TWIN_PAD_MODEL"
# User-accepted simulated lane interaction, 2026-10-28.
export P3_AD_REQUEST_WAIT_MS=2000 P3_AD_RETRY_UNCERTAIN=1 P3_SIM_LANE_EDGE_STRICT=0
export P3_ENABLE_CROSSWALK=1 P3_ENABLE_CROSSWALK_PREDICTION=1
export P3_CROSSWALK_CSV="$OUT/crosswalk.csv"
# Keep masking paired with the current hardware OSD bitstream.
P3_NOOSD_PROOF=/run/codex_p3_noosd_a98fedea.verified
if [ -r "$P3_NOOSD_PROOF" ] && [ "$(cat "$P3_NOOSD_PROOF")" = "$(cat /proc/sys/kernel/random/boot_id) a98fedea197cad365acd18af242dbe58bbcc1f082336b04b2239b8ad54e6e962" ]; then
  export P3_HW_OSD=0
else
  export P3_HW_OSD=1
fi
export P3_ATLAS_V4_SHADOW=1 P3_ATLAS_V4_DECISION=1
export P3_ATLAS_V4_MODEL=/userdata/atlas_v4_demo_20261010/models/attributes_atlas_v4_128_fp16.rknn
export P3_ATLAS_V4_DRAW=0  # [LANE-GATE 2026-10-22] hide Atlas panel only
export P3_ATLAS_V4_CSV="$OUT/attributes.csv"
export P3_LIGHT_INTERVAL=2 P3_LIGHT_TRACK=1 P3_LIGHT_TRACK_HOLD=2
export P3_LIGHT_CONF=.20 P3_LIGHT_VERIFY=1 P3_LIGHT_VERIFY_CONF=.50 P3_LIGHT_VERIFY_TOPK=2
export P3_SIGN_INTERVAL=20 P3_OBJECT_INTERVAL=1 P3_OBJECT_EMA=0 P3_OBJECT_TRACK_HOLD=2
export P3_ENABLE_SIGN=0 P3_LANE_INTERVAL=2 P3_ENABLE_UFLD=0 P3_ENABLE_TWINLITE=1 P3_ENABLE_SPEED=0
export P3_FPGA_DECISION=1 P3_LANE_UPGRADE=1 P3_LANE_UPGRADE_DRAW=1 P3_LANE_UPGRADE_MAX_ROWS=200000
export P3_LANE_UPGRADE_CSV="$OUT/upgrade.csv"
export P3_DISPLAY_THREAD=1 P3_DECISION_OSD=1 P3_QT_MAIN_ACTOR=1
export P3_LIGHT_ZOOM=1
SHA_ARGS=(--ai-mode 3 --fpga-decision --warmup 5
          --raw-csv "$OUT/experiment_frames.csv" --perf-frames "$OUT/perf_frames.csv"
          --summary-csv "$OUT/summary.csv" --perf-summary "$OUT/perf_summary.csv"
          --run-config-json "$OUT/config.json")
if [ "$MODE" = smoke ]; then
  set -- --frames 180 "${SHA_ARGS[@]}"
  python3 "$V9/bounded_run.py" "$OUT" "$APP" "$@" 9>&-
  echo "SMOKE_RC=$?"
else
  nohup setsid python3 "$V9/bounded_run.py" "$OUT" "$APP" "${SHA_ARGS[@]}" \
      >"$OUT/supervisor.log" 2>&1 </dev/null 9>&- &
  sleep 2
  PID=$(cat "$OUT/app.pid")
  [ -d "/proc/$PID" ] || { echo 'REFUSE: app not alive' >&2; exit 12; }
  [ "$(readlink -f "/proc/$PID/exe")" = "$(readlink -f "$APP")" ] || { echo 'REFUSE: exe mismatch' >&2; exit 13; }
  echo "QT ASYNC DEFAULT running pid=$PID run=$OUT"
fi
