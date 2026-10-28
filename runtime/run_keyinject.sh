#!/bin/bash
# Start/stop the USB-keyboard -> KEY_INJECT bridge.
#
# Identity of a running bridge is proven by: PID alive + interpreter is python +
# the FIRST NON-OPTION ARGUMENT of the NUL-separated argv is exactly this
# package script. A cmdline substring is NOT accepted (an impostor that merely
# carries the path as a normal argument is rejected). start and stop share one
# flock so they cannot interleave.
set -u
DIR="$(cd "$(dirname "$0")" && pwd)"
SCRIPT="$DIR/keyinject.py"
SCRIPT_REAL="$(readlink -f "$SCRIPT" 2>/dev/null || printf %s "$SCRIPT")"
LOG="${KEYINJECT_LOG:-/userdata/keyinject_20261019.log}"
PIDFILE="${KEYINJECT_PID:-/userdata/keyinject_20261019.pid}"
LOCKFILE="${KEYINJECT_LOCK:-/userdata/keyinject_20261019.lock}"
LOCK_WAIT="${KEYINJECT_LOCK_WAIT:-5}"    # seconds; start and stop both wait on it
STOP_WAIT="${KEYINJECT_STOP_WAIT:-50}"   # 50 x 0.1s = 5s bounded TERM wait

is_python() {
  case "$(readlink -f "/proc/$1/exe" 2>/dev/null)" in
    */python3|*/python3.*|*/python) return 0 ;;
    *) return 1 ;;
  esac
}

# argv identity: the first non-option token after argv[0] must BE the script.
is_ours() {
  local pid="${1:-}"
  [ -n "$pid" ] && [ -d "/proc/$pid" ] || return 1
  is_python "$pid" || return 1
  local -a argv=()
  local tok
  while IFS= read -r -d "" tok; do argv+=("$tok"); done < "/proc/$pid/cmdline" 2>/dev/null || return 1
  [ "${#argv[@]}" -ge 2 ] || return 1
  local i cand=""
  for ((i = 1; i < ${#argv[@]}; i++)); do
    case "${argv[$i]}" in
      -c|-m|-) return 1 ;;   # code / module / stdin: never our daemon
      -*) continue ;;        # benign flags such as -u or -B
      *) cand="${argv[$i]}"; break ;;
    esac
  done
  [ -n "$cand" ] || return 1
  [ "$cand" = "$SCRIPT" ] && return 0
  [ "$(readlink -f "$cand" 2>/dev/null)" = "$SCRIPT_REAL" ] && return 0
  return 1
}

read_pid() { if [ -s "$PIDFILE" ]; then cat "$PIDFILE"; else echo ""; fi; }

cmd_status() {
  local pid; pid="$(read_pid)"
  if is_ours "$pid"; then echo "running pid $pid"; [ -f "$LOG" ] && tail -3 "$LOG"; return 0; fi
  if [ -n "$pid" ]; then echo "stale pidfile (pid $pid is not our keyinject)"; else echo "not running"; fi
  return 1
}

cmd_stop() {
  # same lock as start: bounded wait, then refuse rather than race
  exec 8>"$LOCKFILE" || { echo "cannot open lock $LOCKFILE"; return 1; }
  if ! flock -w "$LOCK_WAIT" 8; then echo "lock busy after ${LOCK_WAIT}s; not stopping"; return 1; fi
  local pid; pid="$(read_pid)"
  if [ -z "$pid" ]; then echo "not running"; rm -f "$PIDFILE"; return 0; fi
  if ! is_ours "$pid"; then
    echo "pid $pid is not our keyinject; refusing to signal it"
    rm -f "$PIDFILE"
    return 0
  fi
  kill -TERM "$pid" 2>/dev/null || true
  local i
  for i in $(seq 1 "$STOP_WAIT"); do
    kill -0 "$pid" 2>/dev/null || break
    sleep 0.1
  done
  if kill -0 "$pid" 2>/dev/null; then
    echo "pid $pid still alive after bounded wait; not forcing"
    return 1
  fi
  rm -f "$PIDFILE"
  echo "stopped $pid"
  return 0
}

cmd_start() {
  exec 9>"$LOCKFILE" || { echo "cannot open lock $LOCKFILE"; return 1; }
  if ! flock -w "$LOCK_WAIT" 9; then echo "lock busy after ${LOCK_WAIT}s; not starting"; return 1; fi
  local pid; pid="$(read_pid)"
  if is_ours "$pid"; then echo "already running pid $pid"; return 0; fi
  rm -f "$PIDFILE"
  # 9>&- : never leak the lock into the daemon, or the lock stays held by the
  # child and a later start/stop could never acquire it.
  nohup python3 "$SCRIPT" "$@" 9>&- >>"$LOG" 2>&1 &
  pid=$!
  echo "$pid" > "$PIDFILE"
  sleep 1
  if ! is_ours "$pid"; then
    echo "start FAILED: pid $pid is not alive or is not this script"
    tail -5 "$LOG" 2>/dev/null
    rm -f "$PIDFILE"
    return 1
  fi
  echo "started pid $pid; log $LOG"
  tail -3 "$LOG" 2>/dev/null
  return 0
}

case "${1:-start}" in
  start) shift || true; cmd_start "$@" ;;
  stop) cmd_stop ;;
  status) cmd_status ;;
  dry) shift || true; exec python3 "$SCRIPT" --dry-run "$@" ;;
  *) echo "usage: $0 [start [args] | stop | status | dry]"; exit 2 ;;
esac
