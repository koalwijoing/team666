#!/bin/bash
export KEYINJECT_PID=/userdata/claude_keyinject_20261021/keyinject.pid
export KEYINJECT_LOG=/userdata/claude_keyinject_20261021/keyinject.log
export KEYINJECT_LOCK=/userdata/claude_keyinject_20261021/keyinject.lock
exec bash /userdata/claude_keyinject_20261021/run_keyinject.sh "$1"
