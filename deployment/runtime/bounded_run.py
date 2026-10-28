#!/usr/bin/env python3
"""Bounded userdata console logs; forward termination to this exact child only."""
import os
import signal
import subprocess
import sys
from pathlib import Path

out = Path(sys.argv[1])
out.mkdir(parents=True, exist_ok=True)
child = subprocess.Popen(sys.argv[2:], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         stdin=subprocess.DEVNULL)
(out / 'app.pid').write_text(str(child.pid))
def stop(signum, frame):
    if child.poll() is None:
        child.send_signal(signal.SIGTERM)
signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGINT, stop)
log = out / 'app.log'
stream = log.open('wb')
size = 0
try:
    for line in iter(child.stdout.readline, b''):
        if size + len(line) > 4 * 1024 * 1024:
            stream.close()
            for index in (2, 1):
                src = out / f'app.log.{index}'
                if src.exists():
                    os.replace(src, out / f'app.log.{index + 1}')
            os.replace(log, out / 'app.log.1')
            stream = log.open('wb')
            size = 0
        stream.write(line)
        stream.flush()
        size += len(line)
finally:
    stream.close()
sys.exit(child.wait())
