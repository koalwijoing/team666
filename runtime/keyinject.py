#!/usr/bin/env python3
"""USB keyboard -> P3 KEY_INJECT bridge (READ keyboard, WRITE BAR0 0x20C).

Mapping, press only, autorepeat ignored. Never bind Q or ESC here: the
application quits on q/Q/ESC (main.cpp:1695) and would stop on a keypress.
    W -> START_STOP     0x02
    S -> LANE_CYCLE     0x01
    A -> TURN_LEFT      0x04
    D -> TURN_RIGHT     0x08
    F -> FORCE_DRIVE    0x10

Injected bits land in P3_KEY_EVENT (0x208) through the same W1C-consumed
register the physical-key path uses, so the existing application needs no
change. Nothing else in the decision ABI is touched: CTRL/LED/ALARM are
never written by this tool.
"""
import argparse
import mmap
import os
import select
import struct
import sys
import time

EV_KEY = 1
EVENT = struct.Struct("<qqHHi")   # aarch64 input_event, 24 bytes
REG_CTRL = 0x200
REG_STATUS = 0x204
REG_EVENT = 0x208
REG_INJECT = 0x20C
PANG_OFF = 0x1F4
PANG_SIG = 0x50414E47

MAPPING = {
    17: ("W", 0x02, "START_STOP"),
    31: ("S", 0x01, "LANE_CYCLE"),
    30: ("A", 0x04, "TURN_LEFT"),
    32: ("D", 0x08, "TURN_RIGHT"),
    33: ("F", 0x10, "FORCE_DRIVE"),
}


def log(msg):
    print(msg, flush=True)


def find_keyboard():
    try:
        raw = open("/proc/bus/input/devices", encoding="utf-8", errors="replace").read()
    except OSError:
        return None, None
    for block in raw.split("\n\n"):
        if "kbd" not in block:
            continue
        name = ""
        handler = None
        for line in block.splitlines():
            if line.startswith("N: Name="):
                name = line.split("=", 1)[1].strip().strip('"')
            elif line.startswith("H: Handlers="):
                for tok in line.split("=", 1)[1].split():
                    if tok.startswith("event"):
                        handler = tok
        low = name.lower()
        if handler and "keyboard" in low and "control" not in low and "system" not in low:
            return "/dev/input/" + handler, name
    return None, None


def open_bar(res):
    bar0 = int(open(res, encoding="ascii").readline().split()[0], 16)
    fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
    mm = mmap.mmap(fd, 8192, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=bar0)
    os.close(fd)
    return mm


def wait_for_device(path, seconds):
    end = time.time() + max(0.0, seconds)
    while time.time() < end:
        if os.path.exists(path):
            return True
        time.sleep(0.2)
    return os.path.exists(path)


def rd32(mm, off):
    return struct.unpack_from("<I", mm, off)[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default=None, help="evdev path; default auto-detect")
    ap.add_argument("--resource", default="/sys/bus/pci/devices/0002:21:00.0/resource")
    ap.add_argument("--dry-run", action="store_true", help="decode keys, never write FPGA")
    ap.add_argument("--duration", type=float, default=0.0, help="seconds; 0 = until stopped")
    ap.add_argument("--wait-device", type=float, default=8.0,
                    help="seconds to wait for the evdev node to appear (udev race)")
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()

    dev, dev_name = (args.device, "(explicit)") if args.device else find_keyboard()
    if args.list:
        log("device: %s (%s)" % (dev, dev_name))
        for code, (k, bit, sem) in sorted(MAPPING.items(), key=lambda kv: kv[1][0]):
            log("  %s -> 0x%02X %s" % (k, bit, sem))
        return 0
    if not dev:
        log("ERROR: no keyboard evdev found (plug a USB keyboard, or pass --device)")
        return 2
    if not wait_for_device(dev, args.wait_device):
        log("ERROR: evdev %s did not appear within %.1fs" % (dev, args.wait_device))
        return 2

    mm = None
    if not args.dry_run:
        mm = open_bar(args.resource)
        sig = rd32(mm, PANG_OFF)
        if sig != PANG_SIG:
            log("ERROR: BAR0 signature 0x%08X != 0x%08X (FPGA not decoded; refuse to write)" % (sig, PANG_SIG))
            return 3
        log("BAR0 OK: PANG 0x%08X, CTRL 0x%08X, STATUS 0x%08X, EVENT 0x%08X" % (
            sig, rd32(mm, REG_CTRL), rd32(mm, REG_STATUS), rd32(mm, REG_EVENT)))
    else:
        log("DRY RUN: FPGA will not be written")

    log("listening on %s (%s)%s" % (dev, dev_name, "" if args.dry_run else " -> KEY_INJECT 0x20C"))
    fd = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)
    t0 = time.time()
    presses = 0
    try:
        while True:
            if args.duration and (time.time() - t0) >= args.duration:
                log("duration reached")
                break
            r, _, _ = select.select([fd], [], [], 0.5)
            if not r:
                continue
            try:
                data = os.read(fd, EVENT.size * 64)
            except BlockingIOError:
                continue
            except OSError as exc:
                if exc.errno == 19:  # ENODEV: keyboard unplugged
                    log("device %s disappeared (ENODEV); exiting cleanly" % dev)
                    break
                raise
            for i in range(0, len(data) - EVENT.size + 1, EVENT.size):
                _s, _us, etype, code, value = EVENT.unpack_from(data, i)
                if etype != EV_KEY or value != 1:
                    continue
                hit = MAPPING.get(code)
                if not hit:
                    continue
                key, bit, sem = hit
                presses += 1
                if mm is not None:
                    struct.pack_into("<I", mm, REG_INJECT, bit)
                    ev = rd32(mm, REG_EVENT)
                    log("[%7.2fs] %s -> %s 0x%02X  EVENT=0x%08X" % (time.time() - t0, key, sem, bit, ev))
                else:
                    log("[%7.2fs] %s -> %s 0x%02X  (dry-run)" % (time.time() - t0, key, sem, bit))
    except KeyboardInterrupt:
        log("interrupted")
    finally:
        os.close(fd)
    log("stopped after %d mapped presses" % presses)
    return 0


if __name__ == "__main__":
    sys.exit(main())
