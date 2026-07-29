#!/usr/bin/env python3
"""Listen on the CAN bus and live-display one or more fingertip sensors.

A listen-only sibling of motor_hold.py: it drives nothing, it just decodes the
fingertip frames (4 pressure messages = 8 channels, plus 3 ToF ranges per
sensor) and reprints a refreshing terminal dashboard.

Configure everything by editing the constants below (CHANNEL, SENSOR_IDS) --
there are no command-line flags.

Linux setup (run once; matches the firmware's FD-BRS timing):
    sudo ip link set can0 down
    sudo ip link set can0 up type can bitrate 1000000 dbitrate 2000000 fd on

Usage:
    python3 fingertip_display.py
"""

import socket
import struct
import time

# ---------------------------------------------------------------------------
# Configuration  (edit these constants instead of passing command-line flags)
# ---------------------------------------------------------------------------
CHANNEL = "can0"          # SocketCAN interface

# Fingertip sensor ids to display. This is the firmware's SELECTED_FINGER value
# (a per-board sensor id, NOT a finger number); it selects the CAN id block
# base = (sensor_id - 1) * 4. List every sensor plugged into the bus.
SENSOR_IDS = [4]

REFRESH_HZ = 20           # screen redraw rate (Hz)

# ---------------------------------------------------------------------------
# Linux SocketCAN / CAN-FD constants
# ---------------------------------------------------------------------------
SOL_CAN_RAW = 101            # socket.SOL_CAN_RAW
CAN_RAW_FD_FRAMES = 5        # socket.CAN_RAW_FD_FRAMES
CAN_SFF_MASK = 0x000007FF    # 11-bit standard ID mask

FDCAN_FRAME_FMT = "=IBB2x64s"
FDCAN_FRAME_SIZE = 72


# ---------------------------------------------------------------------------
# Fingertip decoding (mirrors can_unpack.py / fingertip.cpp)
# ---------------------------------------------------------------------------
def build_sensor_id_map(sensor_id):
    """CAN id -> (name, kind, channels) for one fingertip sensor board.

    Mirrors fingertip.cpp: base = (SELECTED_FINGER - 1) * 4, then PR_1..PR_4,
    PR_TOF, PR_IMU at base+0..base+5.
    """
    base = (sensor_id - 1) * 4
    return {
        base + 0: ("PR_1", "pressure", (0, 1)),
        base + 1: ("PR_2", "pressure", (2, 3)),
        base + 2: ("PR_3", "pressure", (4, 5)),
        base + 3: ("PR_4", "pressure", (6, 7)),
        base + 4: ("PR_TOF", "tof", None),
        base + 5: ("PR_IMU", "imu", None),
    }


def build_combined_map(sensor_ids):
    """Merge several sensors' id maps -> {can_id: (sensor_id, name, kind, channels)}.

    Also returns id collisions between transmitted (non-IMU) messages. The
    firmware spaces sensor blocks only 4 ids apart but emits 6 ids per sensor, so
    adjacent sensor ids overlap (e.g. sensor 3's PR_TOF == sensor 4's PR_1).
    """
    combined = {}
    owners = {}  # can_id -> [(sensor_id, name), ...] for transmitted messages
    for sid in sensor_ids:
        for cid, (name, kind, channels) in build_sensor_id_map(sid).items():
            combined[cid] = (sid, name, kind, channels)
            if kind != "imu":  # IMU is declared but not transmitted by firmware
                owners.setdefault(cid, []).append((sid, name))
    collisions = {cid: o for cid, o in owners.items() if len(o) > 1}
    return combined, collisions


def decode_pressure(data, channels):
    """Two signed big-endian int32 -> {channel_index: value}."""
    if len(data) < 8:
        return {}
    a, b = struct.unpack(">ii", bytes(data[:8]))
    return {channels[0]: a, channels[1]: b}


def decode_tof(data):
    """range[0..2] packed as single bytes (uint8, truncated by firmware)."""
    return {i: data[i] for i in range(min(3, len(data)))}


# ---------------------------------------------------------------------------
# Bus wrapper (listen-only)
# ---------------------------------------------------------------------------
class Bus:
    def __init__(self, channel):
        try:
            self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.sock.bind((channel,))
            self.sock.setsockopt(SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1)
            self.sock.setblocking(False)
            try:
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
            except OSError:
                pass
            print(f"[Success] Bound to {channel} (CAN-FD)")
        except Exception as e:
            print(f"[Error] Could not bind to {channel}: {e}")
            print("Did you run:")
            print(f"  sudo ip link set {channel} up type can bitrate 1000000 dbitrate 2000000 fd on")
            raise SystemExit(1)

    def recv(self):
        """Non-blocking single frame -> (can_id, dlc, payload) or None."""
        try:
            frame = self.sock.recv(FDCAN_FRAME_SIZE)
        except BlockingIOError:
            return None
        can_id, dlc, _flags, data = struct.unpack(FDCAN_FRAME_FMT, frame)
        return can_id & CAN_SFF_MASK, dlc, data[:dlc]


def render(sensor_ids, pressure, tof, rates, first):
    """Reprint the dashboard in place using ANSI cursor moves."""
    # 3 header lines + per sensor: 1 title + 2 pressure + 1 tof + 1 blank
    nlines = 3 + len(sensor_ids) * 5
    if not first:
        print(f"\x1b[{nlines}A", end="")  # move cursor up to overwrite
    clr = "\x1b[K"  # clear to end of line

    lines = []
    lines.append(f"Fingertip sensors {sensor_ids} on {CHANNEL}   (Ctrl-C to stop)")
    lines.append("-" * 60)
    lines.append("")
    for sid in sensor_ids:
        p = pressure[sid]
        t = tof[sid]
        hz = rates.get(sid, 0.0)
        lines.append(f"Sensor {sid}   ({hz:5.0f} Hz pressure updates)")
        lines.append("  pressure ch0-3: " + " ".join(_fmt(p[i]) for i in range(4)))
        lines.append("  pressure ch4-7: " + " ".join(_fmt(p[i]) for i in range(4, 8)))
        lines.append("  tof (mm)      : " + " ".join(_fmt(t[i], width=6) for i in range(3)))
        lines.append("")
    print("\n".join(line + clr for line in lines))


def _fmt(v, width=10):
    return f"{'--':>{width}}" if v is None else f"{v:>{width}}"


def main():
    sensor_ids = list(SENSOR_IDS)
    combined_map, collisions = build_combined_map(sensor_ids)
    bus = Bus(CHANNEL)

    for sid in sensor_ids:
        ids = build_sensor_id_map(sid)
        print(f"Sensor {sid} IDs: "
              + ", ".join(f"{name}=0x{cid:X}({cid})" for cid, (name, *_) in ids.items()))
    if collisions:
        print("\n[WARNING] CAN id collisions between sensors (firmware uses a stride of 4")
        print("          but 6 ids per sensor, so adjacent sensor ids overlap):")
        for cid, owners in sorted(collisions.items()):
            who = ", ".join(f"sensor {s} {n}" for s, n in owners)
            print(f"          id {cid} (0x{cid:X}): {who}")
        print()

    pressure = {s: [None] * 8 for s in sensor_ids}
    tof = {s: [None] * 3 for s in sensor_ids}
    press_msgs = {s: 0 for s in sensor_ids}   # PR_* frames since last redraw
    rates = {s: 0.0 for s in sensor_ids}

    redraw_period = 1.0 / REFRESH_HZ
    last_draw = time.perf_counter()
    first = True

    try:
        while True:
            # drain everything currently buffered
            while True:
                got = bus.recv()
                if got is None:
                    break
                can_id, _dlc, payload = got
                entry = combined_map.get(can_id)
                if entry is None:
                    continue
                sid, _name, kind, channels = entry
                if kind == "pressure":
                    for idx, val in decode_pressure(payload, channels).items():
                        pressure[sid][idx] = val
                    press_msgs[sid] += 1
                elif kind == "tof":
                    for idx, val in decode_tof(payload).items():
                        tof[sid][idx] = val

            now = time.perf_counter()
            if now - last_draw >= redraw_period:
                elapsed = now - last_draw
                for s in sensor_ids:
                    rates[s] = press_msgs[s] / elapsed
                    press_msgs[s] = 0
                render(sensor_ids, pressure, tof, rates, first)
                first = False
                last_draw = now
            else:
                time.sleep(0.001)

    except KeyboardInterrupt:
        print()
    finally:
        bus.sock.close()


if __name__ == "__main__":
    main()
