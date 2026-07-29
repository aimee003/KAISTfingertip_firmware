#!/usr/bin/env python3
"""Sine-drive motor(s) and live-plot the motor state + fingertip sensors.

Same bus / protocol as motor_and_fingertip_test.py, but instead of printing a
status line every second it shows a live matplotlib dashboard:

  * top    : motor position -- commanded p_des vs. actual p (rad), one line/motor
  * middle : 8 fingertip pressure channels per sensor
  * bottom : 3 fingertip ToF ranges (mm) per sensor

The CAN send/recv loop runs in a background thread so its 2 kHz pacing is not
disturbed by the GUI; the main thread just redraws a rolling time window.

What's on the bus is configured by the constants below (MOTOR_IDS /
FINGERTIP_SENSOR_IDS). Note these are *ids*, not finger/joint numbers: the
firmware's SELECTED_FINGER is really a per-board sensor id, and the motor CAN
ids likewise don't correspond to a finger number.

Linux setup (run once; matches the firmware's FD-BRS timing):
    sudo ip link set can0 down
    sudo ip link set can0 up type can bitrate 1000000 dbitrate 2000000 fd on

Usage:
    python3 motor_and_fingertip_plot.py                      # uses constants below
    python3 motor_and_fingertip_plot.py --motors 3,4 --sensors 3,4 --channel can0
    python3 motor_and_fingertip_plot.py --no-motor           # plot fingertips only
    python3 motor_and_fingertip_plot.py --window 10

    sudo ip link set can0 up type can bitrate 1000000 dbitrate 2000000 fd on restart-ms 100
"""

import argparse
import math
import socket
import struct
import threading
import time
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ---------------------------------------------------------------------------
# Hardware on the bus  (these are *ids*, not finger/joint numbers)
# ---------------------------------------------------------------------------
# Motor CAN ids currently plugged into the bus. Each is driven with the same
# sine command and its state reply is plotted on its own line.
MOTOR_IDS = [3, 4]

# Fingertip sensor ids currently plugged into the bus. This is the firmware's
# SELECTED_FINGER value (a per-board sensor id, NOT a finger number); it selects
# the CAN id block base = (sensor_id - 1) * 4. Add 3 here once fingertip sensor
# #3 is plugged in -> [3, 4].
FINGERTIP_SENSOR_IDS = [4]

# ---------------------------------------------------------------------------
# MIT-mode motor scaling (must match the motor firmware / motor_test_fdcan_sine.py)
# ---------------------------------------------------------------------------
GR = 30.0
P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -840.0 / GR, 840.0 / GR
KP_MIN, KP_MAX = 0.0, 2.0
KD_MIN, KD_MAX = 0.0, 0.01
T_MIN, T_MAX = -0.15 * GR, 0.15 * GR
I_MAX = 4.5

# Sine trajectory defaults
TARGET_HZ = 2000
SINE_AMP = -1        # rad, peak amplitude; centered sine swings 0 -> amp -> -amp
SINE_FREQ = 5      # rad/s
KP = 1.0
KD = 0.01
T_FF = 0.0

# ---------------------------------------------------------------------------
# Linux SocketCAN / CAN-FD constants
# ---------------------------------------------------------------------------
SOL_CAN_RAW = 101            # socket.SOL_CAN_RAW
CAN_RAW_FD_FRAMES = 5        # socket.CAN_RAW_FD_FRAMES
CANFD_BRS = 0x01             # bit-rate switch flag
CAN_SFF_MASK = 0x000007FF    # 11-bit standard ID mask

FDCAN_FRAME_FMT = "=IBB2x64s"
FDCAN_FRAME_SIZE = 72


# ---------------------------------------------------------------------------
# Float <-> uint helpers (MIT protocol)
# ---------------------------------------------------------------------------
def _float_to_uint(x, x_min, x_max, bits):
    span = x_max - x_min
    x = max(min(x, x_max), x_min)
    return int((x - x_min) * ((2 ** bits - 1) / span))


def _uint_to_float(x_int, x_min, x_max, bits):
    span = x_max - x_min
    return x_int * (span / (2 ** bits - 1)) + x_min


def pack_command(p_des, v_des, kp, kd, t_ff):
    p_int = _float_to_uint(p_des, P_MIN, P_MAX, 16)
    v_int = _float_to_uint(v_des, V_MIN, V_MAX, 12)
    kp_int = _float_to_uint(kp, KP_MIN, KP_MAX, 12)
    kd_int = _float_to_uint(kd, KD_MIN, KD_MAX, 12)
    t_int = _float_to_uint(t_ff, T_MIN, T_MAX, 12)

    b = bytearray(8)
    b[0] = (p_int >> 8) & 0xFF
    b[1] = p_int & 0xFF
    b[2] = (v_int >> 4) & 0xFF
    b[3] = ((v_int << 4) & 0xF0) | ((kp_int >> 8) & 0x0F)
    b[4] = kp_int & 0xFF
    b[5] = (kd_int >> 4) & 0xFF
    b[6] = ((kd_int << 4) & 0xF0) | ((t_int >> 8) & 0x0F)
    b[7] = t_int & 0xFF
    return bytes(b)


def unpack_reply(data):
    """Decode a MIT-mode state reply -> (motor_id, p, v, iq, t_des)."""
    motor_id = data[0]
    p_int = (data[1] << 8) | data[2]
    v_int = (data[3] << 4) | ((data[4] >> 4) & 0x0F)
    iq_int = ((data[4] & 0x0F) << 8) | data[5]
    t_des_int = (data[6] << 4) | (data[7] >> 4)

    p = _uint_to_float(p_int, P_MIN, P_MAX, 16)
    v = _uint_to_float(v_int, V_MIN, V_MAX, 12)
    iq = _uint_to_float(iq_int, -I_MAX, I_MAX, 12)
    t_des = _uint_to_float(t_des_int, T_MIN, T_MAX, 12)
    return motor_id, p, v, iq, t_des


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
    if len(data) < 8:
        return {}
    a, b = struct.unpack(">ii", bytes(data[:8]))
    return {channels[0]: a, channels[1]: b}


def decode_tof(data):
    return {i: data[i] for i in range(min(3, len(data)))}


# ---------------------------------------------------------------------------
# Bus wrapper
# ---------------------------------------------------------------------------
class BusOff(Exception):
    """Raised when the CAN controller goes bus-off (interface ENETDOWN)."""


class Bus:
    def __init__(self, channel):
        try:
            self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.sock.bind((channel,))
            self.sock.setsockopt(SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1)
            self.sock.setblocking(False)
            try:
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
            except OSError:
                pass
            print(f"[Success] Bound to {channel} (CAN-FD)")
        except Exception as e:
            print(f"[Error] Could not bind to {channel}: {e}")
            print("Did you run:")
            print(f"  sudo ip link set {channel} up type can bitrate 1000000 dbitrate 2000000 fd on")
            raise SystemExit(1)

    def _frame(self, can_id, data):
        padded = data + b"\x00" * (64 - len(data))
        return struct.pack(FDCAN_FRAME_FMT, can_id, len(data), CANFD_BRS, padded)

    def send_to_motor(self, motor_id, data):
        try:
            self.sock.send(self._frame(motor_id, data))
            return True
        except BlockingIOError:
            return False
        except OSError as e:
            if e.errno == 105:  # ENETDOWN -> controller went bus-off
                raise BusOff()
            raise

    def enable_motor(self, motor_id):
        print(f"Enabling motor {motor_id}...")
        self.send_to_motor(motor_id, b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFC")
        time.sleep(0.1)

    def disable_motor(self, motor_id):
        print(f"Disabling motor {motor_id}...")
        self.send_to_motor(motor_id, b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFD")

    def zero_encoder(self, motor_id):
        print(f"Zeroing encoder of motor {motor_id}...")
        self.send_to_motor(motor_id, b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFE")
        time.sleep(0.1)

    def recv(self):
        try:
            frame = self.sock.recv(FDCAN_FRAME_SIZE)
        except BlockingIOError:
            return None
        can_id, dlc, _flags, data = struct.unpack(FDCAN_FRAME_FMT, frame)
        return can_id & CAN_SFF_MASK, dlc, data[:dlc]


# ---------------------------------------------------------------------------
# Shared state filled by the CAN thread, read by the plot thread
# ---------------------------------------------------------------------------
class Shared:
    def __init__(self, window, motor_ids, sensor_ids):
        self.lock = threading.Lock()
        self.window = window
        self.motor_ids = list(motor_ids)
        self.sensor_ids = list(sensor_ids)
        # rolling (t, value) buffers
        self.p_des = deque()                                       # shared sine cmd
        self.p_act = {m: deque() for m in self.motor_ids}         # per motor
        self.pressure = {s: [deque() for _ in range(8)] for s in self.sensor_ids}
        self.tof = {s: [deque() for _ in range(3)] for s in self.sensor_ids}
        self.stop = False

    def _all_deques(self):
        yield self.p_des
        for dq in self.p_act.values():
            yield dq
        for s in self.sensor_ids:
            yield from self.pressure[s]
            yield from self.tof[s]

    def _trim(self, now):
        cutoff = now - self.window
        for dq in self._all_deques():
            while dq and dq[0][0] < cutoff:
                dq.popleft()


def can_loop(bus, shared, combined_map, motor_ids, sensor_ids, rate, drive_motor):
    period = (1.0 / rate) if rate > 0 else 0.0

    motor_id_set = set(motor_ids)
    latest_p_des = 0.0
    latest_p_act = {m: float("nan") for m in motor_ids}
    latest_press = {s: [float("nan")] * 8 for s in sensor_ids}
    latest_tof = {s: [float("nan")] * 3 for s in sensor_ids}

    if drive_motor:
        for m in motor_ids:
            bus.zero_encoder(m)
            bus.enable_motor(m)

    start = time.perf_counter()
    next_deadline = start
    last_sample = start
    sample_period = 1.0 / 200.0  # log to plot buffers at 200 Hz, not 2 kHz

    try:
        while not shared.stop:
            if period > 0.0:
                # sleep-based pacing (releases the GIL so the GUI thread can
                # redraw); a busy-wait here would starve matplotlib and freeze
                # the window.
                sleep_t = next_deadline - time.perf_counter()
                if sleep_t > 0:
                    time.sleep(sleep_t)
                next_deadline += period
                # If a GUI stall left us behind, resync instead of bursting a
                # catch-up flood that overruns the bus and triggers bus-off.
                behind = time.perf_counter()
                if next_deadline < behind:
                    next_deadline = behind

            t = time.perf_counter() - start

            if drive_motor:
                # centered sine: starts at midpoint 0, swings 0 -> +amp -> -amp
                latest_p_des = SINE_AMP * math.sin(SINE_FREQ * t)
                v_des = SINE_AMP * SINE_FREQ * math.cos(SINE_FREQ * t)
                cmd = pack_command(latest_p_des, v_des, KP, KD, T_FF)
                for m in motor_ids:
                    bus.send_to_motor(m, cmd)

            # drain RX
            while True:
                got = bus.recv()
                if got is None:
                    break
                can_id, dlc, payload = got
                entry = combined_map.get(can_id)
                if entry is not None:
                    sid, _name, kind, channels = entry
                    if kind == "pressure":
                        for idx, val in decode_pressure(payload, channels).items():
                            latest_press[sid][idx] = val
                    elif kind == "tof":
                        for idx, val in decode_tof(payload).items():
                            latest_tof[sid][idx] = val
                elif len(payload) >= 8 and payload[0] in motor_id_set:
                    mid, p, _v, _iq, _tq = unpack_reply(payload)
                    latest_p_act[mid] = p

            # downsample into the shared plot buffers
            if t - (last_sample - start) >= sample_period:
                last_sample = time.perf_counter()
                with shared.lock:
                    shared.p_des.append((t, latest_p_des))
                    for m in motor_ids:
                        shared.p_act[m].append((t, latest_p_act[m]))
                    for s in sensor_ids:
                        for i in range(8):
                            shared.pressure[s][i].append((t, latest_press[s][i]))
                        for i in range(3):
                            shared.tof[s][i].append((t, latest_tof[s][i]))
                    shared._trim(t)
    except BusOff:
        shared.stop = True
        print("\n[bus-off] CAN controller went down (TX errors -> bus-off).")
        print("  Recover and enable auto-restart so this self-heals:")
        print("    sudo ip link set can0 down")
        print("    sudo ip link set can0 up type can bitrate 1000000 dbitrate 2000000 "
              "fd on restart-ms 100")
        print("  Also check motor power/wiring and bus termination (120 ohm each end).")
    finally:
        if drive_motor:
            for m in motor_ids:
                try:
                    bus.disable_motor(m)
                except BusOff:
                    pass


def _xy(dq):
    if not dq:
        return [], []
    xs = [pt[0] for pt in dq]
    ys = [pt[1] for pt in dq]
    return xs, ys


def _parse_id_list(text, default):
    if text is None:
        return list(default)
    return [int(x) for x in text.replace(",", " ").split()]


def main():
    ap = argparse.ArgumentParser(description="Sine-drive motor(s) and live-plot the fingertips")
    ap.add_argument("--channel", default="can0", help="SocketCAN interface (default can0)")
    ap.add_argument("--motors", default=None,
                    help=f"Comma-separated motor CAN ids (default {MOTOR_IDS})")
    ap.add_argument("--sensors", default=None,
                    help="Comma-separated fingertip sensor ids / SELECTED_FINGER values "
                         f"(default {FINGERTIP_SENSOR_IDS})")
    ap.add_argument("--rate", type=int, default=1000,
                    help="Command send rate Hz (default 1000; lower than the 2 kHz "
                         "headless tool to keep the GUI responsive and the bus calm)")
    ap.add_argument("--window", type=float, default=5.0, help="Rolling plot window in seconds (default 5)")
    ap.add_argument("--no-motor", action="store_true", help="Plot fingertips only; do not drive the motors")
    args = ap.parse_args()

    motor_ids = _parse_id_list(args.motors, MOTOR_IDS)
    sensor_ids = _parse_id_list(args.sensors, FINGERTIP_SENSOR_IDS)

    combined_map, collisions = build_combined_map(sensor_ids)
    bus = Bus(args.channel)

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
        print("          These frames cannot be told apart on the bus -- space sensor")
        print("          ids >=6 apart, or change the firmware id stride.\n")

    shared = Shared(args.window, motor_ids, sensor_ids)
    drive = not args.no_motor
    worker = threading.Thread(
        target=can_loop,
        args=(bus, shared, combined_map, motor_ids, sensor_ids, args.rate, drive),
        daemon=True,
    )
    worker.start()

    # ---- figure ----
    fig, (ax_m, ax_p, ax_t) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle(f"Motors {motor_ids} + Fingertip sensors {sensor_ids} on {args.channel}")

    line_des, = ax_m.plot([], [], label="p_des (cmd)", lw=1.5, color="k")
    act_lines = {m: ax_m.plot([], [], lw=1.0, label=f"motor {m} p")[0] for m in motor_ids}
    ax_m.set_ylabel("position (rad)")
    ax_m.legend(loc="upper right", fontsize=8)
    ax_m.grid(True, alpha=0.3)

    press_lines = {s: [ax_p.plot([], [], lw=1.0, label=f"s{s} ch{i}")[0] for i in range(8)]
                   for s in sensor_ids}
    ax_p.set_ylabel("pressure (raw)")
    ax_p.legend(loc="upper right", ncol=4, fontsize=6)
    ax_p.grid(True, alpha=0.3)

    tof_lines = {s: [ax_t.plot([], [], lw=1.2, label=f"s{s} r{i}")[0] for i in range(3)]
                 for s in sensor_ids}
    ax_t.set_ylabel("ToF (mm)")
    ax_t.set_xlabel("time (s)")
    ax_t.legend(loc="upper right", ncol=max(1, len(sensor_ids)), fontsize=7)
    ax_t.grid(True, alpha=0.3)

    def update(_frame):
        with shared.lock:
            des = list(shared.p_des)
            act = {m: list(shared.p_act[m]) for m in motor_ids}
            press = {s: [list(dq) for dq in shared.pressure[s]] for s in sensor_ids}
            tofb = {s: [list(dq) for dq in shared.tof[s]] for s in sensor_ids}

        line_des.set_data(*_xy(des))
        for m in motor_ids:
            act_lines[m].set_data(*_xy(act[m]))
        for s in sensor_ids:
            for i, ln in enumerate(press_lines[s]):
                ln.set_data(*_xy(press[s][i]))
            for i, ln in enumerate(tof_lines[s]):
                ln.set_data(*_xy(tofb[s][i]))

        # x-window: prefer the command timeline, else any sensor buffer
        xs = [pt[0] for pt in des]
        if not xs:
            for s in sensor_ids:
                if tofb[s][0]:
                    xs = [pt[0] for pt in tofb[s][0]]
                    break
        if xs:
            xmax = xs[-1]
            xmin = max(0.0, xmax - args.window)
            for ax in (ax_m, ax_p, ax_t):
                ax.set_xlim(xmin, xmax if xmax > xmin else xmin + 1e-3)
        for ax in (ax_m, ax_p, ax_t):
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)

        lines = [line_des, *act_lines.values()]
        for s in sensor_ids:
            lines += press_lines[s] + tof_lines[s]
        return lines

    ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)

    try:
        plt.show()
    finally:
        shared.stop = True
        worker.join(timeout=1.0)
        bus.sock.close()


if __name__ == "__main__":
    main()
