#!/usr/bin/env python3
"""Drive one motor with a sine trajectory while decoding one fingertip's CAN frames.

This is the combination of two existing tools, for the case where a single motor
and a single KAIST fingertip share one FDCAN bus:

  * motor_test_fdcan_sine.py  (kaist_demo/templates) -- streams a 2 kHz sinusoidal
    position/velocity MIT-mode command to the motor and reads its state replies.
  * can_unpack.py             (this directory)        -- decodes the fingertip's
    pressure / ToF reply frames.

Everything runs on ONE raw SocketCAN FD socket: the paced send loop transmits the
sine command, and on every iteration we drain RX and dispatch each frame to either
the motor-reply decoder or the fingertip decoder (demuxed by CAN ID).

Linux setup (run once; matches the firmware's FD-BRS timing):
    sudo ip link set can0 down
    sudo ip link set can0 up type can bitrate 1000000 dbitrate 2000000 fd on

Usage:
    python3 motor_and_fingertip_test.py                       # motor id 1, finger 3, can0
    python3 motor_and_fingertip_test.py --motor-id 1 --finger 3 --channel can0
    python3 motor_and_fingertip_test.py --no-motor            # listen only (like can_unpack.py)
"""

import argparse
import math
import socket
import struct
import time

# ---------------------------------------------------------------------------
# MIT-mode motor scaling (must match the motor firmware / motor_test_fdcan_sine.py)
# ---------------------------------------------------------------------------
GR = 1.0
P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -840.0 / GR, 840.0 / GR
KP_MIN, KP_MAX = 0.0, 2.0
KD_MIN, KD_MAX = 0.0, 0.01
T_MIN, T_MAX = -0.15 * GR, 0.15 * GR
I_MAX = 4.5

# Sine trajectory defaults
TARGET_HZ = 2000
SINE_AMP = -6.0        # rad, peak-to-peak amplitude (negative -> swings 0 to -6)
SINE_FREQ = 25.0       # rad/s
KP = 0.1
KD = 0.0001
T_FF = 0.0
PRINT_EVERY = 1.0      # seconds between status lines

# ---------------------------------------------------------------------------
# Linux SocketCAN / CAN-FD constants
# ---------------------------------------------------------------------------
SOL_CAN_RAW = 101            # socket.SOL_CAN_RAW
CAN_RAW_FD_FRAMES = 5        # socket.CAN_RAW_FD_FRAMES
CANFD_BRS = 0x01             # bit-rate switch flag
CAN_SFF_MASK = 0x000007FF    # 11-bit standard ID mask

# struct canfd_frame { canid_t can_id; u8 len; u8 flags; u8 __res0; u8 __res1; u8 data[64]; }
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
def build_finger_id_map(finger):
    base = (finger - 1) * 4
    return {
        base + 0: ("PR_1", "pressure", (0, 1)),
        base + 1: ("PR_2", "pressure", (2, 3)),
        base + 2: ("PR_3", "pressure", (4, 5)),
        base + 3: ("PR_4", "pressure", (6, 7)),
        base + 4: ("PR_TOF", "tof", None),
        base + 5: ("PR_IMU", "imu", None),
    }


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
# Bus wrapper
# ---------------------------------------------------------------------------
class Bus:
    def __init__(self, channel, motor_id):
        self.motor_id = motor_id
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

    def send_to_motor(self, data, blocking=False):
        try:
            self.sock.send(self._frame(self.motor_id, data))
            return True
        except BlockingIOError:
            return False
        except OSError as e:
            if e.errno == 105:
                print("Error: Network is down. Bring up the interface first.")
                raise SystemExit(1)
            raise

    def enable_motor(self):
        print(f"Enabling motor {self.motor_id}...")
        self.send_to_motor(b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFC", blocking=True)
        time.sleep(0.1)

    def disable_motor(self):
        print(f"Disabling motor {self.motor_id}...")
        self.send_to_motor(b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFD", blocking=True)

    def zero_encoder(self):
        print(f"Zeroing encoder of motor {self.motor_id}...")
        self.send_to_motor(b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFE", blocking=True)
        time.sleep(0.1)

    def recv(self):
        """Non-blocking single frame -> (can_id, dlc, payload) or None."""
        try:
            frame = self.sock.recv(FDCAN_FRAME_SIZE)
        except BlockingIOError:
            return None
        can_id, dlc, _flags, data = struct.unpack(FDCAN_FRAME_FMT, frame)
        return can_id & CAN_SFF_MASK, dlc, data[:dlc]


def main():
    ap = argparse.ArgumentParser(description="Sine-drive a motor and decode a fingertip on one bus")
    ap.add_argument("--channel", default="can0", help="SocketCAN interface (default can0)")
    ap.add_argument("--motor-id", type=int, default=1, help="Motor CAN id (default 1)")
    ap.add_argument("--finger", type=int, default=3, help="SELECTED_FINGER value (default 3)")
    ap.add_argument("--rate", type=int, default=TARGET_HZ, help="Command send rate in Hz (default 2000)")
    ap.add_argument("--no-motor", action="store_true", help="Listen only; do not drive the motor")
    args = ap.parse_args()

    finger_map = build_finger_id_map(args.finger)
    pressure = [None] * 8
    tof = [None] * 3
    seen_pressure = set()
    motor_state = None  # (p, v, iq, t_des)

    bus = Bus(args.channel, args.motor_id)
    print(f"Finger {args.finger} IDs: "
          + ", ".join(f"{name}=0x{cid:X}({cid})" for cid, (name, *_) in finger_map.items()))

    def handle_rx():
        nonlocal motor_state
        count = 0
        while True:
            got = bus.recv()
            if got is None:
                break
            count += 1
            can_id, dlc, payload = got
            entry = finger_map.get(can_id)
            if entry is not None:
                name, kind, channels = entry
                if kind == "pressure":
                    for idx, val in decode_pressure(payload, channels).items():
                        pressure[idx] = val
                    seen_pressure.add(name)
                    if seen_pressure >= {"PR_1", "PR_2", "PR_3", "PR_4"}:
                        seen_pressure.clear()
                elif kind == "tof":
                    for idx, val in decode_tof(payload).items():
                        tof[idx] = val
                # imu: declared but not transmitted by current firmware
            elif len(payload) >= 8 and payload[0] == args.motor_id:
                # MIT-mode state reply (demuxed by data[0] == motor id)
                _id, p, v, iq, t_des = unpack_reply(payload)
                motor_state = (p, v, iq, t_des)
        return count

    if not args.no_motor:
        bus.zero_encoder()
        bus.enable_motor()

    period = (1.0 / args.rate) if args.rate > 0 else 0.0
    half_amp = SINE_AMP / 2.0
    phase_offset = -math.pi / 2

    start = time.perf_counter()
    next_deadline = start
    last_print = start
    sent_since_print = 0
    rx_since_print = 0
    backpressure_since_print = 0
    p_des = v_des = 0.0

    try:
        while True:
            now = time.perf_counter()
            if period > 0.0:
                if now < next_deadline:
                    while time.perf_counter() < next_deadline:
                        pass
                next_deadline += period

            t = time.perf_counter() - start

            if not args.no_motor:
                p_des = half_amp * math.sin(SINE_FREQ * t + phase_offset) + half_amp
                v_des = half_amp * SINE_FREQ * math.cos(SINE_FREQ * t + phase_offset)
                if bus.send_to_motor(pack_command(p_des, v_des, KP, KD, T_FF)):
                    sent_since_print += 1
                else:
                    backpressure_since_print += 1

            rx_since_print += handle_rx()

            now = time.perf_counter()
            if now - last_print >= PRINT_EVERY:
                elapsed = now - last_print
                tx_hz = sent_since_print / elapsed
                rx_hz = rx_since_print / elapsed
                if motor_state is not None:
                    p, v, iq, tq = motor_state
                    motor_str = f"motor p={p:+.3f} v={v:+.2f} iq={iq:+.2f}"
                else:
                    motor_str = "motor --"
                print(f"tx={tx_hz:8.1f}Hz rx={rx_hz:8.1f}Hz full={backpressure_since_print:4d} "
                      f"| p_des={p_des:+.3f} | {motor_str} "
                      f"| pressures={pressure} tof(mm)={tof}")
                sent_since_print = 0
                rx_since_print = 0
                backpressure_since_print = 0
                last_print = now

    except KeyboardInterrupt:
        pass
    finally:
        if not args.no_motor:
            bus.disable_motor()
        bus.sock.close()


if __name__ == "__main__":
    main()
