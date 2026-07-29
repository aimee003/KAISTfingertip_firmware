#!/usr/bin/env python3
"""Zero one or more motors and hold them at that zeroed position.

A trimmed-down sibling of motor_and_fingertip_test.py: instead of streaming a
sine trajectory, it zeros each motor's encoder, enables it, and then streams a
constant MIT-mode command of p_des = 0, v_des = 0 with position/velocity gains
so the motor actively holds its zero position. Ctrl-C disables the motors.

Linux setup (run once; matches the firmware's FD-BRS timing):
    sudo ip link set can0 down
    sudo ip link set can0 up type can bitrate 1000000 dbitrate 2000000 fd on

Configure everything by editing the constants below (CHANNEL, MOTOR_GAINS,
ZERO_ENCODER) -- there are no command-line flags.

Usage:
    python3 motor_hold.py
"""

import socket
import struct
import time

# ---------------------------------------------------------------------------
# Configuration  (edit these constants instead of passing command-line flags)
# ---------------------------------------------------------------------------
CHANNEL = "can0"          # SocketCAN interface

# Per-motor hold gains: motor CAN id -> (kp, kd).
#   kp = position stiffness (how hard it pulls back to zero)
#   kd = velocity damping
# Add or remove motors here; each can hold with its own gains.
MOTOR_GAINS = {
    3: (0.1, 0.001),
    4: (0.01, 0.0001),
}

ZERO_ENCODER = True       # True: zero each encoder before holding
                          # False: hold the current position (don't re-zero)
TARGET_HZ = 2000          # command send rate (Hz)

# ---------------------------------------------------------------------------
# MIT-mode motor scaling (must match the motor firmware / the other tools)
# ---------------------------------------------------------------------------
GR = 30.0
P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -840.0 / GR, 840.0 / GR
KP_MIN, KP_MAX = 0.0, 2.0
KD_MIN, KD_MAX = 0.0, 0.01
T_MIN, T_MAX = -0.15 * GR, 0.15 * GR
I_MAX = 4.5

T_FF = 0.0
PRINT_EVERY = 1.0      # seconds between status lines

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
# Bus wrapper
# ---------------------------------------------------------------------------
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
            if e.errno == 105:
                print("Error: Network is down. Bring up the interface first.")
                raise SystemExit(1)
            raise

    def enable_motor(self, motor_id):
        print(f"Enabling motor {motor_id}...")
        self.send_to_motor(motor_id, b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFC")
        time.sleep(0.1)

    def disable_motor(self, motor_id):
        print(f"Disabling motor {motor_id}...")
        # Retry: a single non-blocking send can hit a full TX buffer and be
        # dropped, which would leave the motor enabled. Keep trying so Ctrl-C
        # reliably disables the motor.
        for _ in range(100):
            if self.send_to_motor(motor_id, b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFD"):
                return
            time.sleep(0.001)
        print(f"  [warning] could not confirm disable frame queued for motor {motor_id}")

    def zero_encoder(self, motor_id):
        print(f"Zeroing encoder of motor {motor_id}...")
        self.send_to_motor(motor_id, b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFE")
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
    motor_ids = list(MOTOR_GAINS)
    motor_id_set = set(motor_ids)
    motor_state = {m: None for m in motor_ids}  # m -> (p, v, iq, t_des)
    kp = {m: MOTOR_GAINS[m][0] for m in motor_ids}
    kd = {m: MOTOR_GAINS[m][1] for m in motor_ids}

    bus = Bus(CHANNEL)

    def handle_rx():
        count = 0
        while True:
            got = bus.recv()
            if got is None:
                break
            count += 1
            _can_id, _dlc, payload = got
            if len(payload) >= 8 and payload[0] in motor_id_set:
                mid, p, v, iq, t_des = unpack_reply(payload)
                motor_state[mid] = (p, v, iq, t_des)
        return count

    # The hold command: stay at p_des = 0, v_des = 0; the motor's internal
    # PD loop (kp/kd) generates the restoring torque that keeps it there.
    # Gains are per-motor so motors can hold with different stiffness.
    hold_cmds = {m: pack_command(0.0, 0.0, kp[m], kd[m], T_FF) for m in motor_ids}

    for m in motor_ids:
        if ZERO_ENCODER:
            bus.zero_encoder(m)
        bus.enable_motor(m)
    gains_str = ", ".join(f"m{m}(kp={kp[m]}, kd={kd[m]})" for m in motor_ids)
    print(f"Holding motors {motor_ids} at p_des=0 [{gains_str}]. Ctrl-C to stop.")

    period = (1.0 / TARGET_HZ) if TARGET_HZ > 0 else 0.0
    start = time.perf_counter()
    next_deadline = start
    last_print = start
    sent_since_print = 0
    rx_since_print = 0
    backpressure_since_print = 0

    try:
        while True:
            now = time.perf_counter()
            if period > 0.0:
                if now < next_deadline:
                    while time.perf_counter() < next_deadline:
                        pass
                next_deadline += period

            for m in motor_ids:
                if bus.send_to_motor(m, hold_cmds[m]):
                    sent_since_print += 1
                else:
                    backpressure_since_print += 1

            rx_since_print += handle_rx()

            now = time.perf_counter()
            if now - last_print >= PRINT_EVERY:
                elapsed = now - last_print
                tx_hz = sent_since_print / elapsed
                rx_hz = rx_since_print / elapsed
                states = []
                for m in motor_ids:
                    st = motor_state[m]
                    if st is not None:
                        p, v, iq, _tq = st
                        states.append(f"m{m} p={p:+.3f} v={v:+.2f} iq={iq:+.2f}")
                    else:
                        states.append(f"m{m} --")
                print(f"tx={tx_hz:8.1f}Hz rx={rx_hz:8.1f}Hz full={backpressure_since_print:4d} "
                      f"| {' | '.join(states)}")
                sent_since_print = 0
                rx_since_print = 0
                backpressure_since_print = 0
                last_print = now

    except KeyboardInterrupt:
        pass
    finally:
        for m in motor_ids:
            bus.disable_motor(m)
        bus.sock.close()


if __name__ == "__main__":
    main()
