#!/usr/bin/env python3
"""Drive both finger motors together off a single fingertip PRESSURE channel,
keeping the fingertip orientation fixed while a PD controller pushes the finger
toward / away from contact based on a pressure threshold.

Idea:
  Two motors form the finger: MOTOR_A_ID (3, the bottom joint) and MOTOR_B_ID
  (4, the fingertip joint). As in motor_and_fingertip_plot.py, both motors are
  commanded with the SAME position trajectory so the fingertip ORIENTATION stays
  constant -- the bottom motor swings the finger and the fingertip motor counters
  it (one CW, one CCW) so the tip keeps pointing the same way. The per-motor sign
  knobs (MOTOR_A_SIGN / MOTOR_B_SIGN) let you flip a motor if its mounting makes
  it turn the wrong way.

  A single pressure channel (PRESSURE_CHANNEL = 5 on SENSOR_ID = 4) is the only
  feedback. It is smoothed with a 30-sample MOVING AVERAGE, then a PD controller
  acts on the error vs. PRESSURE_THRESHOLD (15000):
        pressure < threshold  ->  move one way   (seeking / pressing in)
        pressure > threshold  ->  move the other way (backing off)
  The sign of the error already flips the direction at the threshold; DIRECTION_SIGN
  flips the WHOLE convention if the finger moves the wrong way to start with.

PD on what:
  error      = pressure_filtered - PRESSURE_THRESHOLD   (raw pressure units)
  d(error)/dt at the pressure sample rate
  vel (rad/s) = DIRECTION_SIGN * (PRESS_KP * error + PRESS_KD * error_rate)
  Pressure numbers are large (~10000s), so PRESS_KP is correspondingly TINY.
  START TINY and tune up.

Two DIFFERENT sets of gains -- don't confuse them:
  * MIT-mode torque-controller gains (KP/KD packed in each CAN command) live
    inside the motor and stay normal (kp=1, kd=0.01).
  * PRESS_KP / PRESS_KD -- the OUTER PD gains mapping pressure error -> motor
    velocity. These are the ones you tune.

Linux setup (run once):
    sudo ip link set can0 down
    sudo ip link set can0 up type can bitrate 1000000 dbitrate 2000000 fd on

Usage:
    python3 fingertip_pressure_align.py
"""

import socket
import struct
import time
from collections import deque

# ===========================================================================
# CONFIGURATION  -- edit these
# ===========================================================================
CHANNEL = "can0"          # SocketCAN interface

# --- which motor does what (CAN ids) ---
MOTOR_A_ID = 3            # bottom motor
MOTOR_B_ID = 4            # fingertip motor
# Both are commanded with the same trajectory to hold the tip orientation. Flip a
# sign if a motor turns the wrong way physically (mounting-dependent).
MOTOR_A_SIGN = +1
MOTOR_B_SIGN = +1

# --- fingertip sensor + pressure channel providing the feedback ---
SENSOR_ID = 4             # firmware SELECTED_FINGER value of the fingertip
PRESSURE_CHANNEL = 5      # which of the 8 pressure channels to use (0..7)

# --- threshold + direction ---
PRESSURE_THRESHOLD = 8000  # move one way below this, the other way above it
DIRECTION_SIGN = -1         # set to -1 if the finger moves the wrong way overall

# --- moving-average filter on the pressure channel ---
FILTER_WINDOW = 30          # number of samples in the time-average window

# --- OUTER pressure PD gains (the things you tune) ---
# Output is motor velocity (rad/s). Pressure error is in raw units (~10000s),
# so these are TINY. START TINY and raise slowly.
PRESS_KP = 0.0008
PRESS_KD = 0.000001        # damping on pressure-error rate; raise to reduce overshoot

# --- MIT-mode torque-controller gains (inside the motor; keep normal) ---
MOTOR_KP, MOTOR_KD = 1.0, 0.01

# --- safety limits ---
MAX_VEL = 3.0             # rad/s, cap on commanded velocity
# Hard position limits applied to EVERY motor command (rad). Each motor's
# commanded position is clipped to [POS_MIN, POS_MAX] before it goes on the bus.
POS_MIN, POS_MAX = -1.0, 0.75
PRESS_DEADBAND = 200      # raw pressure units; below this |error|, don't move
MIN_ACTIVE_PRESSURE = 5700  # raw pressure units; below this filtered value, don't move at all

ZERO_ENCODER = True       # zero both encoders before starting
TARGET_HZ = 2000          # command send rate (Hz)
PRINT_EVERY = 0.5         # seconds between status lines

# ===========================================================================
# MIT-mode motor scaling (must match the motor firmware / the other tools)
# ===========================================================================
GR = 30.0
P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -840.0 / GR, 840.0 / GR
KP_MIN, KP_MAX = 0.0, 2.0
KD_MIN, KD_MAX = 0.0, 0.01
T_MIN, T_MAX = -0.15 * GR, 0.15 * GR
I_MAX = 4.5
T_FF = 0.0

# ---------------------------------------------------------------------------
# Linux SocketCAN / CAN-FD constants
# ---------------------------------------------------------------------------
SOL_CAN_RAW = 101
CAN_RAW_FD_FRAMES = 5
CANFD_BRS = 0x01
CAN_SFF_MASK = 0x000007FF
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
# Fingertip decoding (mirrors fingertip.cpp)
#
# Each pressure CAN frame carries two int32 channels. Channel c lives in frame
# base + (c // 2), where base = (sensor_id - 1) * 4; it is the FIRST int32 if c
# is even, the SECOND if c is odd.
# ---------------------------------------------------------------------------
def pressure_can_id(sensor_id, channel):
    return (sensor_id - 1) * 4 + (channel // 2)


def decode_pressure_channel(data, channel):
    """Return the int32 value of `channel` from its PR_x frame, or None."""
    if len(data) < 8:
        return None
    a, b = struct.unpack(">ii", bytes(data[:8]))
    return a if (channel % 2 == 0) else b


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

    def _send_blocking(self, motor_id, data, timeout=0.05):
        """Send one frame, spinning past transient BlockingIOError (full TX buf)."""
        deadline = time.perf_counter() + timeout
        while True:
            try:
                self.sock.send(self._frame(motor_id, data))
                return True
            except BlockingIOError:
                if time.perf_counter() > deadline:
                    return False
            except OSError:
                return False

    def shutdown(self, motor_ids):
        """Reliably disable every motor, then close the socket."""
        print("\nDisabling motors...")
        disable = b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFD"
        for _ in range(40):
            for m in motor_ids:
                self._send_blocking(m, disable)
            try:
                time.sleep(0.004)
            except KeyboardInterrupt:
                pass  # ignore extra Ctrl-C during shutdown; keep disabling
        try:
            time.sleep(0.05)
        except KeyboardInterrupt:
            pass
        try:
            self.sock.close()
        except OSError:
            pass
        print("Motors disabled, socket closed.")

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


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def main():
    press_id = pressure_can_id(SENSOR_ID, PRESSURE_CHANNEL)
    motor_ids = [MOTOR_A_ID, MOTOR_B_ID]
    motor_id_set = set(motor_ids)
    motor_sign = {MOTOR_A_ID: MOTOR_A_SIGN, MOTOR_B_ID: MOTOR_B_SIGN}

    bus = Bus(CHANNEL)
    print(f"Motor A (bottom)   = {MOTOR_A_ID} (sign {MOTOR_A_SIGN:+d})")
    print(f"Motor B (fingertip)= {MOTOR_B_ID} (sign {MOTOR_B_SIGN:+d})")
    print(f"Pressure frame id  = 0x{press_id:X} ({press_id})  channel {PRESSURE_CHANNEL}")
    print(f"threshold={PRESSURE_THRESHOLD}  DIRECTION_SIGN={DIRECTION_SIGN}  "
          f"PRESS_KP={PRESS_KP}  window={FILTER_WINDOW}")

    samples = deque(maxlen=FILTER_WINDOW)   # raw pressure samples for the moving average
    press_filt = None                       # current moving-average value
    new_press = False                       # True on loops where a fresh pressure frame arrived
    motor_state = {m: None for m in motor_ids}  # m -> (p, v, iq, t_des)

    def handle_rx():
        nonlocal new_press
        while True:
            got = bus.recv()
            if got is None:
                break
            can_id, _dlc, payload = got
            if can_id == press_id:
                val = decode_pressure_channel(payload, PRESSURE_CHANNEL)
                if val is not None:
                    samples.append(val)
                    new_press = True
            elif len(payload) >= 8 and payload[0] in motor_id_set:
                mid, p, v, iq, t_des = unpack_reply(payload)
                motor_state[mid] = (p, v, iq, t_des)

    # Everything that enables a motor lives inside this try so the finally below
    # ALWAYS runs -- including on a Ctrl-C during the zero/enable setup loop, which
    # would otherwise leave already-enabled motors holding.
    try:
        for m in motor_ids:
            if ZERO_ENCODER:
                bus.zero_encoder(m)
            bus.enable_motor(m)
        print("Running. Press the fingertip to cross the threshold. Ctrl-C to stop.")

        p_des = 0.0               # integrated target position (rad), shared by both motors
        last_err = 0.0
        prev_err = 0.0
        err_rate = 0.0
        period = (1.0 / TARGET_HZ) if TARGET_HZ > 0 else 0.0
        start = time.perf_counter()
        next_deadline = start
        last_loop = start
        last_print = start
        prev_err_t = start

        while True:
            now = time.perf_counter()
            if period > 0.0:
                if now < next_deadline:
                    while time.perf_counter() < next_deadline:
                        pass
                next_deadline += period

            now = time.perf_counter()
            dt = now - last_loop
            last_loop = now

            handle_rx()

            # ---- filtered pressure (moving average) + its sample-rate derivative ----
            have_signal = len(samples) > 0
            if have_signal:
                press_filt = sum(samples) / len(samples)
                err = press_filt - PRESSURE_THRESHOLD   # <0 below threshold, >0 above
                last_err = err
                if new_press:
                    dt_e = now - prev_err_t
                    if dt_e > 0:
                        err_rate = (err - prev_err) / dt_e
                        prev_err, prev_err_t = err, now
            new_press = False

            # ---- pressure PD controller (P on error, D damps its rate) ----
            # Hold still until the fingertip is actually being touched: below
            # MIN_ACTIVE_PRESSURE the filtered reading is noise, so don't move.
            active = press_filt is not None and press_filt >= MIN_ACTIVE_PRESSURE
            vel = 0.0
            if have_signal and active and abs(last_err) > PRESS_DEADBAND:
                vel = DIRECTION_SIGN * (PRESS_KP * last_err + PRESS_KD * err_rate)
                vel = _clamp(vel, -MAX_VEL, MAX_VEL)
            # integrate, bounding the integrator to the position limit so it
            # cannot wind up past what the motors are actually allowed to reach
            p_des = _clamp(p_des + vel * dt, POS_MIN, POS_MAX)

            # ---- both motors track the same trajectory (orientation held) ----
            for m in motor_ids:
                s = motor_sign[m]
                p_cmd = _clamp(s * p_des, POS_MIN, POS_MAX)   # hard per-motor clip
                cmd = pack_command(p_cmd, s * vel, MOTOR_KP, MOTOR_KD, T_FF)
                bus.send_to_motor(m, cmd)

            # ---- status ----
            if now - last_print >= PRINT_EVERY:
                a = motor_state[MOTOR_A_ID]
                b = motor_state[MOTOR_B_ID]
                a_p = f"{a[0]:+.3f}" if a else "--"
                b_p = f"{b[0]:+.3f}" if b else "--"
                ps = f"{press_filt:8.1f}" if press_filt is not None else "      --"
                print(f"press={ps} (thr={PRESSURE_THRESHOLD}) err={last_err:+9.1f} "
                      f"v={vel:+.3f} p_des={p_des:+.3f} "
                      f"(m{MOTOR_A_ID} p={a_p}  m{MOTOR_B_ID} p={b_p})  n={len(samples)}")
                last_print = now

    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown(motor_ids)


if __name__ == "__main__":
    main()
