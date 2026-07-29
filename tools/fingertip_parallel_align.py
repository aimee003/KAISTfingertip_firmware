#!/usr/bin/env python3
"""Hold one motor still while a second motor rotates to keep a hand-held board
parallel to the fingertip, using the fingertip's two ToF ranges as feedback.

Idea:
  The fingertip has two ToF zones a fixed distance apart (TOF_SPACING_INCH).
  If their measured ranges are y1 and y2, the board's tilt relative to the
  fingertip is   theta = atan2(y2 - y1, spacing).  theta = 0 means the board
  surface is parallel to the fingertip. A small PROPORTIONAL controller spins
  the moving motor to drive theta -> 0:
        CW  if theta < 0
        CCW if theta > 0
  (set DIRECTION_SIGN = -1 if your CW/CCW comes out backwards).

  Meanwhile HELD_MOTOR_ID runs a SECOND proportional controller that keeps a
  standoff distance: it reads one ToF zone and rotates to hold that range at
  DISTANCE_SETPOINT (CW if too far, CCW if too close). Set HOLD_DISTANCE=False
  to make it simply hold its zeroed position instead.

Two DIFFERENT sets of gains, don't confuse them:
  * MIT-mode torque-controller gains (kp/kd sent in each CAN command) -- these
    live inside the motor and are kept normal (kp=1, kd=0.001).
  * ALIGN_KP -- the OUTER proportional gain that maps board tilt (rad) to motor
    velocity (rad/s). This is intentionally TINY to start; tune it up slowly.

Safety: the ToF values are raw bytes that WRAP at 256 mm (firmware truncation),
so a wrap glitch can produce a huge fake tilt. Gains are tiny and the angle,
the alignment velocity, and the moving motor's total travel are all clamped so
a bad reading can't fling the board you're holding.

Linux setup (run once):
    sudo ip link set can0 down
    sudo ip link set can0 up type can bitrate 1000000 dbitrate 2000000 fd on

Usage:
    python3 fingertip_parallel_align.py
"""

import math
import socket
import struct
import time

# ===========================================================================
# CONFIGURATION  -- edit these
# ===========================================================================
CHANNEL = "can0"          # SocketCAN interface

# --- which motor does what (CAN ids) ---
HELD_MOTOR_ID = 3         # this motor holds still
MOVING_MOTOR_ID = 4       # this motor rotates to keep the board parallel

# --- fingertip sensor providing the ToF feedback ---
SENSOR_ID = 4             # firmware SELECTED_FINGER value of the fingertip

# --- which ToF range is y1 vs y2 (FILL THIS IN -- not sure which is which) ---
# The fingertip reports 3 ToF ranges with indices 0, 1, 2. Pick the two zones
# that are TOF_SPACING_INCH apart along the axis you want to keep parallel.
TOF1_INDEX = 1            # index of y1   (<-- fill in / swap once you know)
TOF2_INDEX = 2            # index of y2   (<-- fill in / swap once you know)
TOF_SPACING_INCH = 1.4    # distance between the two ToF zones

# The ToF only updates ~10 Hz and is noisy. Exponential moving-average filter,
# applied to each ToF range as new frames arrive:
#   filt += TOF_FILTER_ALPHA * (raw - filt)
# 1.0 = no filtering (raw), smaller = smoother but more lag. ~0.3-0.5 is a good
# start. BOTH controllers use the FILTERED values.
TOF_FILTER_ALPHA = 0.4

# --- direction: flip if the motor turns the wrong way ---
DIRECTION_SIGN = +1       # set to -1 if CW/CCW is reversed on your setup

# --- OUTER alignment PD gains (the things you tune) ---
# Output is motor velocity (rad/s): P acts on the tilt, D damps its rate. The
# derivative is computed from the (filtered) ToF at the sensor's ~10 Hz update
# rate, so it stays sane despite the slow sampling. START TINY.
ALIGN_KP = 5
ALIGN_KD = 0.0001       # damping on tilt rate; raise to reduce overshoot/chatter

# --- MIT-mode torque-controller gains (inside the motor; keep normal) ---
# These are the INNER torque loops, not the outer P-controllers above.
HELD_KP, HELD_KD = 1.0, 0.01      # DISTANCE motor (HELD_MOTOR_ID) torque loop
MOVE_KP, MOVE_KD = 1.0, 0.01      # ALIGNMENT motor (MOVING_MOTOR_ID) torque loop

# --- safety limits (because the ToF wraps and you're holding a board) ---
MAX_ANGLE_DEG = 30.0      # ignore/clamp tilt beyond this (wrap protection)
MAX_ALIGN_VEL = 5       # rad/s, cap on commanded alignment velocity
MAX_TRAVEL_RAD = 3.0      # rad, clamp the moving motor's total displacement
ANGLE_DEADBAND_DEG = 0.0  # below this tilt, don't move (raise to stop drift)

# ---------------------------------------------------------------------------
# DISTANCE controller (the HELD_MOTOR_ID now follows a standoff distance)
# ---------------------------------------------------------------------------
# A second proportional controller. The regulated distance is the AVERAGE of the
# two alignment zones, d = (y1 + y2) / 2, driven to DISTANCE_SETPOINT:
#       CW  if  mean(y1, y2) > setpoint
#       CCW if  mean(y1, y2) < setpoint
# Set HOLD_DISTANCE = False to go back to just holding still at zero.
HOLD_DISTANCE = True
DISTANCE_SETPOINT = 100    # target mean ToF reading (raw byte / mm, 0..255)

# Outer PD gains: motor velocity (rad/s) per unit of distance error.
# START TINY and tune up. (Distance error is in raw ToF units, ~mm.)
DIST_KP = 0.02
DIST_KD = 0.0001             # damping on distance-error rate; raise to reduce overshoot
# Default: CW when ToF > setpoint (error > 0). Flip to +1 to reverse.
DIST_DIRECTION_SIGN = +1

MAX_DIST_VEL = 5.0        # rad/s, cap on commanded distance velocity
MAX_DIST_TRAVEL_RAD = 3.0 # rad, clamp the held motor's total displacement
DIST_DEADBAND = 0.0       # ToF units; below this error, don't move

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
# ---------------------------------------------------------------------------
def tof_can_id(sensor_id):
    """CAN id that carries this sensor's 3 ToF ranges (PR_TOF)."""
    return (sensor_id - 1) * 4 + 4


def decode_tof(data):
    """range[0..2] packed as single bytes (uint8, truncated -> wraps at 256)."""
    return [data[i] if i < len(data) else None for i in range(3)]


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
        # Retry: a dropped single send would leave the motor enabled.
        for _ in range(100):
            if self.send_to_motor(motor_id, b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFD"):
                return
            time.sleep(0.001)
        print(f"  [warning] could not confirm disable frame for motor {motor_id}")

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
        """Reliably disable every motor, then close the socket.

        Sends the MIT disable command MANY times so several reach each motor
        even if individual frames are lost or stuck behind the command backlog,
        then waits for the kernel TX buffer to actually drain onto the bus
        BEFORE closing (closing can discard still-queued frames -- which is why
        a single disable + immediate close would leave the motor holding).
        """
        print("\nDisabling motors...")
        disable = b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFD"
        for _ in range(40):
            for m in motor_ids:
                self._send_blocking(m, disable)
            try:
                time.sleep(0.004)
            except KeyboardInterrupt:
                pass  # ignore extra Ctrl-C during shutdown; keep disabling
        # let the last disable frames transmit before closing the socket
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
    spacing_mm = TOF_SPACING_INCH * 25.4
    max_angle_rad = math.radians(MAX_ANGLE_DEG)
    deadband_rad = math.radians(ANGLE_DEADBAND_DEG)
    tof_id = tof_can_id(SENSOR_ID)
    motor_ids = [HELD_MOTOR_ID, MOVING_MOTOR_ID]
    motor_id_set = set(motor_ids)

    bus = Bus(CHANNEL)
    print(f"Held motor   = {HELD_MOTOR_ID}")
    print(f"Moving motor = {MOVING_MOTOR_ID}")
    print(f"ToF frame id = 0x{tof_id:X} ({tof_id})  using y1=range[{TOF1_INDEX}], y2=range[{TOF2_INDEX}]")
    print(f"ALIGN_KP={ALIGN_KP}  DIRECTION_SIGN={DIRECTION_SIGN}  spacing={spacing_mm:.1f} mm")

    tof_raw = [None, None, None]
    tof_filt = [None, None, None]   # EMA-smoothed ToF ranges
    new_tof = False                 # set True on loops where a fresh ToF frame arrived
    motor_state = {m: None for m in motor_ids}  # m -> (p, v, iq, t_des)

    def handle_rx():
        nonlocal new_tof
        while True:
            got = bus.recv()
            if got is None:
                break
            can_id, _dlc, payload = got
            if can_id == tof_id:
                tof_raw[:] = decode_tof(payload)
                for i in range(3):
                    r = tof_raw[i]
                    if r is None:
                        continue
                    if tof_filt[i] is None:
                        tof_filt[i] = float(r)
                    else:
                        tof_filt[i] += TOF_FILTER_ALPHA * (r - tof_filt[i])
                new_tof = True
            elif len(payload) >= 8 and payload[0] in motor_id_set:
                mid, p, v, iq, t_des = unpack_reply(payload)
                motor_state[mid] = (p, v, iq, t_des)

    for m in motor_ids:
        if ZERO_ENCODER:
            bus.zero_encoder(m)
        bus.enable_motor(m)
    print("Running. Tilt the board (angle) / move it nearer-farther (distance). Ctrl-C to stop.")

    move_p_des = 0.0          # integrated target angle of the moving motor (rad)
    held_p_des = 0.0          # integrated target of the held/distance motor (rad)
    last_dist_err = 0.0
    last_d = None
    last_theta = 0.0
    # derivative state (updated at the ToF sample rate, held between samples)
    prev_theta = 0.0
    theta_rate = 0.0
    prev_dist_err = 0.0
    dist_rate = 0.0
    period = (1.0 / TARGET_HZ) if TARGET_HZ > 0 else 0.0
    start = time.perf_counter()
    next_deadline = start
    last_loop = start
    last_print = start
    prev_theta_t = start
    prev_dist_t = start

    try:
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

            # ---- filtered signals + their sample-rate derivatives ----
            y1 = tof_filt[TOF1_INDEX]
            y2 = tof_filt[TOF2_INDEX]
            have_signal = y1 is not None and y2 is not None
            theta = 0.0
            dist_err = last_dist_err
            if have_signal:
                theta = _clamp(math.atan2(y2 - y1, spacing_mm),
                               -max_angle_rad, max_angle_rad)   # tilt (rad), wrap-guarded
                last_theta = theta
                last_d = 0.5 * (y1 + y2)                        # distance = mean of the two zones
                dist_err = last_d - DISTANCE_SETPOINT           # >0 too far, <0 too close
                last_dist_err = dist_err
                if new_tof:
                    # derivatives from fresh samples only (~10 Hz), held between
                    dt_th = now - prev_theta_t
                    if dt_th > 0:
                        theta_rate = (theta - prev_theta) / dt_th
                        prev_theta, prev_theta_t = theta, now
                    dt_ds = now - prev_dist_t
                    if dt_ds > 0:
                        dist_rate = (dist_err - prev_dist_err) / dt_ds
                        prev_dist_err, prev_dist_t = dist_err, now
            new_tof = False

            # ---- alignment controller (PD: P on tilt, D damps tilt rate) ----
            align_vel = 0.0
            if have_signal and abs(theta) > deadband_rad:
                align_vel = DIRECTION_SIGN * (ALIGN_KP * theta + ALIGN_KD * theta_rate)
                align_vel = _clamp(align_vel, -MAX_ALIGN_VEL, MAX_ALIGN_VEL)
            move_p_des = _clamp(move_p_des + align_vel * dt,
                                -MAX_TRAVEL_RAD, MAX_TRAVEL_RAD)
            move_cmd = pack_command(move_p_des, align_vel, MOVE_KP, MOVE_KD, T_FF)

            # ---- distance controller (PD: P on distance error, D damps its rate) ----
            dist_vel = 0.0
            if HOLD_DISTANCE:
                if have_signal and abs(dist_err) > DIST_DEADBAND:
                    # err > 0 (too far) -> CW by default; flip via sign.
                    dist_vel = DIST_DIRECTION_SIGN * (DIST_KP * dist_err + DIST_KD * dist_rate)
                    dist_vel = _clamp(dist_vel, -MAX_DIST_VEL, MAX_DIST_VEL)
                held_p_des = _clamp(held_p_des + dist_vel * dt,
                                    -MAX_DIST_TRAVEL_RAD, MAX_DIST_TRAVEL_RAD)
                held_cmd = pack_command(held_p_des, dist_vel, HELD_KP, HELD_KD, T_FF)
            else:
                # just hold still at zero
                held_cmd = pack_command(0.0, 0.0, HELD_KP, HELD_KD, T_FF)

            # ---- send ----
            bus.send_to_motor(HELD_MOTOR_ID, held_cmd)
            bus.send_to_motor(MOVING_MOTOR_ID, move_cmd)

            # ---- status ----
            if now - last_print >= PRINT_EVERY:
                held = motor_state[HELD_MOTOR_ID]
                mov = motor_state[MOVING_MOTOR_ID]
                held_p = f"{held[0]:+.3f}" if held else "--"
                mov_p = f"{mov[0]:+.3f}" if mov else "--"
                y1s = f"{y1:5.1f}" if y1 is not None else "   --"
                y2s = f"{y2:5.1f}" if y2 is not None else "   --"
                ds = f"{last_d:5.1f}" if last_d is not None else "   --"
                print(f"ANGLE: y1={y1s} y2={y2s} "
                      f"theta={math.degrees(last_theta):+6.1f}deg v={align_vel:+.3f} "
                      f"move_p={move_p_des:+.3f} (m{MOVING_MOTOR_ID} p={mov_p}) "
                      f"|| DIST: d={ds} set={DISTANCE_SETPOINT} "
                      f"err={last_dist_err:+6.1f} v={dist_vel:+.3f} "
                      f"held_p={held_p_des:+.3f} (m{HELD_MOTOR_ID} p={held_p})")
                last_print = now

    except KeyboardInterrupt:
        pass
    finally:
        # single cleanup path: robustly disable both motors, then close
        bus.shutdown(motor_ids)


if __name__ == "__main__":
    main()
