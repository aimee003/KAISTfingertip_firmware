#!/usr/bin/env python3
"""Live monitor for the KAIST hand's thumb bus: 2 motors + 2 fingertip sensors.

Bus `thumb` (device_id 0xFFFFFFFB) carries, per configs/kaist_hand/hardware/default.yaml:
    motor id 1  L_thumb_mcp     motor id 2  L_thumb_pip
    fingertip TX 23 / RX 32     fingertip TX 24 / RX 42
Motors reply on frame id 0 (master_id) and are demuxed by data[0], the echoed
motor id.

SAFETY: commands are streamed with kp = kd = 0, i.e. zero impedance -- the
motors are free and backdrivable, they will not hold or drive to a setpoint.
Motors are NOT enabled at startup; press Enable when you want them armed.
Position feedback arrives either way, because the firmware replies to every
frame regardless of MOTOR/MENU mode.

    ~/miniconda3/envs/manip_env/bin/python tools/thumb_monitor.py
"""

import math
import struct
import threading
import time
from collections import deque

import can
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, RadioButtons

# --- config: edit here -----------------------------------------------------
BUS_DEVICE_ID   = 0xFFFFFFFB    # thumb bus (the finger above the thumb swivel)
FINGERTIPS      = [(23, 32, "ft 23"), (24, 42, "ft 24")]   # (tx_id, rx_id, label)
MOTORS          = [(1, "mcp"), (2, "pip")]                 # (motor_id, label)
MOTOR_MASTER_ID = 0             # frame id motors reply on

MOTOR_KP        = 0.0           # <-- keep 0 until you want the motors to hold
MOTOR_KD        = 0.0           # <-- keep 0 until you want damping
MOTOR_P_DES     = 0.0
MOTOR_V_DES     = 0.0
MOTOR_T_FF      = 0.0
CMD_RATE_HZ     = 1000          # keep-alive rate; firmware times out at ~47 ms

# Per-motor position sweep (the "sweep" buttons). Zeroes the motor at its
# current pose, then tracks amp*sin(2*pi*t/period) about that zero.
MOTION_KP       = 2.0           # Nm/rad  -> ~0.8 Nm at full deflection
MOTION_KD       = 0.002         # Nm/(rad/s)
MOTION_AMP      = 0.3   # rad, sweep is +/- this ABOUT THE ZEROED POSE
MOTION_PERIOD   = 2          # s per full cycle -- deliberately slow
MOTION_RAMP     = 1           # s to ramp amplitude 0 -> full, so nothing jerks
ZERO_BEFORE_FIRST_SWEEP = True  # zero once per session, not on every start
ZERO_SETTLE     = 0.40          # s to let the flash write + menu printf finish
ENABLE_ON_START = False         # True arms the motors without pressing Enable

HISTORY_SECONDS = 5.0
REDRAW_PERIOD   = 0.1
TRAIL_POINTS    = 100
FORCE_SCALE     = 1.0           # mm per N, for the 3D force arrow
STALE_MS        = 500           # no frame for this long -> NO DATA

# --- motor MIT scaling -----------------------------------------------------
# Must match what is flashed on these motors (pico-motor-driver user_config.h);
# values taken from configs/kaist_hand/hardware/default.yaml motor_limits.
P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -28.0, 28.0
KP_MIN, KP_MAX = 0.0, 10.0
KD_MIN, KD_MAX = 0.0, 0.1
T_MIN, T_MAX = -4.5, 4.5
I_MAX = 5.5

ENABLE_CMD  = b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFC"
DISABLE_CMD = b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFD"
ZERO_CMD    = b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFE"

# --- fingertip frame -------------------------------------------------------
BODY_FMT = ">12h"               # bytes 2..25, twelve big-endian int16
FT_CMD_CALIBRATE = 0x0B
RECAL_CHOICES    = (250, 500, 1000, 2000)   # selectable sample counts
RECAL_DEFAULT    = 1000
STATUS_NAMES = {0: "OK", 1: "WARMUP", 2: "CALIBRATING"}
STATUS_COLORS = {0: "#0d6360", 1: "#b06d12", 2: "#9c3f2e"}

# Fingertip surface, from NN_SE_* in Fingertip_KAIST/Core/Inc/kaist_net.h.
SE_A, SE_B, SE_C = 15.5, 12.0, 5.5
SE_E1, SE_E2 = 1.0, 0.5

PCAN_FD_TIMING = dict(f_clock_mhz=80,
                      nom_brp=1, nom_tseg1=63, nom_tseg2=16, nom_sjw=16,
                      data_brp=1, data_tseg1=31, data_tseg2=8, data_sjw=8)


def frame_airtime_us(nbytes, nom_bps=1e6, dat_bps=2e6):
    """Airtime of one standard-id CAN FD frame with BRS, in microseconds.

    ~29 bits always run at the nominal rate (SOF + id + control up to BRS, then
    ACK/EOF/IFS at the end); the rest runs at the data rate. Stuff bits are
    approximated. This is the term that makes short frames expensive."""
    crc = 17 if nbytes <= 16 else 21
    data_bits = 1 + 4 + 8 * nbytes + crc + crc // 4 + 1
    return 29.0 / nom_bps * 1e6 + data_bits / dat_bps * 1e6


class BusLoad:
    """Estimated utilisation. rx_us and tx_us each have a single writer
    thread, so no lock is needed -- the GUI only reads their sum."""

    def __init__(self):
        self.rx_us = 0.0
        self.tx_us = 0.0
        self.load = 0.0
        self._mark_t = None
        self._mark_us = 0.0

    def sample(self, now):
        total = self.rx_us + self.tx_us
        if self._mark_t is None:
            self._mark_t, self._mark_us = now, total
            return
        dt = now - self._mark_t
        if dt >= 1.0:
            self.load = (total - self._mark_us) / (dt * 1e6)
            self._mark_t, self._mark_us = now, total


def signed_pow(v, e):
    return np.sign(v) * (np.abs(v) ** e)


def _f2u(x, lo, hi, bits):
    x = max(min(x, hi), lo)
    return int((x - lo) * ((2 ** bits - 1) / (hi - lo)))


def _u2f(i, lo, hi, bits):
    return (float(i) * (hi - lo) / (2 ** bits - 1)) + lo


def pack_motor_command(p, v, kp, kd, t_ff):
    p_i, v_i = _f2u(p, P_MIN, P_MAX, 16), _f2u(v, V_MIN, V_MAX, 12)
    kp_i, kd_i = _f2u(kp, KP_MIN, KP_MAX, 12), _f2u(kd, KD_MIN, KD_MAX, 12)
    t_i = _f2u(t_ff, T_MIN, T_MAX, 12)
    return bytes([
        (p_i >> 8) & 0xFF,
        p_i & 0xFF,
        (v_i >> 4) & 0xFF,
        ((v_i << 4) & 0xF0) | ((kp_i >> 8) & 0x0F),
        kp_i & 0xFF,
        (kd_i >> 4) & 0xFF,
        ((kd_i << 4) & 0xF0) | ((t_i >> 8) & 0x0F),
        t_i & 0xFF,
    ])


def unpack_motor_reply(d):
    """-> (motor_id, position rad, velocity rad/s, iq A, t_des Nm)"""
    return (d[0],
            _u2f((d[1] << 8) | d[2], P_MIN, P_MAX, 16),
            _u2f((d[3] << 4) | ((d[4] >> 4) & 0x0F), V_MIN, V_MAX, 12),
            _u2f(((d[4] & 0x0F) << 8) | d[5], -I_MAX, I_MAX, 12),
            _u2f((d[6] << 4) | (d[7] >> 4), T_MIN, T_MAX, 12))


def decode_fingertip(payload):
    if len(payload) < 26:
        return None
    v = struct.unpack(BODY_FMT, bytes(payload[2:26]))
    return dict(status=payload[0], seq=payload[1], prob=v[0] / 1000.0,
                F=(v[1] / 100.0, v[2] / 100.0, v[3] / 100.0),
                u=(v[4] / 100.0, v[5] / 100.0, v[6] / 100.0),
                range=(v[7], v[8]), rpy=(v[9], v[10], v[11]))


class RateMeter:
    """Frames/s over a 1 s window, plus the worst inter-arrival gap seen.
    max_gap distinguishes "the sender paused" from "the GUI starved us"."""

    def __init__(self):
        self.count = 0
        self.rate = 0.0
        self.last = None
        self.max_gap = 0.0
        self._mark_t = None
        self._mark_n = 0

    def tick(self, now):
        if self.last is not None:
            gap = (now - self.last) * 1e3
            if gap > self.max_gap:
                self.max_gap = gap
        self.last = now
        self.count += 1
        if self._mark_t is None:
            self._mark_t = now
        dt = now - self._mark_t
        if dt >= 1.0:
            self.rate = (self.count - self._mark_n) / dt
            self._mark_t, self._mark_n = now, self.count

    def age_ms(self, now):
        return (now - self.last) * 1e3 if self.last is not None else float("inf")


class FingertipState:
    def __init__(self, tx_id, rx_id, label, n):
        self.tx_id, self.rx_id, self.label = tx_id, rx_id, label
        self.t = deque(maxlen=n)
        self.prob = deque(maxlen=n)
        self.fmag = deque(maxlen=n)
        self.F = [deque(maxlen=n) for _ in range(3)]
        self.u = [deque(maxlen=n) for _ in range(3)]
        self.rng = [deque(maxlen=n) for _ in range(2)]
        self.status = None
        self.dropped = 0
        self.last_seq = None
        self.meter = RateMeter()


class MotorState:
    def __init__(self, motor_id, label, n):
        self.motor_id, self.label = motor_id, label
        self.t = deque(maxlen=n)
        self.p = deque(maxlen=n)
        self.v = deque(maxlen=n)
        self.iq = deque(maxlen=n)
        self.pdes = deque(maxlen=n)
        self.meter = RateMeter()
        self.moving = False
        self.t_start = 0.0
        self.p_des = 0.0
        self.zeroed = False


class Receiver(threading.Thread):
    """Single RX thread; demuxes fingertip frames and motor replies."""

    daemon = True

    def __init__(self, bus, fts, motors, t0, busload):
        super().__init__()
        self.bus, self.t0, self.busload = bus, t0, busload
        self.stop_flag = threading.Event()
        self.by_tx = {f.tx_id: f for f in fts}
        self.by_mid = {m.motor_id: m for m in motors}

    def run(self):
        while not self.stop_flag.is_set():
            msg = self.bus.recv(timeout=0.2)
            if msg is None:
                continue
            abs_now = time.monotonic()
            now = abs_now - self.t0
            self.busload.rx_us += frame_airtime_us(len(msg.data))

            if msg.arbitration_id == MOTOR_MASTER_ID and len(msg.data) >= 8:
                mid, p, v, iq, _t = unpack_motor_reply(msg.data)
                m = self.by_mid.get(mid)
                if m is not None:
                    m.t.append(now); m.p.append(p); m.v.append(v); m.iq.append(iq)
                    m.pdes.append(m.p_des)
                    m.meter.tick(abs_now)
                continue

            f = self.by_tx.get(msg.arbitration_id)
            if f is None:
                continue
            s = decode_fingertip(msg.data)
            if s is None:
                continue
            if f.last_seq is not None:
                gap = (s["seq"] - f.last_seq) & 0xFF
                if gap != 1:
                    f.dropped += gap - 1
            f.last_seq = s["seq"]
            f.t.append(now)
            f.prob.append(s["prob"])
            fx, fy, fz = s["F"]
            f.fmag.append((fx * fx + fy * fy + fz * fz) ** 0.5)
            for i in range(3):
                f.F[i].append(s["F"][i]); f.u[i].append(s["u"][i])
            for i in range(2):
                f.rng[i].append(s["range"][i])
            f.status = s["status"]
            f.meter.tick(abs_now)


class Commander(threading.Thread):
    """Streams zero-impedance commands so the firmware's CAN watchdog never
    trips. The watchdog is ~47 ms; a silent gap drops a motor back to MENU."""

    daemon = True

    def __init__(self, bus, motors, lock, busload):
        super().__init__()
        self.bus, self.motors, self.lock = bus, motors, lock
        self.busload = busload
        self.stop_flag = threading.Event()
        self.armed = False
        self.idle = pack_motor_command(MOTOR_P_DES, MOTOR_V_DES,
                                       MOTOR_KP, MOTOR_KD, MOTOR_T_FF)
        self.meter = RateMeter()

    def _send(self, mid, data):
        self.busload.tx_us += frame_airtime_us(len(data))
        with self.lock:
            self.bus.send(can.Message(arbitration_id=mid, data=data,
                                      is_extended_id=False, is_fd=True,
                                      bitrate_switch=True))

    def enable(self):
        for m in self.motors:
            self._send(m.motor_id, ENABLE_CMD)
        self.armed = True          # streaming starts immediately, no sleep
        print("motors ENABLED (kp=kd=0, free)")

    def disable(self):
        self.armed = False
        for m in self.motors:
            m.moving = False; m.p_des = 0.0
        for m in self.motors:
            self._send(m.motor_id, DISABLE_CMD)
        print("motors DISABLED")

    def zero(self, m):
        """Set the motor's current pose as encoder zero.

        Only honoured in MENU mode, so DISABLE first. This is EXPENSIVE and not
        idempotent-friendly: the firmware handles ZERO_CMD inside the ADC ISR
        and does a flash write plus a full menu re-print over UART, stalling
        the 21 kHz control loop for as long as that takes. Do it once, not on
        every sweep -- and re-zeroing mid-session moves the sweep centre to
        wherever the finger happens to be sitting."""
        m.moving = False
        self._send(m.motor_id, DISABLE_CMD); time.sleep(0.05)
        self._send(m.motor_id, ZERO_CMD)
        time.sleep(ZERO_SETTLE)              # flash write + menu printf
        m.zeroed = True
        print(f"motor {m.motor_id} {m.label}: zeroed at current pose")

    def start_sweep(self, m):
        """Sweep a slow sine about the encoder zero. Zeroes first, once."""
        if ZERO_BEFORE_FIRST_SWEEP and not m.zeroed:
            self.zero(m)
        m.moving = False
        self._send(m.motor_id, ENABLE_CMD)
        time.sleep(0.02)
        m.t_start = time.monotonic()
        m.moving = True
        self.armed = True
        print(f"motor {m.motor_id} {m.label}: sweeping +/-{MOTION_AMP:.2f} rad "
              f"over {MOTION_PERIOD:.0f} s (ramp {MOTION_RAMP:.0f} s), "
              f"kp={MOTION_KP} kd={MOTION_KD}")

    def stop_sweep(self, m):
        m.moving = False
        m.p_des = 0.0
        try:
            self._send(m.motor_id, self.idle)      # zero impedance first
            self._send(m.motor_id, DISABLE_CMD)
        except Exception:
            pass
        print(f"motor {m.motor_id} {m.label}: sweep off, disabled")

    def _frame_for(self, m, now):
        if m.moving:
            t = now - m.t_start
            amp = MOTION_AMP * min(1.0, t / MOTION_RAMP)      # ease in
            m.p_des = amp * math.sin(2.0 * math.pi * t / MOTION_PERIOD)
            return pack_motor_command(m.p_des, 0.0, MOTION_KP, MOTION_KD, 0.0)
        if self.armed:
            m.p_des = 0.0
            return self.idle
        return None

    def run(self):
        """Commands are STAGGERED, not sent back-to-back. Every motor on this
        bus replies on the same id (master_id 0), so two replies issued in the
        same arbitration window collide -> bit errors -> dropped commands ->
        the ~47 ms watchdog drops a motor back to MENU. Spacing the commands by
        period/n keeps each reply clear of the next command."""
        period = 1.0 / CMD_RATE_HZ
        slot = period / max(1, len(self.motors))
        nxt = time.monotonic()
        while not self.stop_flag.is_set():
            sent_any = False
            for m in self.motors:
                now = time.monotonic()
                frame = self._frame_for(m, now)
                if frame is not None:
                    try:
                        self._send(m.motor_id, frame)
                        sent_any = True
                    except Exception:
                        pass
                nxt += slot
                time.sleep(max(0.0, nxt - time.monotonic()))
            # one tick per round, so the readout is the PER-MOTOR rate that the
            # ~47 ms firmware watchdog actually cares about, not frames/s
            if sent_any:
                self.meter.tick(time.monotonic())


def main():
    n = max(50, int(HISTORY_SECONDS * 250))
    fts = [FingertipState(tx, rx, lb, n) for tx, rx, lb in FINGERTIPS]
    motors = [MotorState(mid, lb, n) for mid, lb in MOTORS]

    try:
        bus = can.Bus(interface="pcan", fd=True, channel="PCAN_USBBUS1",
                      device_id=BUS_DEVICE_ID, **PCAN_FD_TIMING)
    except Exception as e:
        raise SystemExit(f"[error] could not open thumb bus 0x{BUS_DEVICE_ID:X}: {e}\n"
                         "  list adapters: python -c \"import can; "
                         "print(can.detect_available_configs('pcan'))\"")

    t0 = time.monotonic()
    lock = threading.Lock()
    busload = BusLoad()
    rx = Receiver(bus, fts, motors, t0, busload); rx.start()
    cmd = Commander(bus, motors, lock, busload); cmd.start()
    if ENABLE_ON_START:
        cmd.enable()
    print(f"thumb bus 0x{BUS_DEVICE_ID:X}: motors {[m.motor_id for m in motors]}, "
          f"fingertips {[f.tx_id for f in fts]}")

    # --- figure -------------------------------------------------------------
    fig = plt.figure(figsize=(18, 9.5))
    fig.canvas.manager.set_window_title("KAIST thumb - motors + fingertips")
    gs = fig.add_gridspec(3, 4, width_ratios=(1, 1, 1.05, 1.05),
                          left=0.055, right=0.985, top=0.885, bottom=0.075,
                          wspace=0.34, hspace=0.32)

    ax_prob = fig.add_subplot(gs[0, 0])
    ax_f    = fig.add_subplot(gs[1, 0], sharex=ax_prob)
    ax_tof  = fig.add_subplot(gs[2, 0], sharex=ax_prob)
    ax_mp   = fig.add_subplot(gs[0, 1], sharex=ax_prob)
    ax_mv   = fig.add_subplot(gs[1, 1], sharex=ax_prob)
    ax_mi   = fig.add_subplot(gs[2, 1], sharex=ax_prob)
    ax3d = [fig.add_subplot(gs[0:2, 2], projection="3d"),
            fig.add_subplot(gs[0:2, 3], projection="3d")]
    ax_txt = fig.add_subplot(gs[2, 2:4]); ax_txt.axis("off")

    ax_prob.set_ylabel("contact prob"); ax_prob.set_ylim(-0.05, 1.05)
    ax_f.set_ylabel("|F| [N]")
    ax_tof.set_ylabel("ToF [mm]"); ax_tof.set_xlabel("time [s]")
    ax_mp.set_ylabel("motor pos [rad]")
    ax_mv.set_ylabel("motor vel [rad/s]")
    ax_mi.set_ylabel("motor iq [A]"); ax_mi.set_xlabel("time [s]")
    for ax in (ax_prob, ax_f, ax_tof, ax_mp, ax_mv, ax_mi):
        ax.yaxis.label.set_size(8)
        ax.xaxis.label.set_size(8)
        ax.tick_params(labelsize=7)

    ft_colors = ["#0d6360", "#b06d12"]
    m_colors = ["#3b6ea5", "#9c3f2e"]
    l_prob = [ax_prob.plot([], [], lw=1.2, color=c, label=f.label)[0]
              for f, c in zip(fts, ft_colors)]
    l_fmag = [ax_f.plot([], [], lw=1.2, color=c, label=f.label)[0]
              for f, c in zip(fts, ft_colors)]
    l_tof = [ax_tof.plot([], [], lw=1.0, color=c, ls=ls,
                         label=f"{f.label} r{i+1}")[0]
             for f, c in zip(fts, ft_colors) for i, ls in enumerate(("-", "--"))]
    l_mp = [ax_mp.plot([], [], lw=1.2, color=c, label=m.label)[0]
            for m, c in zip(motors, m_colors)]
    l_mpd = [ax_mp.plot([], [], lw=0.9, ls="--", color=c, alpha=0.6,
                        label=f"{m.label} cmd")[0] for m, c in zip(motors, m_colors)]
    l_mv = [ax_mv.plot([], [], lw=1.2, color=c, label=m.label)[0]
            for m, c in zip(motors, m_colors)]
    l_mi = [ax_mi.plot([], [], lw=1.2, color=c, label=m.label)[0]
            for m, c in zip(motors, m_colors)]
    for ax in (ax_prob, ax_f, ax_tof, ax_mp, ax_mv, ax_mi):
        ax.grid(alpha=0.25, lw=0.5)
        ax.legend(loc="upper left", fontsize=7, ncol=2)

    # superellipsoid, drawn once per fingertip; only marker/trail/arrow move
    eta, omega = np.linspace(-np.pi / 2, np.pi / 2, 24), np.linspace(-np.pi, np.pi, 48)
    E, O = np.meshgrid(eta, omega)
    ce, se = signed_pow(np.cos(E), SE_E1), signed_pow(np.sin(E), SE_E1)
    sx = SE_A * ce * signed_pow(np.cos(O), SE_E2)
    sy = SE_B * ce * signed_pow(np.sin(O), SE_E2)
    sz = SE_C * se
    l_trail, l_pt, l_force = [], [], []
    for ax, f in zip(ax3d, fts):
        ax.plot_wireframe(sx, sy, sz, rstride=2, cstride=2, lw=0.3,
                          color="#8d96a1", alpha=0.5)
        ax.set_xlim(-SE_A, SE_A); ax.set_ylim(-SE_B, SE_B); ax.set_zlim(-SE_C, SE_C)
        ax.set_box_aspect((SE_A, SE_B, SE_C))
        ax.set_title(f"{f.label}  contact + force", fontsize=9, family="monospace")
        ax.tick_params(labelsize=6)
        l_trail.append(ax.plot([], [], [], lw=1.0, color="#b06d12", alpha=0.7)[0])
        l_pt.append(ax.plot([], [], [], "o", ms=8, color="#9c3f2e",
                            mec="white", mew=1.0)[0])
        l_force.append(ax.plot([], [], [], "-o", lw=2.0, ms=4, color="#0d6360",
                               markevery=[-1], zorder=5)[0])

    info = ax_txt.text(0.0, 0.95, "", va="top", ha="left", fontsize=9,
                       family="monospace", transform=ax_txt.transAxes)
    rates_txt = fig.text(0.012, 0.905, "", fontsize=11, family="monospace",
                         fontweight="bold", va="top", color="#0d6360")

    draw_ms = [0.0]

    def update(_):
        t_draw = time.perf_counter()
        now = time.monotonic()
        tmax = 0.0
        for i, f in enumerate(fts):
            t = list(f.t)
            if not t:
                continue
            tmax = max(tmax, t[-1])
            l_prob[i].set_data(t, list(f.prob))
            l_fmag[i].set_data(t, list(f.fmag))
            l_tof[2 * i].set_data(t, list(f.rng[0]))
            l_tof[2 * i + 1].set_data(t, list(f.rng[1]))
            ux, uy, uz = list(f.u[0]), list(f.u[1]), list(f.u[2])
            k = min(len(ux), TRAIL_POINTS)
            l_trail[i].set_data_3d(ux[-k:], uy[-k:], uz[-k:])
            l_pt[i].set_data_3d([ux[-1]], [uy[-1]], [uz[-1]])
            l_pt[i].set_alpha(0.2 + 0.8 * min(1.0, max(0.0, f.prob[-1])))
            fx, fy, fz = f.F[0][-1], f.F[1][-1], f.F[2][-1]
            l_force[i].set_data_3d([ux[-1], ux[-1] + fx * FORCE_SCALE],
                                   [uy[-1], uy[-1] + fy * FORCE_SCALE],
                                   [uz[-1], uz[-1] + fz * FORCE_SCALE])
        for i, m in enumerate(motors):
            t = list(m.t)
            if not t:
                continue
            tmax = max(tmax, t[-1])
            l_mp[i].set_data(t, list(m.p))
            l_mpd[i].set_data(t, list(m.pdes))
            l_mv[i].set_data(t, list(m.v))
            l_mi[i].set_data(t, list(m.iq))

        for ax in (ax_prob, ax_f, ax_tof, ax_mp, ax_mv, ax_mi):
            ax.relim(); ax.autoscale_view(scalex=False)
        ax_prob.set_ylim(-0.05, 1.05)
        ax_prob.set_xlim(max(0.0, tmax - HISTORY_SECONDS), max(HISTORY_SECONDS, tmax))

        armed = "ARMED" if cmd.armed else "idle "
        lines = [f"motors {armed}   kp={MOTOR_KP} kd={MOTOR_KD}   "
                 f"cmd tx {cmd.meter.rate:6.1f} Hz per motor "
                 f"({cmd.meter.count} rounds, {len(motors)} frames each)",
                 "",
                 f"  {'device':14}{'rate':>9}{'age':>9}{'worst gap':>11}"
                 f"{'frames':>9}  state",
                 f"  {'-'*14}{'-'*9:>9}{'-'*8:>9}{'-'*10:>11}{'-'*8:>9}  {'-'*12}"]
        for m in motors:
            age = m.meter.age_ms(now)
            state = "ok" if age < STALE_MS else "NO DATA"
            lines.append(f"  motor {m.motor_id} {m.label:6}{m.meter.rate:7.1f} Hz"
                         f"{age:7.0f} ms{m.meter.max_gap:9.0f} ms{m.meter.count:9d}  {state}"
                         + (f"   p {m.p[-1]:+6.3f} v {m.v[-1]:+6.2f} iq {m.iq[-1]:+5.2f}"
                            if m.p else "")
                         + (f"   SWEEP cmd {m.p_des:+6.3f}" if m.moving else ""))
        lines.append("")
        for f in fts:
            age = f.meter.age_ms(now)
            st = STATUS_NAMES.get(f.status, "?") if age < STALE_MS else "NO DATA"
            lines.append(f"  {f.label} 0x{f.tx_id:02X}     {f.meter.rate:7.1f} Hz"
                         f"{age:7.0f} ms{f.meter.max_gap:9.0f} ms{f.meter.count:9d}  "
                         f"{st}   {f.dropped} dropped")
        lines += ["", f"  gui draw {draw_ms[0]:5.0f} ms   (a worst gap far below the age"
                      " means the GUI stalled, not the sensor)"]
        info.set_text("\n".join(lines))
        busload.sample(now)
        rates_txt.set_text(
            "   ".join([f"{f.label} {f.meter.rate:5.1f} Hz" for f in fts]
                       + [f"{m.label} {m.meter.rate:5.1f} Hz" for m in motors]
                       + [f"cmd {cmd.meter.rate:6.1f} Hz/motor",
                          f"bus {busload.load * 100:4.1f} %"]))
        rates_txt.set_color("#9c3f2e" if busload.load > 0.7 else "#0d6360")
        draw_ms[0] = 0.9 * draw_ms[0] + 0.1 * (time.perf_counter() - t_draw) * 1e3

    sel = {"n": RECAL_DEFAULT}

    def calibrate(ft):
        def _cb(*_):
            n = sel["n"]
            try:
                with lock:
                    bus.send(can.Message(arbitration_id=ft.rx_id,
                                         data=bytes([FT_CMD_CALIBRATE, (n >> 8) & 0xFF, n & 0xFF]),
                                         is_extended_id=False, is_fd=True,
                                         bitrate_switch=True))
                print(f"-> recalibrate {ft.label} (rx {ft.rx_id}, {n} samples)")
            except Exception as e:
                print(f"send failed: {e}")
        return _cb

    _anim = FuncAnimation(fig, update, interval=int(REDRAW_PERIOD * 1000),
                          cache_frame_data=False)

    # buttons last so tight-layout-ish adjustments don't move them; keep refs
    btns = []

    def toggle_sweep(m, holder):
        def _cb(*_):
            if m.moving:
                cmd.stop_sweep(m)
                holder[0].label.set_text(f"sweep {m.label}")
                holder[0].ax.set_facecolor("#e6dcc6")
            else:
                cmd.start_sweep(m)
                holder[0].label.set_text(f"STOP {m.label}")
                holder[0].ax.set_facecolor("#e2a95f")
            fig.canvas.draw_idle()
        return _cb

    def rezero(*_):
        for m in motors:
            m.zeroed = False
        print("motors will re-zero on the next sweep start")

    specs = [("Recal " + fts[0].label, calibrate(fts[0]), "#dfe4ea"),
             ("Recal " + fts[1].label, calibrate(fts[1]), "#dfe4ea"),
             ("Enable all", lambda *_: cmd.enable(), "#cfe3e2"),
             ("Disable all", lambda *_: cmd.disable(), "#f0d6d0"),
             ("Re-zero", rezero, "#e6dcc6")]
    for i, (label, cb, colour) in enumerate(specs):
        ax_b = fig.add_axes((0.225 + i * 0.086, 0.955, 0.080, 0.032))
        b = Button(ax_b, label, color=colour, hovercolor="#b8c2cc")
        b.label.set_fontsize(8); b.label.set_family("monospace")
        b.on_clicked(cb)
        btns.append(b)

    for i, m in enumerate(motors):
        ax_b = fig.add_axes((0.672 + i * 0.098, 0.955, 0.091, 0.032))
        holder = [None]
        b = Button(ax_b, f"sweep {m.label}", color="#e6dcc6", hovercolor="#d8c9a6")
        b.label.set_fontsize(8); b.label.set_family("monospace")
        holder[0] = b
        b.on_clicked(toggle_sweep(m, holder))
        btns.append(b)

    rad_ax = fig.add_axes((0.895, 0.895, 0.098, 0.085))
    rad_ax.set_title("cal samples", fontsize=7, family="monospace")
    _rad = RadioButtons(rad_ax, [str(c) for c in RECAL_CHOICES],
                        active=RECAL_CHOICES.index(RECAL_DEFAULT))
    for lb in _rad.labels:
        lb.set_fontsize(7); lb.set_family("monospace")
    _rad.on_clicked(lambda label: sel.__setitem__("n", int(label)))
    btns.append(_rad)

    fig.text(0.012, 0.972, "THUMB BUS 0xFFFFFFFB", fontsize=12, family="monospace",
             fontweight="bold", va="top", color="#15181d")
    fig.text(0.012, 0.940,
             f"idle: kp=kd=0 (free)    sweep: kp={MOTION_KP} kd={MOTION_KD}, "
             f"+/-{MOTION_AMP:.2f} rad over {MOTION_PERIOD:.0f} s about the ZEROED pose",
             fontsize=8, family="monospace", va="top", color="#9c3f2e")

    try:
        plt.show()
    finally:
        cmd.disable()
        cmd.stop_flag.set(); rx.stop_flag.set()
        cmd.join(timeout=1.0); rx.join(timeout=1.0)
        bus.shutdown()
        for m in motors:
            print(f"motor {m.motor_id} {m.label}: {m.meter.count} replies, "
                  f"worst gap {m.meter.max_gap:.0f} ms")
        for f in fts:
            print(f"{f.label}: {f.meter.count} frames, {f.dropped} dropped, "
                  f"worst gap {f.meter.max_gap:.0f} ms")


if __name__ == "__main__":
    main()
