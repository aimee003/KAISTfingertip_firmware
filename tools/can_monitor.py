#!/usr/bin/env python3
"""Live plot of the KAIST fingertip's CAN FD reply frames.

Receives on a background thread and redraws at a fixed slow interval, so
plotting never throttles reception.

Frame layout (Fingertip_KAIST/Core/Src/fdcan.cpp, pack_all_reply):
    32-byte CAN FD payload on FINGERTIP_SENSOR_TX_ID,
    big-endian signed int16 throughout.

    byte   field              scale  unit
    0      status             -      0 OK / 1 WARMUP / 2 CALIBRATING
    1      seq                -      rolling, gaps mean dropped frames
    2- 3   contact prob       1000   0..1
    4- 9   Fx, Fy, Fz         100    N
    10-15  ux, uy, uz         100    mm
    16-19  ToF range 1, 2     1      mm
    20-25  roll, pitch, yaw   1      deg
    26-31  reserved

This machine runs PEAK's chardev driver (/dev/pcanusbfd*), not SocketCAN, so
the default interface is 'pcan'. python-can lives in the manip_env conda env:

    ~/miniconda3/envs/manip_env/bin/python tools/can_monitor.py

Click the Recalibrate button, or press 'r', to trigger a baseline recalibration.
"""

import argparse
import struct
import threading
import time
from collections import deque

import can
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, RadioButtons

# --- config: edit here, or override with the matching command-line flag ----
TX_ID           = 24            # --tx-id      FINGERTIP_SENSOR_TX_ID in fdcan.h (0x17)
INTERFACE       = "pcan"        # --interface  "pcan" (chardev) or "socketcan"
CHANNEL         = None          # --channel    None -> pin by DEVICE_ID
DEVICE_ID       = 0xFFFFFFFC    # --device-id  pcan only; channel numbers are unstable
HISTORY_SECONDS = 10.0          # --seconds    plotted time window
REDRAW_PERIOD   = 0.05          # --redraw     seconds between redraws
TRAIL_POINTS    = 100           #              contact-position trail length
FORCE_SCALE     = 1.0           #              force arrow length, mm per N

# --- frame format ---------------------------------------------------------
BODY_FMT = ">12h"          # bytes 2..25, twelve big-endian int16
FINGERTIP_SENSOR_RX_ID = 42         # host -> sensor, must match fdcan.h (0x20)
FT_CMD_CALIBRATE = 0x0B
RECAL_CHOICES    = (250, 500, 1000, 2000)   # selectable sample counts
RECAL_DEFAULT    = 1000

# Fingertip surface, from NN_SE_* in Fingertip_KAIST/Core/Inc/kaist_net.h.
# nn.u[] is the contact point on this superellipsoid, in mm.
SE_A, SE_B, SE_C = 15.5, 12.0, 5.5
SE_E1, SE_E2 = 1.0, 0.5


def signed_pow(v, e):
    """|v|**e keeping v's sign -- the superellipsoid parametrisation needs it."""
    return np.sign(v) * (np.abs(v) ** e)


STATUS_NAMES = {0: "OK", 1: "WARMUP", 2: "CALIBRATING"}
STATUS_COLORS = {0: "#0d6360", 1: "#b06d12", 2: "#9c3f2e"}

# PCAN FD timing at f_clock 80 MHz -- 1 Mbit/s nominal, 2 Mbit/s data, 80% SP.
# Matches the firmware and the Manipulator-Software PCAN config.
PCAN_FD_TIMING = dict(
    f_clock_mhz=80,
    nom_brp=1, nom_tseg1=63, nom_tseg2=16, nom_sjw=16,
    data_brp=1, data_tseg1=31, data_tseg2=8, data_sjw=8,
)


def decode(payload):
    """32-byte payload -> dict, or None if it isn't a full frame."""
    if len(payload) < 26:
        return None
    v = struct.unpack(BODY_FMT, bytes(payload[2:26]))
    return {
        "status": payload[0],
        "seq": payload[1],
        "prob": v[0] / 1000.0,
        "F": (v[1] / 100.0, v[2] / 100.0, v[3] / 100.0),
        "u": (v[4] / 100.0, v[5] / 100.0, v[6] / 100.0),
        "range": (v[7], v[8]),
        "rpy": (v[9], v[10], v[11]),
    }


class Receiver(threading.Thread):
    """Drains the bus as fast as it arrives; plotting reads the deques."""

    daemon = True

    def __init__(self, bus, msg_id, history):
        super().__init__()
        self.bus = bus
        self.msg_id = msg_id
        self.stop_flag = threading.Event()

        self.t = deque(maxlen=history)
        self.prob = deque(maxlen=history)
        self.F = [deque(maxlen=history) for _ in range(3)]
        self.u = [deque(maxlen=history) for _ in range(3)]
        self.rng = [deque(maxlen=history) for _ in range(2)]

        self.status = None
        self.frames = 0
        self.dropped = 0
        self.last_seq = None
        self.last_rx = None
        self.t0 = time.monotonic()
        self._rate_mark = (0.0, 0)   # (relative time, frame count) -- `now` is t0-relative
        self.rate = 0.0

    def run(self):
        while not self.stop_flag.is_set():
            msg = self.bus.recv(timeout=0.2)
            if msg is None or msg.arbitration_id != self.msg_id:
                continue
            s = decode(msg.data)
            if s is None:
                continue

            # seq is a rolling byte; any step other than +1 means loss
            if self.last_seq is not None:
                gap = (s["seq"] - self.last_seq) & 0xFF
                if gap != 1:
                    self.dropped += gap - 1
            self.last_seq = s["seq"]

            now = time.monotonic() - self.t0
            self.t.append(now)
            self.prob.append(s["prob"])
            for i in range(3):
                self.F[i].append(s["F"][i])
                self.u[i].append(s["u"][i])
            for i in range(2):
                self.rng[i].append(s["range"][i])
            self.status = s["status"]
            self.last_rx = time.monotonic()
            self.frames += 1

            # frame rate over a ~1 s window
            elapsed = now - self._rate_mark[0]
            if elapsed >= 1.0:
                self.rate = (self.frames - self._rate_mark[1]) / elapsed
                self._rate_mark = (now, self.frames)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    hexint = lambda x: int(x, 0)
    ap.add_argument("--tx-id", type=hexint, default=TX_ID,
                    help=f"FINGERTIP_SENSOR_TX_ID from fdcan.h (default 0x{TX_ID:X})")
    ap.add_argument("--interface", default=INTERFACE, choices=["pcan", "socketcan"],
                    help=f"python-can interface (default {INTERFACE})")
    ap.add_argument("--channel", default=CHANNEL,
                    help="socketcan: can0 | pcan: PCAN_USBBUS1 (omit to pin by --device-id)")
    ap.add_argument("--device-id", type=hexint, default=DEVICE_ID,
                    help=f"pcan only: pin the adapter by device id (default 0x{DEVICE_ID:X})")
    ap.add_argument("--seconds", type=float, default=HISTORY_SECONDS,
                    help=f"history window in s (default {HISTORY_SECONDS})")
    ap.add_argument("--redraw", type=float, default=REDRAW_PERIOD,
                    help=f"redraw period in s (default {REDRAW_PERIOD})")
    args = ap.parse_args()

    msg_id = args.tx_id
    history = max(50, int(args.seconds * 250))   # 250 Hz headroom over the 200 Hz stream

    kwargs = dict(interface=args.interface, fd=True)
    if args.interface == "pcan":
        kwargs.update(PCAN_FD_TIMING)
        if args.channel:
            kwargs["channel"] = args.channel
        else:
            kwargs["device_id"] = args.device_id
    else:
        kwargs["channel"] = args.channel or "can0"

    try:
        bus = can.Bus(**kwargs)
    except Exception as e:
        raise SystemExit(
            f"[error] could not open {args.interface}: {e}\n"
            "  pcan     : needs the chardev driver -- check /dev/pcanusbfd* exists\n"
            "  socketcan: sudo ip link set can0 up type can "
            "bitrate 1000000 dbitrate 2000000 fd on"
        )

    rx = Receiver(bus, msg_id, history)
    rx.start()
    print(f"listening on {args.interface} for ID 0x{msg_id:03X}")

    # --- figure -----------------------------------------------------------
    fig = plt.figure(figsize=(15, 6.5))
    fig.canvas.manager.set_window_title(f"fingertip - CAN 0x{msg_id:03X}")
    gs = fig.add_gridspec(2, 3, width_ratios=(1, 1, 1.15))
    ax_p = fig.add_subplot(gs[0, 0])
    ax_f = fig.add_subplot(gs[1, 0], sharex=ax_p)
    ax_u = fig.add_subplot(gs[0, 1], sharex=ax_p)
    ax_r = fig.add_subplot(gs[1, 1], sharex=ax_p)
    ax_3d = fig.add_subplot(gs[:, 2], projection="3d")
    axes = np.array([[ax_p, ax_f], [ax_u, ax_r]])

    ax_p.set_ylabel("contact prob")
    ax_p.set_ylim(-0.05, 1.05)
    ax_f.set_ylabel("force [N]")
    ax_u.set_ylabel("position [mm]")
    ax_r.set_ylabel("ToF range [mm]")
    for ax in (ax_u, ax_r):
        ax.set_xlabel("time [s]")

    l_prob, = ax_p.plot([], [], lw=1.4, color="#0d6360")
    l_F = [ax_f.plot([], [], lw=1.2, label=n)[0] for n in ("Fx", "Fy", "Fz")]
    l_u = [ax_u.plot([], [], lw=1.2, label=n)[0] for n in ("ux", "uy", "uz")]
    l_r = [ax_r.plot([], [], lw=1.2, label=n)[0] for n in ("range 1", "range 2")]
    for ax in (ax_f, ax_u, ax_r):
        ax.legend(loc="upper left", fontsize=8, ncol=3)
    for ax in axes.flat:
        ax.grid(alpha=0.25, lw=0.5)

    # --- superellipsoid: the surface nn.u[] is reported on -----------------
    # Static wireframe drawn once; only the marker and trail move each frame.
    eta = np.linspace(-np.pi / 2, np.pi / 2, 24)
    omega = np.linspace(-np.pi, np.pi, 48)
    E, O = np.meshgrid(eta, omega)
    ce, se = signed_pow(np.cos(E), SE_E1), signed_pow(np.sin(E), SE_E1)
    ax_3d.plot_wireframe(SE_A * ce * signed_pow(np.cos(O), SE_E2),
                         SE_B * ce * signed_pow(np.sin(O), SE_E2),
                         SE_C * se,
                         rstride=2, cstride=2, lw=0.35, color="#8d96a1", alpha=0.5)
    ax_3d.set_xlim(-SE_A, SE_A); ax_3d.set_ylim(-SE_B, SE_B); ax_3d.set_zlim(-SE_C, SE_C)
    ax_3d.set_box_aspect((SE_A, SE_B, SE_C))       # true proportions, not a cube
    ax_3d.set_xlabel("x [mm]", fontsize=8)
    ax_3d.set_ylabel("y [mm]", fontsize=8)
    ax_3d.set_zlabel("z [mm]", fontsize=8)
    ax_3d.tick_params(labelsize=7)
    ax_3d.set_title(f"contact position   a={SE_A} b={SE_B} c={SE_C} "
                    f"e1={SE_E1} e2={SE_E2}", fontsize=9, family="monospace")
    l_trail, = ax_3d.plot([], [], [], lw=1.0, color="#b06d12", alpha=0.7)
    l_pt, = ax_3d.plot([], [], [], "o", ms=9, color="#9c3f2e", mec="white", mew=1.0)
    # force vector drawn from the contact point; tip marker stands in for an arrowhead
    l_force, = ax_3d.plot([], [], [], "-o", lw=2.0, ms=4, color="#0d6360",
                          markevery=[-1], zorder=5)
    force_txt = ax_3d.text2D(0.02, 0.02, "", transform=ax_3d.transAxes,
                             fontsize=9, family="monospace", color="#0d6360")

    status_txt = fig.text(0.012, 0.965, " WAITING ", fontsize=13, family="monospace",
                          fontweight="bold", va="top", ha="left", color="white",
                          bbox=dict(boxstyle="round,pad=0.35", facecolor="#888888", edgecolor="none"))
    metrics_txt = fig.text(0.20, 0.958, "", fontsize=10, family="monospace",
                           va="top", ha="left", color="#555555")

    def update(_):
        t = list(rx.t)
        if not t:
            return
        l_prob.set_data(t, list(rx.prob))
        for i in range(3):
            l_F[i].set_data(t, list(rx.F[i]))
            l_u[i].set_data(t, list(rx.u[i]))
        for i in range(2):
            l_r[i].set_data(t, list(rx.rng[i]))

        ux, uy, uz = list(rx.u[0]), list(rx.u[1]), list(rx.u[2])
        if ux:
            n = min(len(ux), TRAIL_POINTS)
            l_trail.set_data_3d(ux[-n:], uy[-n:], uz[-n:])
            l_pt.set_data_3d([ux[-1]], [uy[-1]], [uz[-1]])
            l_pt.set_alpha(0.2 + 0.8 * min(1.0, max(0.0, rx.prob[-1])))

            # force vector from the contact point, in the same frame as u
            fx, fy, fz = rx.F[0][-1], rx.F[1][-1], rx.F[2][-1]
            l_force.set_data_3d([ux[-1], ux[-1] + fx * FORCE_SCALE],
                                [uy[-1], uy[-1] + fy * FORCE_SCALE],
                                [uz[-1], uz[-1] + fz * FORCE_SCALE])
            mag = (fx * fx + fy * fy + fz * fz) ** 0.5
            force_txt.set_text(f"|F| {mag:5.2f} N   ({fx:+.2f}, {fy:+.2f}, {fz:+.2f})")

        for ax in axes.flat:
            ax.relim()
            ax.autoscale_view(scalex=False)
        ax_p.set_ylim(-0.05, 1.05)
        axes.flat[0].set_xlim(max(0.0, t[-1] - args.seconds), max(args.seconds, t[-1]))

        # stale if the stream stops -- otherwise the last status would look current
        age = time.monotonic() - rx.last_rx if rx.last_rx else 999.0
        if age > 0.5:
            name, color = "NO DATA", "#9c3f2e"
        else:
            name = STATUS_NAMES.get(rx.status, f"UNKNOWN {rx.status}")
            color = STATUS_COLORS.get(rx.status, "#888888")
        status_txt.set_text(f" {name} ")
        status_txt.get_bbox_patch().set_facecolor(color)
        metrics_txt.set_text(
            f"{rx.rate:5.1f} Hz   {rx.frames} frames   {rx.dropped} dropped"
            f"   seq {rx.last_seq if rx.last_seq is not None else '--'}"
            f"   last {age*1000:.0f} ms ago")

    sel = {"n": RECAL_DEFAULT}

    def send_calibrate(*_):
        n = sel["n"]
        try:
            bus.send(can.Message(arbitration_id=FINGERTIP_SENSOR_RX_ID,
                                 data=bytes([FT_CMD_CALIBRATE, (n >> 8) & 0xFF, n & 0xFF]),
                                 is_extended_id=False, is_fd=True, bitrate_switch=True))
            print(f"-> recalibrate sent ({n} samples)")
        except Exception as e:
            print(f"send failed: {e}")

    def on_key(event):
        if event.key in ("r", "c"):
            send_calibrate()

    fig.canvas.mpl_connect("key_press_event", on_key)

    # cache_frame_data=False: this is a live stream, not a fixed-length animation
    _anim = FuncAnimation(fig, update, interval=int(args.redraw * 1000),
                          cache_frame_data=False)
    plt.tight_layout(rect=(0, 0, 1, 0.93))   # leave room for the status banner

    # added after tight_layout so it doesn't get repositioned; keep a reference
    # to _btn or matplotlib garbage-collects the callback
    btn_ax = fig.add_axes((0.845, 0.945, 0.145, 0.045))
    _btn = Button(btn_ax, "Recalibrate  (r)", color="#dfe4ea", hovercolor="#c3ccd6")
    _btn.label.set_fontsize(9)
    _btn.label.set_family("monospace")
    _btn.on_clicked(send_calibrate)

    rad_ax = fig.add_axes((0.845, 0.775, 0.145, 0.16))
    rad_ax.set_title("samples", fontsize=8, family="monospace")
    _rad = RadioButtons(rad_ax, [str(c) for c in RECAL_CHOICES],
                        active=RECAL_CHOICES.index(RECAL_DEFAULT))
    for lb in _rad.labels:
        lb.set_fontsize(8); lb.set_family("monospace")
    _rad.on_clicked(lambda label: sel.__setitem__("n", int(label)))
    try:
        plt.show()
    finally:
        rx.stop_flag.set()
        rx.join(timeout=1.0)
        bus.shutdown()
        print(f"\n{rx.frames} frames, {rx.dropped} dropped")


if __name__ == "__main__":
    main()
