#!/usr/bin/env python3
"""Unpack CAN frames from the KAIST fingertip firmware.

Uses raw SocketCAN (no external library needed, same approach as
kaist_demo/templates/motor_test_fdcan.py). The fingertip firmware sends
CAN-FD frames with bit-rate switching (FDCAN_FRAME_FD_BRS): 1 Mbit/s
arbitration phase, 2 Mbit/s data phase. This reads the 72-byte canfd_frame.

Mirrors the packing in Fingertip_KAIST/Core/Src/fingertip.cpp:
  - pack_pressure_reply(): 8 pressure channels (int32, signed, big-endian),
    two per frame across IDs PR_1..PR_4.
  - pack_tof_reply(): range[0..2] as single bytes (uint8) in PR_TOF.

CAN IDs depend on SELECTED_FINGER in the firmware (default 3):
    base   = (finger - 1) * 4
    PR_1   = base + 0   pressure[0], pressure[1]
    PR_2   = base + 1   pressure[2], pressure[3]
    PR_3   = base + 2   pressure[4], pressure[5]
    PR_4   = base + 3   pressure[6], pressure[7]
    PR_TOF = base + 4   range[0..2]
    PR_IMU = base + 5   (declared but not transmitted by current firmware)

Linux setup (run once before this script; matches firmware FD-BRS timing):
    sudo ip link set can0 down
    sudo ip link set can0 up type can bitrate 1000000 dbitrate 2000000 fd on

Usage:
    python3 can_unpack.py                          # finger 3, can0
    python3 can_unpack.py --finger 1 --channel can0
"""

import argparse
import socket
import struct

# CAN-FD with bit-rate switching (firmware uses FDCAN_FRAME_FD_BRS).
# struct canfd_frame { canid_t can_id; __u8 len; __u8 flags; __u8 __res0;
#                      __u8 __res1; __u8 data[64]; } -> 4 + 1 + 1 + 2 pad + 64 = 72 bytes
CANFD_FRAME_FMT = "=IBB2x64s"
CANFD_FRAME_SIZE = 72
CAN_EFF_FLAG = 0x80000000
CAN_SFF_MASK = 0x000007FF

# Linux SocketCAN options to enable FD frames on the raw socket
SOL_CAN_RAW = 101            # socket.SOL_CAN_RAW
CAN_RAW_FD_FRAMES = 5        # socket.CAN_RAW_FD_FRAMES


def build_id_map(finger):
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


def main():
    ap = argparse.ArgumentParser(description="Unpack KAIST fingertip CAN frames")
    ap.add_argument("--channel", default="can0", help="SocketCAN interface (default can0)")
    ap.add_argument("--finger", type=int, default=3, help="SELECTED_FINGER value (default 3)")
    args = ap.parse_args()

    id_map = build_id_map(args.finger)
    pressure = [None] * 8
    tof = [None] * 3
    seen_pressure = set()  # which PR_n frames arrived in the current cycle

    try:
        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.bind((args.channel,))
        sock.setsockopt(SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1)  # enable FD frames
        print(f"[Success] Bound to {args.channel} (CAN-FD)")
    except Exception as e:
        print(f"[Error] Could not bind to {args.channel}: {e}")
        print("Did you run:")
        print(f"  sudo ip link set {args.channel} up type can bitrate 1000000 dbitrate 2000000 fd on")
        raise SystemExit(1)

    print(f"Listening, finger {args.finger}. IDs: "
          + ", ".join(f"{name}=0x{cid:X}({cid})" for cid, (name, *_) in id_map.items()))

    try:
        while True:
            frame = sock.recv(CANFD_FRAME_SIZE)
            can_id, dlc, flags, data = struct.unpack(CANFD_FRAME_FMT, frame)
            can_id &= CAN_SFF_MASK  # strip EFF/RTR/ERR flag bits -> 11-bit standard ID

            entry = id_map.get(can_id)
            if entry is None:
                continue
            name, kind, channels = entry
            payload = data[:dlc]

            if kind == "pressure":
                for idx, val in decode_pressure(payload, channels).items():
                    pressure[idx] = val
                seen_pressure.add(name)
                # print only once all 4 frames (all 8 channels) have arrived
                if seen_pressure >= {"PR_1", "PR_2", "PR_3", "PR_4"}:
                    print(f"pressures: {pressure}")
                    seen_pressure.clear()
            elif kind == "tof":
                for idx, val in decode_tof(payload).items():
                    tof[idx] = val
                print(f"[{name:6}] tof(mm) {tof}")
            elif kind == "imu":
                print(f"[{name:6}] imu raw {list(payload)}")
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()


if __name__ == "__main__":
    main()
