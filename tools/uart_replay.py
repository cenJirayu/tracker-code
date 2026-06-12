#!/usr/bin/env python3
# ============================================================================
# uart_replay.py — replay flight_data.csv as GroundLinkPacket frames over UART
# Cursr-V Antenna Tracker · full mission rehearsal without rocket or radio
#
# Streams the recorded flight into the tracker's mission firmware exactly the
# way the ground Heltec would: 28-byte GroundLinkPacket frames at LINK_RATE_HZ
# through a USB-UART adapter wired to tracker GPIO 44 (RX) + GND.
#
# The replay first holds the rocket's launch-pad position for --pad-hold
# seconds so the firmware can walk WAIT_LINK → PAD_LOCK → ARMED (motors
# de-energised), then plays the flight in real time — the firmware must detect
# the launch and start tracking on its own.
#
# By default the synthetic WGS84 fixes are built around the tracker's default
# base station (config.h HIL_DEFAULT_BASE_*, alt 10 m), so the rehearsal works
# with no set_base step. Pass --base-lat/--base-lon/--base-alt to match a real
# base if the tracker has a GNSS fix or a manual base.
#
#   python tools/uart_replay.py COM7
#   python tools/uart_replay.py COM7 --rate 10 --pad-hold 12 --noise 3
# ============================================================================
import argparse
import math
import random
import struct
import sys
import time

from convert_flight import load_rows, OFFSET_E, OFFSET_N, GROUND_Z, M_PER_DEG

# Tracker defaults (config.h HIL_DEFAULT_BASE_*) — keep in sync.
DEF_BASE_LAT = 13.7563
DEF_BASE_LON = 100.5018
DEF_BASE_ALT = 10.0

PKT_FMT = "<BHddfhbBB"          # GroundLinkPacket — keep in sync with TelemetryPacket.h
assert struct.calcsize(PKT_FMT) == 28


def build_packet(seq, lat, lon, alt, rssi=-70, snr_db=9.0):
    raw = bytearray(struct.pack(PKT_FMT, 0xAA, seq & 0xFFFF, lat, lon, alt,
                                rssi, int(round(snr_db * 4)), 0, 0))
    cs = 0
    for b in raw[1:-1]:
        cs ^= b
    raw[-1] = cs
    return bytes(raw)


def sample_enu(rows, t):
    """Interpolate the CSV (X,Y,Z) at time t → local ENU metres vs tracker."""
    if t <= rows[0][0]:
        _, x, y, z = rows[0]
    elif t >= rows[-1][0]:
        _, x, y, z = rows[-1]
    else:
        lo, hi = 0, len(rows) - 1
        while hi - lo > 1:                      # bisect the time bracket
            mid = (lo + hi) // 2
            if rows[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        t0, x0, y0, z0 = rows[lo]
        t1, x1, y1, z1 = rows[hi]
        f = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
        x = x0 + (x1 - x0) * f
        y = y0 + (y1 - y0) * f
        z = z0 + (z1 - z0) * f
    return OFFSET_E + x, OFFSET_N + y, z - GROUND_Z


def main():
    ap = argparse.ArgumentParser(description="Replay flight_data.csv as GroundLinkPacket frames")
    ap.add_argument("port", help="serial port of the USB-UART adapter (e.g. COM7)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--rate", type=float, default=10.0,
                    help="packet rate Hz — must match LINK_RATE_HZ in config.h (default 10)")
    ap.add_argument("--pad-hold", type=float, default=12.0,
                    help="seconds to hold the pad fix before launch (PAD_LOCK + ARM settle)")
    ap.add_argument("--noise", type=float, default=0.0,
                    help="1-sigma gaussian position noise in metres (default 0)")
    ap.add_argument("--base-lat", type=float, default=DEF_BASE_LAT)
    ap.add_argument("--base-lon", type=float, default=DEF_BASE_LON)
    ap.add_argument("--base-alt", type=float, default=DEF_BASE_ALT,
                    help="tracker base altitude the fixes are built around (default 10 = config.h default)")
    args = ap.parse_args()

    try:
        import serial
    except ImportError:
        sys.exit("pyserial is required:  pip install pyserial")

    def to_wgs84(e, n, u):
        lat = args.base_lat + n / M_PER_DEG
        lon = args.base_lon + e / (M_PER_DEG * math.cos(math.radians(args.base_lat)))
        return lat, lon, args.base_alt + u

    rows = load_rows()
    flight_len = rows[-1][0]
    period = 1.0 / args.rate

    with serial.Serial(args.port, args.baud) as ser:
        print(f"Streaming to {args.port} @ {args.baud}: pad hold {args.pad_hold:.0f}s, "
              f"flight {flight_len:.0f}s, {args.rate:.0f} Hz, noise {args.noise} m")
        seq = 0
        start = time.monotonic()
        phase = "pad"
        while True:
            now = time.monotonic() - start
            t = 0.0 if now < args.pad_hold else now - args.pad_hold
            if phase == "pad" and t > 0.0:
                phase = "flight"
                print("LAUNCH — flight replay running")
            if t > flight_len + 5.0:
                break

            e, n, u = sample_enu(rows, min(t, flight_len))
            if args.noise > 0.0:
                e += random.gauss(0, args.noise)
                n += random.gauss(0, args.noise)
                u += random.gauss(0, args.noise)
            lat, lon, alt = to_wgs84(e, n, u)

            ser.write(build_packet(seq, lat, lon, alt))
            seq += 1
            if seq % int(args.rate * 5) == 0:
                print(f"  t={t:7.1f}s  e={e:8.1f} n={n:8.1f} u={u:8.1f}  ({phase})")

            next_at = start + (seq * period)
            sleep = next_at - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)

    print(f"Done — {seq} packets sent.")


if __name__ == "__main__":
    main()
