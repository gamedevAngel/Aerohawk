#!/usr/bin/env python3
"""
Quick health check for an RPLIDAR C1 on a Jetson / Raspberry Pi.

Usage:
    pip install pyserial
    python3 lidar_check.py                 # defaults to /dev/ttyUSB0
    python3 lidar_check.py /dev/ttyUSB1    # if it enumerated elsewhere

What it checks:
    1. Serial port opens
    2. Device info (model, firmware, serial number)
    3. Health status (Good / Warning / Error)
    4. Spins up, grabs a few full 360° scans, reports points/rev,
       scan rate, and closest/farthest readings
"""
import struct
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
BAUD = 460800          # RPLIDAR C1 default (A1 = 115200, A2/A3/S1 = 256000+)
NUM_SCANS = 5

SYNC = 0xA5
CMD_STOP, CMD_RESET, CMD_SCAN = 0x25, 0x40, 0x20
CMD_INFO, CMD_HEALTH = 0x50, 0x52


def send(ser, cmd):
    ser.write(bytes([SYNC, cmd]))


def read_descriptor(ser):
    d = ser.read(7)
    if len(d) != 7 or d[0] != 0xA5 or d[1] != 0x5A:
        raise RuntimeError(f"Bad/no response descriptor: {d.hex(' ') or 'nothing'}")
    length = struct.unpack("<I", d[2:6])[0] & 0x3FFFFFFF
    return length, d[6]


def get_info(ser):
    send(ser, CMD_INFO)
    length, _ = read_descriptor(ser)
    r = ser.read(length)
    model, fw_minor, fw_major, hw = r[0], r[1], r[2], r[3]
    serial_no = r[4:20].hex().upper()
    return model, f"{fw_major}.{fw_minor:02d}", hw, serial_no


def get_health(ser):
    send(ser, CMD_HEALTH)
    length, _ = read_descriptor(ser)
    r = ser.read(length)
    status, err = r[0], struct.unpack("<H", r[1:3])[0]
    return {0: "Good", 1: "Warning", 2: "Error"}.get(status, "?"), err


def scan(ser, num_scans):
    send(ser, CMD_SCAN)
    read_descriptor(ser)

    scans, current, bad = [], [], 0
    t_start = None
    while len(scans) < num_scans:
        node = ser.read(5)
        if len(node) < 5:
            raise RuntimeError("Timed out mid-scan — check cable/power")
        b0, b1 = node[0], node[1]
        start = b0 & 0x1
        inv_start = (b0 >> 1) & 0x1
        if start == inv_start or not (b1 & 0x1):
            bad += 1                       # out of sync: slide 1 byte
            ser.read(1)
            continue
        quality = b0 >> 2
        angle = ((node[2] << 7) | (b1 >> 1)) / 64.0
        dist_mm = struct.unpack("<H", node[3:5])[0] / 4.0

        if start:
            if current and t_start is not None:
                scans.append(current)
            if t_start is None:
                t_start = time.time()      # ignore the first partial rev
            current = []
        if t_start is not None:
            current.append((angle, dist_mm, quality))

    elapsed = time.time() - t_start
    return scans, elapsed, bad


def main():
    print(f"Opening {PORT} @ {BAUD}...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=2)
    except serial.SerialException as e:
        print(f"FAIL: can't open port: {e}")
        print("  - ls /dev/ttyUSB*   (is it there?)")
        print("  - sudo usermod -aG dialout $USER  then log out/in")
        sys.exit(1)

    ser.dtr = False
    send(ser, CMD_STOP)
    time.sleep(0.1)
    ser.reset_input_buffer()

    try:
        model, fw, hw, sn = get_info(ser)
        print(f"OK   Info:   model=0x{model:02X} fw={fw} hw={hw} serial={sn}")

        status, err = get_health(ser)
        print(f"{'OK  ' if status == 'Good' else 'WARN'} Health: {status} (error code {err})")

        print(f"Spinning up and reading {NUM_SCANS} scans...")
        scans, elapsed, bad = scan(ser, NUM_SCANS)
    except RuntimeError as e:
        print(f"FAIL: {e}")
        print("  - Wrong baud? C1=460800. Try power-cycling the LiDAR.")
        sys.exit(1)
    finally:
        send(ser, CMD_STOP)
        time.sleep(0.05)
        ser.close()

    pts = [len(s) for s in scans]
    valid = [d for s in scans for (_, d, q) in s if d > 0 and q > 0]
    hz = len(scans) / elapsed if elapsed else 0

    print(f"OK   Scan rate:     {hz:.1f} Hz")
    print(f"OK   Points/rev:    {min(pts)}–{max(pts)} (avg {sum(pts)/len(pts):.0f})")
    print(f"     Valid returns: {len(valid)}/{sum(pts)} "
          f"({100*len(valid)/max(sum(pts),1):.0f}%)")
    if valid:
        print(f"     Closest:       {min(valid):.0f} mm")
        print(f"     Farthest:      {max(valid):.0f} mm")
    print(f"     Sync errors:   {bad}")

    # Quick sanity: what's directly in front (±5° of 0°)?
    front = [d for (a, d, q) in scans[-1] if (a < 5 or a > 355) and d > 0]
    if front:
        print(f"     Front (0°):    ~{sum(front)/len(front):.0f} mm")

    ok = hz > 5 and len(valid) > 0 and bad < 50
    print("\nPASS — LiDAR looks healthy." if ok else
          "\nCHECK — data came through but something looks off (see above).")


if __name__ == "__main__":
    main()