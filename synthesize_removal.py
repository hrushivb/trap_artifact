#!/usr/bin/env python3
"""
LiDAR Removal Attack on PCD files — preserves exact input format.
Usage: python removal_attack.py <input.pcd> <start_angle> <end_angle> [output.pcd]

Coordinate system: Front=0°, Right=+90°, Left=-90°, Back=±180°

Examples:
  python removal_attack.py frame_0.pcd -1 1
  python removal_attack.py frame_0.pcd 30 90 right_blind.pcd
  python removal_attack.py frame_0.pcd 170 -170 rear.pcd
"""

import sys
import numpy as np


def removal_attack(input_path, start_angle, end_angle, output_path):
    # ── Detect line ending ────────────────────────────────────────────────────
    with open(input_path, "rb") as f:
        raw = f.read()

    line_ending = b"\r\n" if b"\r\n" in raw else b"\n"
    lines = raw.split(line_ending)

    # ── Split header from data ────────────────────────────────────────────────
    header_lines = []
    data_start_idx = 0
    for i, line in enumerate(lines):
        header_lines.append(line)
        if line.strip().lower().startswith(b"data"):
            data_start_idx = i + 1
            break

    data_lines = [l for l in lines[data_start_idx:] if l.strip()]

    # ── Apply removal attack ──────────────────────────────────────────────────
    start = ((start_angle + 180) % 360) - 180
    end   = ((end_angle   + 180) % 360) - 180

    kept = []
    removed_count = 0

    for line in data_lines:
        parts = line.split()
        x = float(parts[0])   # lateral
        y = float(parts[1])   # forward (front = 0°)

        angle = np.degrees(np.arctan2(x, y))

        if start <= end:
            in_sector = (angle >= start) and (angle <= end)
        else:  # wraps across ±180° (e.g. rear sector)
            in_sector = (angle >= start) or  (angle <= end)

        if in_sector:
            removed_count += 1
        else:
            kept.append(line)

    n_kept  = len(kept)
    n_total = len(data_lines)

    # ── Rebuild header: update WIDTH and POINTS only ──────────────────────────
    new_header = []
    for line in header_lines:
        key = line.split()[0].upper() if line.split() else b""
        if key in (b"WIDTH", b"POINTS"):
            new_header.append(key + b" " + str(n_kept).encode())
        else:
            new_header.append(line)

    # ── Write output with same line endings as input ──────────────────────────
    with open(output_path, "wb") as f:
        for hl in new_header:
            f.write(hl + line_ending)
        for dl in kept:
            f.write(dl + line_ending)

    print(f"Input  : {input_path}")
    print(f"Output : {output_path}")
    print(f"Sector : [{start_angle}°, {end_angle}°]")
    print(f"Total  : {n_total:,}")
    print(f"Removed: {removed_count:,}  ({100 * removed_count / n_total:.2f}%)")
    print(f"Kept   : {n_kept:,}")


if __name__ == "__main__":
    if len(sys.argv) < 4:
        print(__doc__)
        sys.exit(1)

    input_pcd   = sys.argv[1]
    start_angle = float(sys.argv[2])
    end_angle   = float(sys.argv[3])
    output_pcd  = sys.argv[4] if len(sys.argv) > 4 else input_pcd.replace(".pcd", "_attacked.pcd")

    removal_attack(input_pcd, start_angle, end_angle, output_pcd)