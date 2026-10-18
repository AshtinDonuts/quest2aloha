"""Print Quest Touch controller poses from the USB logcat stream, without ROS."""

import argparse
import math
import select
import subprocess
import time


TAG = "wE9ryARX: "


def quaternion(m):
    """Convert a row-major 3x3 rotation matrix in a 4x4 transform to xyzw."""
    a, b, c = m[0], m[1], m[2]
    d, e, f = m[4], m[5], m[6]
    g, h, i = m[8], m[9], m[10]
    trace = a + e + i
    if trace > 0:
        s = 2 * math.sqrt(trace + 1)
        return ((h - f) / s, (c - g) / s, (d - b) / s, s / 4)
    if a > e and a > i:
        s = 2 * math.sqrt(1 + a - e - i)
        return (s / 4, (b + d) / s, (c + g) / s, (h - f) / s)
    if e > i:
        s = 2 * math.sqrt(1 + e - a - i)
        return ((b + d) / s, s / 4, (f + h) / s, (c - g) / s)
    s = 2 * math.sqrt(1 + i - a - e)
    return ((c + g) / s, (f + h) / s, s / 4, (d - b) / s)


def parse(line):
    if TAG not in line:
        return None
    payload = line.split(TAG, 1)[1].strip()
    transforms, _, buttons = payload.partition("&")
    result = {}
    for item in transforms.split("|"):
        side, sep, values = item.partition(":")
        if not sep or side not in ("l", "r"):
            continue
        try:
            matrix = [float(value) for value in values.split()]
            if len(matrix) != 16:
                continue
            position = (matrix[3], matrix[7], matrix[11])
            if not all(math.isfinite(x) for x in matrix):
                continue
            result[side] = (position, quaternion(matrix))
        except (ValueError, ZeroDivisionError):
            continue
    return result, buttons


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=20)
    args = parser.parse_args()
    process = subprocess.Popen(
        ["adb", "shell", "logcat", "-T", "1", "-s", "wE9ryARX:I", "*:S"],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        bufsize=1,
    )
    assert process.stdout is not None
    end = time.monotonic() + args.seconds
    last_print = 0.0
    pose_frames = 0
    empty_frames = 0
    try:
        while time.monotonic() < end:
            ready, _, _ = select.select([process.stdout], [], [], 0.5)
            if not ready:
                continue
            parsed = parse(process.stdout.readline())
            if parsed is None:
                continue
            poses, buttons = parsed
            if not poses:
                empty_frames += 1
                continue
            pose_frames += 1
            now = time.monotonic()
            if now - last_print < 0.5:
                continue
            last_print = now
            for side, (xyz, xyzw) in poses.items():
                print(
                    f"{'left' if side == 'l' else 'right'} "
                    f"xyz(m)=({xyz[0]:+.3f}, {xyz[1]:+.3f}, {xyz[2]:+.3f}) "
                    f"quat(xyzw)=({xyzw[0]:+.3f}, {xyzw[1]:+.3f}, "
                    f"{xyzw[2]:+.3f}, {xyzw[3]:+.3f})",
                    flush=True,
                )
            if buttons:
                print(f"buttons: {buttons}", flush=True)
    finally:
        process.terminate()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            process.kill()
    print(f"pose_frames={pose_frames}, empty_frames={empty_frames}")


if __name__ == "__main__":
    main()
