"""Turn a recorded trial (run.py ... --store-frames --log-dir <dir>) into a review video: the top camera with both
wrist cameras beside it, captioned from the trace, as H.264 MP4. Also writes the setup screenshot (the top
camera's first frame) with --setup.

    python export_media.py runs/demo demo.mp4 --setup setup.png
"""

import argparse
import json
import subprocess
from pathlib import Path

import cv2
import imageio_ffmpeg
import numpy as np

CAMERAS = ("top_cam", "left_cam", "right_cam")
FPS = 20


def frames(run, cam):
    return sorted(run.glob(f"frames/**/*[_~]{cam}_*.npy"))


def caption(row):
    if row is None:
        return "Start"
    arm = row["hinge_deg"]["outer"]
    state = row["fault"] or f"{row['position'] or 'not in rack'}  lean {row['lean_deg']:4.1f} deg  arm {arm:+6.1f} deg"
    return f"t {row['time_s']:5.2f} s  |  {state}  |  score {row['score']:.2f}"


def compose(paths, row, width):
    h = width * 9 // 16
    top = cv2.resize(np.load(paths[0])[..., :3].astype(np.uint8), (width, h))
    side = [cv2.resize(np.load(p)[..., :3].astype(np.uint8), (width // 2, h // 2)) for p in paths[1:]]
    frame = np.hstack((top, np.vstack(side)))
    bar = np.zeros((28, frame.shape[1], 3), np.uint8)
    cv2.putText(bar, caption(row), (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return np.vstack((frame, bar))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--setup", type=Path)
    ap.add_argument("--width", type=int, default=640)
    a = ap.parse_args()
    trace = [json.loads(line) for line in (a.run / "trace-1.jsonl").read_text().splitlines()]
    streams = [frames(a.run, c) for c in CAMERAS]
    n = min(len(s) for s in streams)
    if n < len(trace) + 1:
        raise SystemExit(f"{a.run}: {n} frames for {len(trace)} steps")
    rows = [None, *trace]
    if a.setup:
        img = np.load(streams[0][0])[..., :3].astype(np.uint8)
        cv2.imwrite(str(a.setup), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    first = compose([s[0] for s in streams], None, a.width)
    hh, ww = first.shape[:2]
    cmd = [imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
           "-s", f"{ww}x{hh}", "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p",
           "-movflags", "+faststart", "-crf", "23", str(a.out)]
    enc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    for i, row in enumerate(rows):
        enc.stdin.write(compose([s[i] for s in streams], row, a.width).tobytes())
    enc.stdin.close()
    if enc.wait():
        raise SystemExit("ffmpeg failed")
    print(f"{a.out}: {len(rows)} frames, {a.out.stat().st_size / 1e6:.1f} MB")


if __name__ == "__main__":
    main()
