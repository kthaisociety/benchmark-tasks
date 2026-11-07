#!/usr/bin/env bash
# Render a trial's top and right-wrist cameras side by side as H.264 MP4: export_media.sh <run name> <out name>
# The demo also gives setup.png, its first top-camera frame.
set -eu
source "$HOME/IsaacLab-Arena/yam.env"
cd "$HOME/robocurve-workspaces/expose-buried-pipe/tasks/expose-buried-pipe"
run=runs/${1:?run name}
out=${2:?out name}
mkdir -p media
log=$(find "$run" -maxdepth 1 -name 'adhoc_*.json' -print -quit)
test -f "$log"
if ! find "$run/frames" -name '*right_cam.mp4' -print -quit | grep -q .; then
  inspect-robots video "$log"
fi
top=$(find "$run/frames" -name '*top_cam.mp4' -print -quit)
wrist=$(find "$run/frames" -name '*right_cam.mp4' -print -quit)
ffmpeg -nostdin -v error -y -i "$top" -i "$wrist" -filter_complex '[0:v][1:v]hstack=inputs=2' \
  -an -c:v libx264 -crf 23 -pix_fmt yuv420p -movflags +faststart "media/$out.mp4"
test "$(wc -c < "media/$out.mp4")" -lt 50000000
cp -- "$log" "media/$out.json"
cp -- "$run/dig-1.jsonl" "media/$out-trace.jsonl"
if [ "$out" = demo ]; then
  first=$(find "$run/frames" -name '*top_cam_000000.npy' -print -quit)
  python - "$first" media/setup.png <<'PY'
import sys

import cv2
import numpy as np

assert cv2.imwrite(sys.argv[2], cv2.cvtColor(np.load(sys.argv[1]), cv2.COLOR_RGB2BGR))
PY
fi
ls -la media
