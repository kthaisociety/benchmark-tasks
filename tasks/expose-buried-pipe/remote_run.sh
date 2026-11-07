#!/usr/bin/env bash
# One trial on the GPU VM: remote_run.sh <run name> [policy] [max steps]
set -eu
source "$HOME/IsaacLab-Arena/yam.env"
cd "$HOME/robocurve-workspaces/expose-buried-pipe/tasks/expose-buried-pipe"
out=runs/${1:?run name required}
policy=${2:-yam_scripted}
steps=${3:-3000}
width=320; height=176
if [ "$policy" = yam_scripted ] && [ "${LOWRES:-0}" != 1 ]; then width=640; height=360; fi
rm -rf "$out" && mkdir -p "$out"
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
flock -w 3600 "$HOME/.robocurve-simulation.lock" \
  python run.py run --embodiment yam_arena --policy "$policy" \
    -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width="$width" -E cam_height="$height" \
    --instruction "$instruction" --scorer task_score --max-steps "$steps" --seed 0 --no-prompt \
    --store-frames --log-dir "$out" > "$out/console.log" 2>&1 || { tail -60 "$out/console.log"; exit 1; }
tail -8 "$out/console.log"
tail -1 "$out"/dig-1.jsonl
