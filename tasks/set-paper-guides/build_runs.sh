# Build checks on the instance (scripted only, never Dream). From this folder:
#   bash build_runs.sh                     # full demo over 2 trials, then every shortcut variant
#   bash build_runs.sh full push           # chosen variants
# Variants: full nothing side_only push hold shove_stack. Prints the score and a debug line every 20 steps, and
# writes a 1-fps-ish contact sheet per variant (sheet-<variant>.png).
set -e
source /data/IsaacLab-Arena/yam.env >/dev/null 2>&1
cd "$(dirname "$0")"
nvidia-smi --query-gpu=memory.used,memory.total --format=csv,noheader
df -h / | tail -1
I="Set the paper tray's two guides snug against the paper stack, then let go of them."
for V in ${@:-full nothing side_only push hold shove_stack}; do
  EP=1; [ "$V" = full ] && EP=2
  echo "===== variant $V ($EP trial(s))"
  rm -rf "eval-$V"
  DEMO_VARIANT=$V TASK_DEBUG=1 flock ~/.robocurve-simulation.lock \
    inspect-robots run --embodiment yam_arena -E object_jitter=0 -E control_hz=20 --scorer task_score \
      --policy yam_scripted -E cam_width=320 -E cam_height=176 --store-frames --instruction "$I" \
      --max-steps 360 --epochs $EP -E task=$PWD/task.py --log-dir "eval-$V" 2>&1 \
      | grep -E "^step (1?[02468]0|[0-9]*[05]0):|task_score|Error|error|unavailable" || true
  python -c "
import json, glob
fs = sorted(glob.glob('eval-$V/*.json'))
if fs:
    d = json.load(open(fs[-1])); print('per trial', [e.get('task_score') for s in d['samples'] for e in s.get('epochs', [])] or [s.get('scores') for s in d['samples']])
" 2>/dev/null || true
  python -c "
import numpy as np, cv2, glob
fs=sorted(glob.glob('eval-$V/frames/*/*top_cam_*.npy')); n=len(fs)
if n:
    idx=[int(i*(n-1)/7) for i in range(8)]
    ims=[cv2.putText(cv2.cvtColor(np.load(fs[i]),cv2.COLOR_RGB2BGR),str(i),(5,15),0,0.5,(0,0,255),1) for i in idx]
    cv2.imwrite('sheet-$V.png', np.vstack([np.hstack(ims[:4]), np.hstack(ims[4:])]))"
done
