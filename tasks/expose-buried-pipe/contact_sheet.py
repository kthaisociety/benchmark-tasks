"""Tile recorded camera frames into one PNG for a quick look: contact_sheet.py <run dir> <out.png> <camera> <step>..."""

import sys
from pathlib import Path

import cv2
import numpy as np

run, out, cam = Path(sys.argv[1]), sys.argv[2], sys.argv[3]
steps = [int(s) for s in sys.argv[4:]]
tiles = []
for step in steps:
    hits = sorted(run.glob(f"frames/*/*{cam}_{step:06d}.npy"))
    if not hits:
        continue
    img = cv2.cvtColor(np.load(hits[0]), cv2.COLOR_RGB2BGR)
    cv2.putText(img, f"{cam} t={step / 20:.1f}s", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    tiles.append(img)
cols = 3
while len(tiles) % cols:
    tiles.append(np.zeros_like(tiles[0]))
rows = [np.hstack(tiles[i:i + cols]) for i in range(0, len(tiles), cols)]
cv2.imwrite(out, np.vstack(rows))
