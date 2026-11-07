"""Print a trial's dig trace every N steps: trace_summary.py <run dir> [every]"""

import json
import sys
from pathlib import Path

run = Path(sys.argv[1])
every = int(sys.argv[2]) if len(sys.argv) > 2 else 40
rows = [json.loads(line) for line in (run / "dig-1.jsonl").open()]
print("time_s in_box on_board off_table bare_mm gap_mm closest_mm struck score")
for r in rows[::every] + rows[-1:]:
    print(
        f"{r['time_s']:6.1f} {r['cubes_in_box']:4d} {r['cubes_on_board']:4d} {r['cubes_off_table']:4d} "
        f"{r['bare_m'] * 1000:6.1f} {r['spade_gap_m'] * 1000:7.1f} {r['closest_gap_m'] * 1000:7.1f} "
        f"{r['struck']!s:5} {r['score']}"
    )
