"""Run inspect-robots on this task with a per-step trace. Same command line as `inspect-robots`; with --log-dir <dir>
the task writes <dir>/trace-<trial>.jsonl (score, what the scorer sees, axle position at each dropout, lever closure
and aim, the lever's spring targets). Nothing else changes: parts.py, imported by task.py, builds the jointed wheel
and fork, so `inspect-robots run ... -E task=task.py` works without this wrapper too.

    python run.py run --embodiment yam_arena --policy yam_scripted -E task=$PWD/task.py ... --log-dir runs/demo
"""

import os
import sys
from pathlib import Path

if __name__ == "__main__":
    if "--log-dir" in sys.argv:
        log_dir = Path(sys.argv[sys.argv.index("--log-dir") + 1]).resolve()
        log_dir.mkdir(parents=True, exist_ok=True)
        os.environ["QR_TRACE_DIR"] = str(log_dir)
    from inspect_robots.cli import main

    main()
