"""Offline IK check of every scripted variant (no simulator): worst TCP error per arm, in mm."""
import os
import sys

from yam_arena.constants import LEFT_BASE_POS, RIGHT_BASE_POS
from yam_arena.demo import ArmPlan

import task

for name in task.VARIANTS:
    os.environ["AMO_VARIANT"] = name
    out = []
    for arm, base in (("left", LEFT_BASE_POS), ("right", RIGHT_BASE_POS)):
        plan = ArmPlan(task.demo()[arm], base)
        for k in range(int(plan.duration * 20)):
            plan.action7(k / 20)
        out.append(f"{arm} {plan.duration:.1f}s {plan.worst_ik_mm:.1f}mm")
    print(name.ljust(14), " | ".join(out))
