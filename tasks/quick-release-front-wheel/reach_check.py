"""Offline IK check of every scripted variant (no simulator): duration and worst TCP error per arm, in mm and degrees.
python reach_check.py"""
import importlib
import os
import sys

from yam_arena.constants import LEFT_BASE_POS, RIGHT_BASE_POS
from yam_arena.demo import ArmPlan

task = importlib.import_module("task")
worst = 0.0
for name in task.VARIANTS:
    os.environ["QR_VARIANT"] = name
    out = []
    for arm, base in (("left", LEFT_BASE_POS), ("right", RIGHT_BASE_POS)):
        plan = ArmPlan(task.demo()[arm], base)
        for k in range(int(plan.duration * 20) + 1):
            plan.action7(k / 20)
        worst = max(worst, plan.worst_ik_mm)
        out.append(f"{arm} {plan.duration:5.1f}s {plan.worst_ik_mm:5.1f}mm {plan.worst_ik_deg:4.1f}deg")
    print(f"{name:16}", " | ".join(out))
print(f"worst IK error {worst:.1f} mm")
sys.exit(0 if worst < 2.0 else 1)
