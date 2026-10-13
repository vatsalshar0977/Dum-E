#!/usr/bin/env python3
"""Backwards-compatible wrapper for the earlier trajectory API.

`plan` and `check` keep their original signatures so existing callers keep working, and
they now delegate to `trajectory_and_collision.py`. The old centreline rule (TCP, wrist
and elbow centres all at least 20 mm above the desk) is deliberately KEPT and applied in
addition to the newer keep-out volumes, so nothing that passed before becomes less safe.

New code should use `trajectory_and_collision.plan_min_jerk_trajectory` and
`trajectory_and_collision.check_pose_safety` directly.

This does not do real mesh self-collision, dynamic loading or human avoidance. Never
connect it to physical servos without independently verified sensing and limits.
"""

from __future__ import annotations

import math
from pathlib import Path
import sys
from typing import List, Sequence, Tuple

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from src.phase5_control.kinematics import LIMITS, forward  # noqa: E402
from src.phase5_control.trajectory_and_collision import (  # noqa: E402
    VELOCITY_LIMITS_DEG_S,
    check_pose_safety,
    plan_min_jerk_trajectory,
)

SPEED = VELOCITY_LIMITS_DEG_S          # illustrative deg/s, tune on real hardware
LEGACY_CENTERLINE_Z_MIN_MM = 20.0


def check(q: Sequence[float]):
    """Raise ValueError if the pose violates a joint limit, a keep-out, or the old margin."""
    if len(q) != 5 or any(not math.isfinite(a) or a < lo or a > hi
                          for a, (lo, hi) in zip(q, LIMITS)):
        raise ValueError("non-finite angle or joint limit violated")
    report = check_pose_safety(q)
    if not report.ok:
        raise ValueError(f"schematic keep-out violated: {report.describe()}")
    p = forward(q)
    if min(p.z, p.elbow_z, p.wrist_z) < LEGACY_CENTERLINE_Z_MIN_MM:
        raise ValueError(
            f"schematic link centre below {LEGACY_CENTERLINE_Z_MIN_MM:.0f} mm tabletop margin"
        )
    return p


def plan(start: Sequence[float], goal: Sequence[float],
         sample_hz: int = 50) -> List[Tuple[float, Tuple[float, ...]]]:
    """Return [(t, q), ...] for a fully validated minimum-jerk move, or raise."""
    check(start)
    check(goal)
    result = plan_min_jerk_trajectory(start, goal, sample_hz=sample_hz)
    for _, q in result.samples:      # keep the old per-sample guarantee
        check(q)
    return result.samples


if __name__ == "__main__":
    home = (0, -15, 40, -25, 0)
    path = plan(home, (15, 0, 35, -15, 20))
    print(f"Planned {len(path)} schematic samples in {path[-1][0]:.2f}s")
