#!/usr/bin/env python3
"""Minimum-jerk joint-space planning plus schematic keep-out volumes.

WHAT THIS IS
  * `plan_min_jerk_trajectory` time-scales a joint-space move with the 5th-order
    minimum-jerk polynomial s(tau) = 10tau^3 - 15tau^4 + 6tau^5, which has zero velocity
    AND zero acceleration at both endpoints, and lengthens the move until no joint exceeds
    its declared speed limit. It checks the WHOLE candidate before returning it, and
    raises instead of handing back a partial or "almost safe" path.
  * `check_pose_safety` evaluates a handful of ANALYTIC keep-out volumes on the schematic
    link model from `kinematics.py`: a desk floor, the base turret cylinder and the
    emergency-stop housing.

WHAT THIS IS NOT
  * It is not mesh collision. There is no triangle test against the printed parts, no
    self-collision between links, no payload, no cable snag, no dynamic loading.
  * The speed limits are illustrative, not measured. The servo's real speed depends on
    load and supply. Nothing here has been validated on hardware.
  * It cannot protect a person. Keep hands out of the workspace and use a physical e-stop.

Run `python3 src/phase5_control/trajectory_and_collision.py` for the self-test.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
import sys
from typing import List, Sequence, Tuple

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:          # allows `python3 src/...py` as well as `-m`
    sys.path.insert(0, str(_ROOT))

from src.phase5_control.kinematics import FOREARM, H, LIMITS, UPPER, forward  # noqa: E402

# ---------------------------------------------------------------------------------
# Declared limits. Tune these to your own build, then re-run the planner.
# ---------------------------------------------------------------------------------
VELOCITY_LIMITS_DEG_S = (35.0, 25.0, 35.0, 40.0, 40.0)
ACCELERATION_NOTE = "not constrained; the minimum-jerk profile keeps acceleration finite"

DESK_Z_MIN_MM = 10.0          # tool centre point
WRIST_Z_MIN_MM = 15.0         # wrist pitch/roll bracket
ELBOW_Z_MIN_MM = 18.0         # elbow link

TURRET_R_MIN_MM = 48.0        # stay this far from the base axis ...
TURRET_Z_BELOW_MM = 75.0      # ... while below this height

# The e-stop housing in 03_base_lower_slewing_ring.stl spans x = 41..75, y = -17..17,
# z = 0..32 mm (a 34 x 34 x 28 mm box plus a 4 mm collar).
ESTOP_BOX_MIN_MM = (41.0, -17.0, 0.0)
ESTOP_BOX_MAX_MM = (75.0, 17.0, 32.0)
ESTOP_KEEPOUT_MARGIN_MM = 5.0

# The design brief asked for a keep-out SPHERE at (+56, 0, 26) mm with r = 30 mm. Keep it
# as an option, but note it UNDER-covers the box it is meant to represent: the box corner
# at (75, 17, 32) is sqrt(19^2 + 17^2 + 26^2) = 36.4 mm from that centre, i.e. 6.4 mm
# outside the sphere. A sphere is only conservative if its radius covers the far corner,
# so the box test is the default here.
ESTOP_SPHERE_CENTRE_MM = (56.0, 0.0, 26.0)
ESTOP_SPHERE_RADIUS_MM = 30.0

MIN_MOVE_S = 0.5
MIN_JERK_PEAK_DERIVATIVE = 1.875   # max of s'(tau) for the quintic above


@dataclass(frozen=True)
class SafetyReport:
    """Outcome of a keep-out evaluation. `ok` is False if ANY constraint is violated."""

    ok: bool
    reasons: Tuple[str, ...] = ()
    tcp_mm: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    wrist_mm: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    elbow_mm: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    palm_mm: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    def describe(self) -> str:
        if self.ok:
            return "OK"
        return " | ".join(self.reasons)


@dataclass
class MinJerkPlan:
    """Sampled joint trajectory: `samples` is [(t_seconds, (q1..q5 deg)), ...]."""

    samples: List[Tuple[float, Tuple[float, ...]]]
    duration_s: float
    peak_velocity_deg_s: Tuple[float, ...]
    min_tcp_z_mm: float
    scaled_by_index: int = -1
    notes: List[str] = field(default_factory=list)

    def summary(self) -> str:
        return (
            f"{len(self.samples)} samples over {self.duration_s:.2f}s | "
            f"peak speeds {tuple(round(v, 1) for v in self.peak_velocity_deg_s)} deg/s | "
            f"min TCP z {self.min_tcp_z_mm:.1f} mm | "
            f"limiting joint J{self.scaled_by_index + 1 if self.scaled_by_index >= 0 else '-'}"
        )


def _body_samples(q: Sequence[float]) -> Tuple[Tuple[float, float, float], ...]:
    """Four representative points on the schematic body: TCP, palm, wrist centre, elbow centre."""
    p = forward(tuple(q))
    phi = math.radians(90.0 - p.pitch)          # tool direction, measured from +Z
    yaw = math.radians(q[0])
    palm_r = max(0.0, math.hypot(p.x, p.y) - 25.0 * math.sin(phi))
    wrist_r = UPPER * math.sin(math.radians(q[1])) + FOREARM * math.sin(
        math.radians(q[1] + q[2])
    )
    elbow_r = UPPER * math.sin(math.radians(q[1]))
    elbow_z = H + UPPER * math.cos(math.radians(q[1]))
    return (
        (p.x, p.y, p.z),
        (palm_r * math.cos(yaw), palm_r * math.sin(yaw), p.z - 25.0 * math.cos(phi)),
        (p.wrist_z * 0.0 + wrist_r * math.cos(yaw), wrist_r * math.sin(yaw), p.wrist_z),
        (elbow_r * math.cos(yaw), elbow_r * math.sin(yaw), elbow_z),
    )


def check_pose_safety(
    q: Sequence[float],
    *,
    use_estop_sphere: bool = False,
    margin_mm: float = ESTOP_KEEPOUT_MARGIN_MM,
) -> SafetyReport:
    """Evaluate schematic keep-out volumes at one joint configuration.

    Collects EVERY violation rather than failing on the first, so a caller can log the
    whole picture. `ok` is False if any constraint is violated.
    """
    if len(q) != 5 or any(not math.isfinite(a) for a in q):
        raise ValueError("expected five finite joint angles in degrees")
    tcp, palm, wrist, elbow = _body_samples(q)
    reasons: List[str] = []

    for joint, (lo, hi) in enumerate(LIMITS):
        if q[joint] < lo - 1e-9 or q[joint] > hi + 1e-9:
            reasons.append(f"J{joint + 1}={q[joint]:.1f} deg outside [{lo},{hi}]")

    for name, point, z_min in (
        ("TCP", tcp, DESK_Z_MIN_MM),
        ("palm", palm, WRIST_Z_MIN_MM),
        ("wrist", wrist, WRIST_Z_MIN_MM),
        ("elbow", elbow, ELBOW_Z_MIN_MM),
    ):
        if point[2] < z_min:
            reasons.append(f"desk floor: {name} z={point[2]:.1f} < {z_min:.0f} mm")

    for name, point in (("TCP", tcp), ("palm", palm), ("wrist", wrist), ("elbow", elbow)):
        r = math.hypot(point[0], point[1])
        if point[2] < TURRET_Z_BELOW_MM and r < TURRET_R_MIN_MM:
            reasons.append(
                f"turret cylinder: {name} r={r:.1f} < {TURRET_R_MIN_MM:.0f} mm below "
                f"z={TURRET_Z_BELOW_MM:.0f} mm"
            )

    lo = [v - margin_mm for v in ESTOP_BOX_MIN_MM]
    hi = [v + margin_mm for v in ESTOP_BOX_MAX_MM]
    for name, point in (("TCP", tcp), ("palm", palm), ("wrist", wrist), ("elbow", elbow)):
        if all(lo[i] <= point[i] <= hi[i] for i in range(3)):
            reasons.append(f"e-stop housing box: {name} inside keep-out +{margin_mm:.0f} mm")

    if use_estop_sphere:
        for name, point in (("TCP", tcp), ("palm", palm), ("wrist", wrist), ("elbow", elbow)):
            d = math.dist(point, ESTOP_SPHERE_CENTRE_MM)
            if d < ESTOP_SPHERE_RADIUS_MM:
                reasons.append(f"e-stop sphere: {name} d={d:.1f} < {ESTOP_SPHERE_RADIUS_MM:.0f} mm")

    return SafetyReport(not reasons, tuple(reasons), tcp, wrist, elbow, palm)


def _validate_configuration(q: Sequence[float]) -> None:
    if len(q) != 5:
        raise ValueError("each configuration needs five joint angles")
    if any(not math.isfinite(a) for a in q):
        raise ValueError("non-finite joint angle")


def plan_min_jerk_trajectory(
    start: Sequence[float],
    goal: Sequence[float],
    *,
    sample_hz: int = 100,
    velocity_limits_deg_s: Sequence[float] = VELOCITY_LIMITS_DEG_S,
    enforce_pose_safety: bool = True,
    min_move_s: float = MIN_MOVE_S,
) -> MinJerkPlan:
    """Plan a joint-space minimum-jerk move. Raises if any part of the path is rejected.

    The duration is scaled so that every joint stays inside `velocity_limits_deg_s`:
        T >= 1.875 * |dq_i| / v_max_i
    because 1.875 is the maximum of the quintic's time derivative.

    Raises ValueError - never a warning and never a shortened plan - when the start or goal
    violates a joint limit or a keep-out volume, or when ANY sampled point does. A caller
    that cannot handle a rejected path must not command motion at all.
    """
    _validate_configuration(start)
    _validate_configuration(goal)
    if not (1 <= sample_hz <= 1000):
        raise ValueError("sample_hz must be between 1 and 1000")
    if len(velocity_limits_deg_s) != 5 or any(v <= 0 for v in velocity_limits_deg_s):
        raise ValueError("need five positive velocity limits")

    notes: List[str] = []
    if enforce_pose_safety:
        for label, q in (("start", start), ("goal", goal)):
            report = check_pose_safety(q)
            if not report.ok:
                raise ValueError(f"{label} pose rejected: {report.describe()}")

    duration = min_move_s
    limiting = -1
    per_joint_time = []
    for i, (a, b, v) in enumerate(zip(start, goal, velocity_limits_deg_s)):
        t_i = MIN_JERK_PEAK_DERIVATIVE * abs(b - a) / v
        per_joint_time.append(t_i)
        if t_i > duration:
            duration, limiting = t_i, i
    if min_move_s > max(per_joint_time):
        notes.append(f"duration floored at {min_move_s:.2f}s (speed limits would allow less)")

    count = max(1, math.ceil(duration * sample_hz))
    samples: List[Tuple[float, Tuple[float, ...]]] = []
    peak_velocity = [0.0] * 5
    min_tcp_z = math.inf

    for i in range(count + 1):
        tau = i / count
        s = tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau))
        q = tuple(a + (b - a) * s for a, b in zip(start, goal))
        t = duration * tau

        if enforce_pose_safety:
            report = check_pose_safety(q)
            if not report.ok:
                raise ValueError(
                    f"path rejected at sample {i}/{count} "
                    f"(t={t:.2f}s, tau={tau:.3f}): {report.describe()}"
                )
            min_tcp_z = min(min_tcp_z, report.tcp_mm[2])

        if 0 < i < count:
            ds = tau * tau * (30.0 + tau * (-60.0 + 30.0 * tau))   # s'(tau)
            # dq/dt = dq * s'(tau) / T   (tau runs 0..1 over the whole move)
            for j in range(5):
                peak_velocity[j] = max(peak_velocity[j], abs(goal[j] - start[j]) * ds / duration)

        samples.append((t, q))

    return MinJerkPlan(
        samples=samples,
        duration_s=duration,
        peak_velocity_deg_s=tuple(peak_velocity),
        min_tcp_z_mm=0.0 if min_tcp_z is math.inf else min_tcp_z,
        scaled_by_index=limiting,
        notes=notes,
    )


def declared_loop_travel_mm(approach_speed_mm_s: float, loop_hz: float = 100.0) -> float:
    """Distance travelled in ONE control period: speed / loop rate.

    Read the name carefully. This is NOT a stopping distance. A real stopping distance is
    the sum of
        sensor acquisition latency + transport latency + one control period
        + the servo's own deceleration ramp + the braking of the moving mass,
    and every one of those terms depends on the hardware you actually assembled. This
    function only computes the third term, and only from DECLARED numbers.

    Use it as a planning placeholder and replace it with measurements: time the gap between
    a real obstacle appearing and the command actually reaching the servo, then measure the
    distance the base and the arm tip cover after that command. Keep people and edges
    further away than the measured total.
    """
    if loop_hz <= 0:
        raise ValueError("loop_hz must be positive")
    return approach_speed_mm_s / loop_hz


if __name__ == "__main__":
    home = (0.0, -15.0, 40.0, -25.0, 0.0)
    goal = (15.0, 0.0, 35.0, -15.0, 20.0)

    print("=" * 96)
    print("  PHASE 5 - MINIMUM-JERK PLANNER AND SCHEMATIC KEEP-OUT SELF-TEST")
    print("  These are ANALYTIC model checks. No mesh collision, no dynamics, no hardware.")
    print("=" * 96)

    plan = plan_min_jerk_trajectory(home, goal)
    print(f"1. valid move          : {plan.summary()}")
    for note in plan.notes:
        print(f"                         note: {note}")

    first, last = plan.samples[0], plan.samples[-1]
    assert first[1] == home, first
    assert max(abs(a - b) for a, b in zip(last[1], goal)) < 1e-9
    assert all(t2 > t1 for (t1, _), (t2, _) in zip(plan.samples, plan.samples[1:]))
    print("2. endpoints exact     : start and goal reproduced to <1e-9 deg, time monotonic")

    for label, target in (("desk crash", (90.0, 85.0, 90.0, 85.0, 0.0)),
                          ("joint limit", (100.0, 0.0, 0.0, 0.0, 0.0))):
        try:
            plan_min_jerk_trajectory(home, target)
        except ValueError as exc:
            print(f"3. {label:<19}: REJECTED -> {exc}")
        else:
            raise AssertionError(f"{label} should have been rejected")

    # Both endpoints of this pair pass the keep-outs individually, but the joint-space
    # interpolation between them drives the tool through the desk on the way. This is the
    # case that makes sampling the whole path worth its cost.
    low_a = (26.4, -67.3, -59.2, 67.7, 0.4)
    low_b = (-36.9, -47.8, -65.4, -82.7, -67.9)
    za = check_pose_safety(low_a).tcp_mm[2]
    zb = check_pose_safety(low_b).tcp_mm[2]
    print(f"4. endpoints pass alone: start TCP z={za:.1f} mm OK, goal TCP z={zb:.1f} mm OK")
    try:
        plan_min_jerk_trajectory(low_a, low_b)
    except ValueError as exc:
        print(f"   whole path          : REJECTED -> {exc}")
    else:
        raise AssertionError("this path dips through the desk floor")

    far = max(
        math.dist(corner, ESTOP_SPHERE_CENTRE_MM)
        for corner in (
            (x, y, z)
            for x in (ESTOP_BOX_MIN_MM[0], ESTOP_BOX_MAX_MM[0])
            for y in (ESTOP_BOX_MIN_MM[1], ESTOP_BOX_MAX_MM[1])
            for z in (ESTOP_BOX_MIN_MM[2], ESTOP_BOX_MAX_MM[2])
        )
    )
    print(f"5. e-stop geometry     : the farthest housing corner is {far:.1f} mm from the "
          f"sphere centre, so the brief's r={ESTOP_SPHERE_RADIUS_MM:.0f} mm sphere sits "
          f"{far - ESTOP_SPHERE_RADIUS_MM:+.1f} mm inside it")
    print("                         the box test is the default for that reason")

    print(f"6. loop travel         : 200 mm/s at {100:.0f} Hz = "
          f"{declared_loop_travel_mm(200.0, 100.0):.1f} mm per period. This is ONE term of a "
          f"stopping distance;")
    print("                         sensor, transport, deceleration and braking are all unmeasured.")
    print("=" * 96)
