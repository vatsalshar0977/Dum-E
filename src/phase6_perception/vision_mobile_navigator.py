#!/usr/bin/env python3
"""OFFLINE mobile navigation: geometry, policy, and a simulation harness to test the policy.

Everything in this file runs on your laptop with no hardware. It produces abstract wheel
commands in the range +-0.15 and, in the harness, integrates a unicycle model against
synthetic sensor readings so the fail-closed policy can be tested in software. It does NOT
open a serial port, drive a servo, or read a camera.

SAFETY POSITION - read this before driving anything
  * Camera-only perception is not collision protection, and two uncalibrated reflectance
    sensors are not edge protection. Both fail on dark, glossy or wet surfaces, on soft or
    angled obstacles, and on sensor fault. They also fail silently.
  * `choose_wheels` fails CLOSED: stale data, any cliff reading, a near obstacle, a released
    e-stop or a non-stowed arm all produce (0, 0). That is necessary and nowhere near
    sufficient.
  * Start on the FLOOR in a contained area, arm stowed, a supervisor present, a physical
    bumper, and an e-stop on the DRIVE power rail. Never test an autonomous drive on a
    desk. A top-heavy arm that tips off a bench can hit a person.
  * `declared_latency_budget_ms` is arithmetic on declared numbers. Measure the real one.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import List, Optional, Sequence, Tuple

# ---------------------------------------------------------------------------------------
# Wheel command limits. `choose_wheels` clamps to these; nothing here is a real speed.
# ---------------------------------------------------------------------------------------
MAX_COMMAND = 0.15
REVERSE_COMMAND = -0.15

# Declared sensor limits. Replace with measured values for your parts.
OBSTACLE_MIN_M = 0.30
SENSOR_STALE_S = 0.10
WHEEL_RADIUS_MM = 24.0            # matches WHEEL_DIAMETER_MM / 2 in the Phase 2 generator
TRACK_WIDTH_MM = 96.0             # declared distance between the drive wheels (centre-centre)


@dataclass(frozen=True)
class Reading:
    """One control tick's inputs. Any missing field must be signalled as None, not 0.0."""

    now_s: float
    sensor_time_s: float
    target_bearing_rad: float | None
    obstacle_m: float | None
    left_floor_ok: bool
    right_floor_ok: bool
    estop_ok: bool
    arm_stowed: bool


def choose_wheels(reading: Reading) -> tuple[float, float]:
    """Return normalized left/right commands (-0.15..0.15). Fail closed.

    This is the ONLY thing allowed to produce a drive command. It returns (0, 0) unless
    every precondition holds: fresh data, both cliffs clear, obstacle clearance, e-stop
    released, arm stowed, and a finite target bearing.
    """
    if (not reading.estop_ok or not reading.arm_stowed or
        not reading.left_floor_ok or not reading.right_floor_ok or
        reading.now_s < reading.sensor_time_s or
        reading.now_s - reading.sensor_time_s > SENSOR_STALE_S or
        reading.obstacle_m is None or not math.isfinite(reading.obstacle_m) or
        reading.obstacle_m < OBSTACLE_MIN_M or reading.target_bearing_rad is None or
        not math.isfinite(reading.target_bearing_rad)):
        return (0.0, 0.0)
    turn = max(-0.12, min(0.12, reading.target_bearing_rad * 0.2))
    fwd = 0.08 if abs(reading.target_bearing_rad) < 0.3 else 0.0
    return (max(-MAX_COMMAND, min(MAX_COMMAND, fwd - turn)),
            max(-MAX_COMMAND, min(MAX_COMMAND, fwd + turn)))


def cliff_reflex(reading: Reading) -> Tuple[float, float]:
    """Immediate response to a cliff reading. Returns a command PAIR, never a boolean.

    A cliff is the one failure that a stop alone does not fix: stopping on the edge is
    already too late, so the reflex backs away and turns. This still is not protection -
    the drive has to be able to reverse before the wheel reaches the edge, which depends on
    sensing distance and speed that only you can measure.
    """
    if reading.left_floor_ok and reading.right_floor_ok:
        return choose_wheels(reading)
    if not reading.left_floor_ok and not reading.right_floor_ok:
        return (REVERSE_COMMAND, REVERSE_COMMAND)       # both edges: straight back
    if not reading.left_floor_ok:
        return (REVERSE_COMMAND, MAX_COMMAND)           # left edge: back and turn right
    return (MAX_COMMAND, REVERSE_COMMAND)               # right edge: back and turn left


def declared_latency_budget_ms(
    sensor_latency_ms: float, control_period_ms: float, command_write_ms: float
) -> dict:
    """Sum of DECLARED latencies. A planning placeholder, not a measurement.

    Real stopping distance also includes servo slew, drivetrain inertia and the friction of
    your actual floor - none of which are constants you can look up. Measure the total by
    putting a stopwatch on an obstacle appearing and the base actually coming to rest.
    """
    for name, value in (('sensor_latency_ms', sensor_latency_ms),
                        ('control_period_ms', control_period_ms),
                        ('command_write_ms', command_write_ms)):
        if value < 0 or not math.isfinite(value):
            raise ValueError(f'{name} must be a finite, non-negative number')
    total = sensor_latency_ms + control_period_ms + command_write_ms
    return {'sensor_ms': sensor_latency_ms, 'control_period_ms': control_period_ms,
            'command_write_ms': command_write_ms, 'declared_total_ms': total,
            'note': 'declared values only; excludes servo slew, inertia and braking'}


# ---------------------------------------------------------------------------------------
# Grasp-roll estimation
# ---------------------------------------------------------------------------------------
def principal_axis_roll_deg(points_mm: Sequence[Sequence[float]]) -> dict:
    """Principal-axis angle of a detected object, for aligning the claw jaw.

    Feed this the object's segmented points in TABLE MILLIMETRES (or pixels - the angle is
    the same for an affine view, which is the assumption that makes this cheap and also the
    assumption that breaks under strong perspective).

    Returns the long-axis angle in degrees normalised to [-90, 90), the elongation ratio
    (sqrt of the eigenvalue ratio, 1.0 = a circular blob with NO defined axis) and the
    centroid. Callers must refuse to align when the ratio is close to 1: a symmetric blob
    has no long axis, and pretending otherwise snaps the wrist to an arbitrary angle.

    This is geometry only. It does not find the object; something else has to segment it,
    and a wrong segmentation gives a confident, wrong angle.
    """
    pts = [(float(p[0]), float(p[1])) for p in points_mm]
    n = len(pts)
    if n < 3:
        raise ValueError('need at least three points to estimate a principal axis')
    if any(not (math.isfinite(x) and math.isfinite(y)) for x, y in pts):
        raise ValueError('non-finite point')
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    sxx = sum((p[0] - mx) ** 2 for p in pts) / n
    syy = sum((p[1] - my) ** 2 for p in pts) / n
    sxy = sum((p[0] - mx) * (p[1] - my) for p in pts) / n
    # Closed-form eigen-decomposition of a symmetric 2x2 matrix; no numpy required.
    tr, det = sxx + syy, sxx * syy - sxy * sxy
    disc = math.sqrt(max(0.0, (tr / 2.0) ** 2 - det))
    major, minor = tr / 2.0 + disc, tr / 2.0 - disc
    if major <= 0:
        raise ValueError('degenerate point cloud: zero spread')
    angle = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
    # Rotate the axis by 90 deg when the y-variance dominates, so `angle` always follows the
    # major axis rather than whichever branch atan2 happened to pick.
    if syy > sxx:
        angle += math.pi / 2.0
    angle = (math.degrees(angle) + 90.0) % 180.0 - 90.0
    elongation = math.sqrt(major / minor) if minor > 1e-12 else float('inf')
    return {'roll_deg': angle, 'elongation': elongation,
            'centroid_mm': (mx, my), 'points': n,
            'usable': elongation >= 1.5}


def plan_fetch_sequence(
    object_mm: Tuple[float, float],
    start_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    standoff_mm: float = 185.0,
    max_ticks: int = 2000,
    control_hz: float = 20.0,
) -> dict:
    """Simulate a supervised approach to an object and report whether the policy got there.

    The robot is a unicycle: (x, y, heading). The target is placed `standoff_mm` in front of
    the object along the robot's approach direction. Every tick builds a synthetic Reading
    and asks `choose_wheels`, so this harness exercises the SAME policy the hardware would
    use - including the fail-closed branches.

    This is a POLICY test, not a physics or traction test. There is no wheel slip, no
    inertia, no braking distance and no sensor noise, so a "success" here means the state
    machine works, not that the robot will arrive.
    """
    if control_hz <= 0:
        raise ValueError('control_hz must be positive')
    ox, oy = object_mm
    x, y, heading = start_pose
    dt = 1.0 / control_hz
    trace: List[Tuple[float, float, float, float, float]] = []
    arrived = False

    for tick in range(max_ticks):
        dx, dy = ox - x, oy - y
        distance = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx) - heading
        bearing = (bearing + math.pi) % (2.0 * math.pi) - math.pi
        now = tick * dt
        reading = Reading(now_s=now, sensor_time_s=now - dt, target_bearing_rad=bearing,
                          obstacle_m=max(OBSTACLE_MIN_M + 0.01, (distance - standoff_mm) / 1000.0),
                          left_floor_ok=True, right_floor_ok=True, estop_ok=True,
                          arm_stowed=True)
        left, right = choose_wheels(reading)
        if abs(distance - standoff_mm) <= 8.0 and abs(bearing) < 0.15:
            arrived = True
            break
        # Unicycle step. Wheel commands map linearly to forward speed and yaw rate; the
        # constants are illustrative, not calibrated.
        v = (left + right) / 2.0 * 400.0                 # mm/s at full command
        omega = (right - left) * 2.5                     # rad/s at full command
        x += v * math.cos(heading) * dt
        y += v * math.sin(heading) * dt
        heading += omega * dt
        trace.append((now, x, y, left, right))

    return {'arrived': arrived, 'ticks': len(trace),
            'final_pose': (x, y, heading),
            'distance_to_standoff_mm': abs(math.hypot(ox - x, oy - y) - standoff_mm),
            'trace': trace}


if __name__ == '__main__':
    print("=" * 100)
    print("  PHASE 6 - OFFLINE MOBILE POLICY HARNESS (no hardware, no camera, no serial port)")
    print("=" * 100)

    ok = Reading(10.0, 9.95, 0.1, 0.8, True, True, True, True)
    print(f"1. normal offline command        : {choose_wheels(ok)}")
    for label, bad in (
        ("stale sensor data", Reading(10.0, 9.0, 0.1, 0.8, True, True, True, True)),
        ("left cliff reading", Reading(10.0, 9.95, 0.1, 0.8, False, True, True, True)),
        ("obstacle at 100 mm", Reading(10.0, 9.95, 0.1, 0.1, True, True, True, True)),
        ("e-stop released", Reading(10.0, 9.95, 0.1, 0.8, True, True, False, True)),
        ("arm not stowed", Reading(10.0, 9.95, 0.1, 0.8, True, True, True, False)),
        ("no target bearing", Reading(10.0, 9.95, None, 0.8, True, True, True, True)),
        ("no obstacle distance", Reading(10.0, 9.95, 0.1, None, True, True, True, True)),
    ):
        assert choose_wheels(bad) == (0.0, 0.0), label
        print(f"   {label:<29}: (0.0, 0.0)  STOP")

    print()
    print(f"2. cliff reflex, left edge only  : {cliff_reflex(Reading(1.0,0.99,0.0,1.0,False,True,True,True))}"
          "   <- backs away AND turns")
    print(f"   cliff reflex, both edges      : "
          f"{cliff_reflex(Reading(1.0,0.99,0.0,1.0,False,False,True,True))}")

    print()
    budget = declared_latency_budget_ms(15.0, 50.0, 5.0)
    print(f"3. declared latency budget       : {budget['declared_total_ms']:.0f} ms "
          f"({budget['note']})")

    print()
    screwdriver = [(0.0, 0.0), (30.0, 6.0), (60.0, 10.0), (90.0, 16.0), (15.0, 3.0)]
    axis = principal_axis_roll_deg(screwdriver)
    print(f"4. PCA grasp roll, screwdriver   : {axis['roll_deg']:+.1f} deg, "
          f"elongation {axis['elongation']:.1f}x, usable={axis['usable']}")
    blob = [(0.0, 0.0), (10.0, 0.0), (0.0, 10.0), (10.0, 10.0), (5.0, 5.0)]
    axis2 = principal_axis_roll_deg(blob)
    print(f"   PCA grasp roll, square blob   : {axis2['roll_deg']:+.1f} deg, "
          f"elongation {axis2['elongation']:.2f}x, usable={axis2['usable']}"
          "   <- must NOT align")

    print()
    result = plan_fetch_sequence((520.0, 140.0))
    print(f"5. simulated approach to (520, 140) mm: arrived={result['arrived']} in "
          f"{result['ticks']} ticks, final pose "
          f"({result['final_pose'][0]:.0f}, {result['final_pose'][1]:.0f}) mm, "
          f"heading {math.degrees(result['final_pose'][2]):+.1f} deg")
    print("   Policy works in simulation. Traction, braking, tipping and edge distance are")
    print("   all UNMEASURED, so this says nothing about a real drive.")
    print("=" * 100)
