"""Phase 7 - B.U.D.-E's body language: 100 Hz minimum-jerk keyframe sequences.

Eight channels are driven at once:

    index  0..5   base_yaw, shoulder_pitch, elbow_pitch, wrist_pitch, wrist_roll, jaw  [deg]
    index  6..7   left wheel, right wheel                                              [us]

Wheels take PCA9685 pulse widths, not angles: 1500 us is stop, 1000 us is full reverse and
2000 us is full forward, because the two drive servos are continuous-rotation units.

WHY MINIMUM JERK
----------------
A hobby servo asked for a step input slams the gear train and shakes the whole printed
chassis. The quintic q(tau) = q0 + dq * (10 tau^3 - 15 tau^4 + 6 tau^5) is the unique curve
that starts and ends with zero velocity AND zero acceleration while covering the distance in
the shortest time for a given velocity limit. Because every segment begins and ends at rest,
keyframes chain without a velocity discontinuity, so a whole expressive gesture is one smooth
motion instead of a series of jerks.

The peak speed of that curve is 1.875 * |dq| / T deg/s, which is exactly the rule Phase 5's
planned trajectories use. Every sequence built here is checked against the joint speed limits
at construction time, so an infeasible gesture raises instead of silently frying a servo.

WHAT THIS FILE IS NOT
---------------------
None of these numbers were measured on hardware. The angles are chosen so the printed arm
cannot hit the desk or itself *in the model*; the collision checker in Phase 5 is the
authority, and it has never seen a real arm either. Start with `speed_scale` 0.5 and ramp up.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence as Seq, Tuple

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:            # so `python3 -m` and a direct CLI both work
    sys.path.insert(0, str(ROOT))

from src.phase5_control.trajectory_and_collision import (  # noqa: E402
    VELOCITY_LIMITS_DEG_S,
)

# ---------------------------------------------------------------------------------------
# Channel contract
# ---------------------------------------------------------------------------------------
SAMPLE_HZ = 100.0                       # the brief's minimum control rate
JOINT_NAMES = ("base_yaw", "shoulder_pitch", "elbow_pitch", "wrist_pitch", "wrist_roll",
               "jaw")
WHEEL_NAMES = ("left_wheel", "right_wheel")
CHANNEL_NAMES = JOINT_NAMES + WHEEL_NAMES
N_JOINTS = len(JOINT_NAMES)
N_CHANNELS = len(CHANNEL_NAMES)

# Phase 5's limits cover the five arm joints; the claw is geared 1:1 off an MG90S, so the jaw
# is the sixth and is limited separately (0 deg = closed on the horn stop, 60 deg = wide open).
ARM_LIMITS_DEG: Tuple[Tuple[float, float], ...] = (
    (-90.0, 90.0), (-75.0, 85.0), (-90.0, 90.0), (-85.0, 85.0), (-90.0, 90.0),
)
JAW_LIMITS_DEG = (0.0, 60.0)
JOINT_LIMITS_DEG: Tuple[Tuple[float, float], ...] = ARM_LIMITS_DEG + (JAW_LIMITS_DEG,)

# TWO SPEED BUDGETS, ON PURPOSE.
#
# `VELOCITY_LIMITS_DEG_S` (Phase 5) is the TRANSPORT budget: the limit a planned move gets
# when the arm is carrying a tool across the desk, where a slip means a dropped object.
#
# A personality gesture is a short, small, empty-handed motion, so it gets a bigger budget
# below. The numbers are still conservative against the hardware: an MG996R at 6 V turns
# about 430 deg/s with no load, and the largest move here peaks near a quarter of that. What
# is NOT known is how much the printed arm's inertia costs, so:
#
#   1. keep the masses small (PLA, no counterweight beyond the rubber band), and
#   2. start with `PersonalityDirector(speed_scale=0.5)` and listen to the servos.
GESTURE_SPEED_DEG_S: Tuple[float, ...] = (70.0, 55.0, 70.0, 90.0, 90.0, 90.0)
BASE_SPEED_DEG_S: Tuple[float, ...] = GESTURE_SPEED_DEG_S

WHEEL_STOP_US = 1500.0
WHEEL_MIN_US = 1000.0
WHEEL_MAX_US = 2000.0
WHEEL_SLEW_US_S = 3000.0                # command slew limit, NOT a measured wheel speed

# The rover's top speed is an estimate from the 48 mm wheel and a ~90 RPM continuous-rotation
# servo at 6 V. It is used ONLY to turn "back up 40 mm" into a command duration, and it is
# wrong by however much your servo is: calibrate it with a tape measure.
ROVER_MM_S_AT_FULL = 150.0


def minimum_jerk(q0: float, q1: float, tau: float) -> float:
    """The 10tau^3 - 15tau^4 + 6tau^5 blend, tau in [0, 1]."""
    if not 0.0 <= tau <= 1.0:
        raise ValueError("tau must be in [0, 1]")
    return q0 + (q1 - q0) * (10.0 * tau ** 3 - 15.0 * tau ** 4 + 6.0 * tau ** 5)


def min_jerk_duration(delta_deg: float, speed_deg_s: float) -> float:
    """Shortest time in which minimum jerk can cover `delta_deg` inside a speed limit."""
    if speed_deg_s <= 0.0:
        raise ValueError("speed limit must be positive")
    return 1.875 * abs(delta_deg) / speed_deg_s


# ---------------------------------------------------------------------------------------
# Sequences
# ---------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Keyframe:
    """A pose to reach at `t` seconds, in the joint order above. Wheels default to stop."""

    t: float
    joints: Tuple[float, ...]
    wheels: Tuple[float, float] = (WHEEL_STOP_US, WHEEL_STOP_US)
    note: str = ""

    def __post_init__(self) -> None:
        if len(self.joints) != N_JOINTS:
            raise ValueError(f"keyframe needs {N_JOINTS} joint values")
        if len(self.wheels) != 2:
            raise ValueError("keyframe needs two wheel values")
        for name, value in zip(CHANNEL_NAMES, tuple(self.joints) + tuple(self.wheels)):
            if not math.isfinite(value):
                raise ValueError(f"{name} is not finite")


@dataclass(frozen=True)
class Frame:
    """One 100 Hz sample: what to write to the PCA9685 at `t`."""

    t: float
    joints: Tuple[float, ...]
    wheels: Tuple[float, float]

    def as_dict(self) -> Dict[str, object]:
        return {"t": round(self.t, 4),
                "joints": [round(v, 4) for v in self.joints],
                "wheels": [round(v, 1) for v in self.wheels]}


@dataclass(frozen=True)
class Sequence:
    """A named gesture: keyframes plus the policy around them."""

    name: str
    keyframes: Tuple[Keyframe, ...]
    description: str
    triggers: Tuple[str, ...] = ()
    speed_scale: float = 1.0            # multiplies the per-channel speed limits
    loop: bool = False                  # idle breathing repeats until interrupted
    priority: int = 3                   # lower wins; 0 = reflex, cannot be pre-empted
    returns_to: Optional[Tuple[float, ...]] = None   # pose held after the last keyframe

    @property
    def duration(self) -> float:
        return self.keyframes[-1].t

    def validate(self) -> None:
        if len(self.keyframes) < 2:
            raise ValueError(f"{self.name} needs at least two keyframes")
        if self.keyframes[0].t != 0.0:
            raise ValueError(f"{self.name} must start at t = 0")
        if self.speed_scale <= 0.0:
            raise ValueError(f"{self.name} needs a positive speed scale")
        times = [k.t for k in self.keyframes]
        if any(b <= a for a, b in zip(times, times[1:])):
            raise ValueError(f"{self.name} keyframe times must strictly increase")
        for k in self.keyframes:
            for value, (lo, hi), name in zip(k.joints, JOINT_LIMITS_DEG, JOINT_NAMES):
                if not lo <= value <= hi:
                    raise ValueError(f"{self.name}: {name} = {value} outside [{lo}, {hi}]")
            for value in k.wheels:
                if not WHEEL_MIN_US <= value <= WHEEL_MAX_US:
                    raise ValueError(f"{self.name}: wheel command {value} us is out of range")
        for a, b in zip(self.keyframes, self.keyframes[1:]):
            span = b.t - a.t
            for i, (q0, q1, speed) in enumerate(zip(a.joints, b.joints, BASE_SPEED_DEG_S)):
                needed = min_jerk_duration(q1 - q0, speed * self.speed_scale)
                if needed > span + 1e-9:
                    raise ValueError(
                        f"{self.name}: {JOINT_NAMES[i]} needs {needed:.2f} s for "
                        f"{q1 - q0:+.1f} deg at {speed * self.speed_scale:.1f} deg/s, "
                        f"only {span:.2f} s is allowed")
            for i, (w0, w1) in enumerate(zip(a.wheels, b.wheels)):
                # Stopping (or launching from a stop) is deliberately NOT rate limited: a
                # continuous-rotation servo goes to 1500 us and halts at once, and that is
                # exactly what a cliff reflex wants. What is limited is slamming straight
                # from forward to reverse, which is a shock load on the printed hub.
                if WHEEL_STOP_US in (w0, w1):
                    continue
                if abs(w1 - w0) / span > WHEEL_SLEW_US_S:
                    raise ValueError(
                        f"{self.name}: wheel {i} swings {abs(w1 - w0):.0f} us in {span:.2f} s "
                        f"({abs(w1 - w0) / span:.0f} us/s over the {WHEEL_SLEW_US_S:.0f} us/s "
                        f"limit) without passing through stop")

    def sample(self, hz: float = SAMPLE_HZ) -> List[Frame]:
        """Uniformly sample the whole gesture. The first frame is keyframe zero."""
        if hz <= 0.0:
            raise ValueError("sample rate must be positive")
        self.validate()
        total = self.keyframes[-1].t
        count = max(1, int(round(total * hz)))
        out: List[Frame] = []
        for n in range(count + 1):
            t = min(n / hz, total)
            out.append(self.at(t))
        return out

    def at(self, t: float) -> Frame:
        """The pose at time `t`, clamped to the gesture's own span."""
        self.validate()
        if t < 0.0:
            raise ValueError("time cannot be negative")
        kfs = self.keyframes
        if t >= kfs[-1].t:
            last = kfs[-1]
            return Frame(last.t, tuple(last.joints), tuple(last.wheels))
        for a, b in zip(kfs, kfs[1:]):
            if a.t <= t <= b.t:
                tau = 0.0 if b.t == a.t else (t - a.t) / (b.t - a.t)
                joints = tuple(minimum_jerk(x, y, tau) for x, y in zip(a.joints, b.joints))
                wheels = tuple(minimum_jerk(x, y, tau) for x, y in zip(a.wheels, b.wheels))
                return Frame(t, joints, wheels)
        raise AssertionError("unreachable")


def _reverse_seconds(distance_mm: float) -> float:
    """Open-loop duration for a commanded wheel distance. Calibrate on your own floor."""
    if distance_mm <= 0.0:
        raise ValueError("distance must be positive")
    return distance_mm / ROVER_MM_S_AT_FULL


# ---------------------------------------------------------------------------------------
# The six gestures
# ---------------------------------------------------------------------------------------
def _kf(t: float, yaw: float = 0.0, shoulder: float = 0.0, elbow: float = 0.0,
        wrist: float = 0.0, roll: float = 0.0, jaw: float = 0.0,
        wheels: Tuple[float, float] = (WHEEL_STOP_US, WHEEL_STOP_US),
        note: str = "") -> Keyframe:
    return Keyframe(t, (yaw, shoulder, elbow, wrist, roll, jaw), wheels, note)


# Home is the same pose the viewer and the MoveIt draft start from, so the arm never has to
# jump when a gesture begins.
HOME_DEG: Tuple[float, ...] = (0.0, -15.0, 40.0, -25.0, 0.0, 0.0)

CURIOUS_HEAD_TILT = Sequence(
    name="curious_head_tilt",
    description=("Wake word or a new object: the head cocks over (J5 +35 deg), the base turns a "
                 "little toward whatever was seen, and the wrist lifts slightly."),
    triggers=("wake_word", "new_object"),
    priority=3,
    keyframes=(
        _kf(0.0, yaw=0.0, shoulder=-15.0, elbow=40.0, wrist=-25.0, roll=0.0, jaw=0.0,
            note="home"),
        _kf(1.0, yaw=12.0, shoulder=-6.0, elbow=34.0, wrist=-12.0, roll=35.0, jaw=12.0,
            note="tilt and perk up"),
        _kf(1.35, yaw=12.0, shoulder=-6.0, elbow=34.0, wrist=-12.0, roll=35.0, jaw=12.0,
            note="hold the pose: a held tilt reads as attention"),
    ),
    returns_to=(12.0, -6.0, 34.0, -12.0, 35.0, 12.0),
)

AFFIRMATIVE_NOD = Sequence(
    name="affirmative_nod",
    description=("Double nod on J4 (wrist pitch): down, up, down, settle. The brief's "
                 "'yes, on it' acknowledgement."),
    triggers=("command_acknowledged",),
    priority=2,
    speed_scale=1.4,
    keyframes=(
        _kf(0.0, yaw=0.0, shoulder=-15.0, elbow=40.0, wrist=-25.0, roll=0.0, jaw=5.0,
            note="home, jaw closed on nothing"),
        _kf(0.32, yaw=0.0, shoulder=-13.0, elbow=40.0, wrist=-5.0, roll=0.0, jaw=5.0,
            note="nod down"),
        _kf(0.87, yaw=0.0, shoulder=-16.0, elbow=41.0, wrist=-40.0, roll=0.0, jaw=5.0,
            note="up"),
        _kf(1.42, yaw=0.0, shoulder=-13.0, elbow=40.0, wrist=-5.0, roll=0.0, jaw=5.0,
            note="down again"),
        _kf(1.76, yaw=0.0, shoulder=-15.0, elbow=40.0, wrist=-25.0, roll=0.0, jaw=5.0,
            note="back to home"),
    ),
    returns_to=HOME_DEG,
)

SAD_FAILED_GRASP_DROOP = Sequence(
    name="sad_failed_grasp_droop",
    description=("A closed jaw that draws less than 180 mA did not actually grip anything. "
                 "The arm droops slowly and shakes its base once, like a sigh."),
    triggers=("grasp_failed",),
    priority=2,
    keyframes=(
        _kf(0.0, yaw=0.0, shoulder=-15.0, elbow=40.0, wrist=-25.0, roll=0.0, jaw=0.0,
            note="home, jaw commanded shut"),
        _kf(0.8, yaw=0.0, shoulder=-8.0, elbow=45.0, wrist=-15.0, roll=0.0, jaw=35.0,
            note="jaw opens: nothing was there"),
        _kf(2.2, yaw=0.0, shoulder=20.0, elbow=70.0, wrist=25.0, roll=0.0, jaw=18.0,
            note="droop (2.2 s: slow enough to read as sadness, not a failure)"),
        _kf(2.6, yaw=-7.0, shoulder=22.0, elbow=72.0, wrist=27.0, roll=0.0, jaw=18.0,
            note="sigh, left"),
        _kf(3.0, yaw=7.0, shoulder=20.0, elbow=70.0, wrist=25.0, roll=0.0, jaw=18.0,
            note="sigh, right"),
        _kf(3.4, yaw=0.0, shoulder=25.8, elbow=84.8, wrist=39.4, roll=0.0, jaw=15.0,
            note="the repository's sad_droop pose: shoulders down, head low"),
    ),
    returns_to=(0.0, 25.8, 84.8, 39.4, 0.0, 15.0),
)

EXCITED_SUCCESS_WIGGLE = Sequence(
    name="excited_success_wiggle",
    description=("A confirmed grasp: perky posture, head up, then a rapid alternating "
                 "head-tilt / base wiggle. The jaw stays shut so the tool is not dropped."),
    triggers=("grasp_success",),
    priority=2,
    speed_scale=1.5,
    keyframes=(
        _kf(0.0, yaw=0.0, shoulder=-15.0, elbow=40.0, wrist=-25.0, roll=0.0, jaw=5.0,
            note="home"),
        _kf(0.5, yaw=0.0, shoulder=-28.0, elbow=26.0, wrist=-45.0, roll=-12.0, jaw=2.0,
            note="perk up"),
        _kf(0.9, yaw=11.0, shoulder=-28.0, elbow=26.0, wrist=-45.0, roll=12.0, jaw=2.0,
            note="wiggle 1"),
        _kf(1.3, yaw=-11.0, shoulder=-28.0, elbow=26.0, wrist=-45.0, roll=-12.0, jaw=2.0,
            note="wiggle 2"),
        _kf(1.7, yaw=11.0, shoulder=-28.0, elbow=26.0, wrist=-45.0, roll=12.0, jaw=2.0,
            note="wiggle 3"),
        _kf(2.1, yaw=0.0, shoulder=-20.0, elbow=34.0, wrist=-30.0, roll=0.0, jaw=2.0,
            note="settle, still holding"),
    ),
    returns_to=(0.0, -20.0, 34.0, -30.0, 0.0, 2.0),
)


def _build_startled_cliff_recoil() -> Sequence:
    """The one gesture that moves the rover, so its duration comes from the distance.

    Timing note: the WHEELS are the reflex and they stop after `back_up_s` plus the
    supervisor's 3 ms tick. The BODY pose is deliberately allowed to arrive later, because a
    printed arm cannot and should not snap 25 deg in a quarter of a second. Splitting the two
    is what makes this both fast where it matters and kind to the servos.
    """
    back_up_s = _reverse_seconds(40.0)            # 40 mm at the declared top speed
    stop_s = back_up_s + 0.15                     # the wheels stop here, blind or not
    # The arm target is the startled pose. It is blended in two steps that coincide with the
    # two wheel commands, so neither the wheels nor the servos are asked for a step input.
    startled = (-30.0, 15.0, -50.0, -10.0, 25.0)  # shoulder, elbow, wrist, roll, jaw
    home5 = (-15.0, 40.0, -25.0, 0.0, 0.0)

    def _blend(fraction: float) -> Tuple[float, ...]:
        return tuple(h + fraction * (t - h) for h, t in zip(home5, startled))

    early, late = _blend(0.25), _blend(0.55)
    return Sequence(
        name="startled_cliff_recoil",
        description=(f"TCRT5000 sees the floor vanish: both wheels full reverse for "
                     f"{back_up_s:.2f} s ({back_up_s * ROVER_MM_S_AT_FULL:.0f} mm at the "
                     f"declared {ROVER_MM_S_AT_FULL:.0f} mm/s), then head snaps up and shakes. "
                     f"Priority 0: this interrupts anything else that is playing."),
        triggers=("cliff",),
        priority=0,
        speed_scale=2.0,
        keyframes=(
            _kf(0.0, yaw=0.0, shoulder=-15.0, elbow=40.0, wrist=-25.0, roll=0.0, jaw=0.0,
                wheels=(WHEEL_MIN_US, WHEEL_MIN_US), note="wheels already reversing"),
            _kf(back_up_s, yaw=0.0, shoulder=early[0], elbow=early[1], wrist=early[2],
                roll=early[3], jaw=early[4], wheels=(WHEEL_MIN_US, WHEEL_MIN_US),
                note="recoil starts"),
            _kf(stop_s, yaw=0.0, shoulder=late[0], elbow=late[1], wrist=late[2],
                roll=late[3], jaw=late[4], wheels=(WHEEL_STOP_US, WHEEL_STOP_US),
                note="wheels STOP: never keep reversing blind"),
            _kf(0.75, yaw=0.0, shoulder=-30.0, elbow=15.0, wrist=-50.0, roll=-10.0,
                jaw=25.0, wheels=(WHEEL_STOP_US, WHEEL_STOP_US), note="head up"),
            _kf(1.15, yaw=14.0, shoulder=-30.0, elbow=15.0, wrist=-50.0, roll=-12.0,
                jaw=25.0, note="shake, left"),
            _kf(1.55, yaw=-14.0, shoulder=-30.0, elbow=15.0, wrist=-50.0, roll=-12.0,
                jaw=25.0, note="shake, right"),
            _kf(1.95, yaw=0.0, shoulder=-15.0, elbow=40.0, wrist=-25.0, roll=0.0, jaw=0.0,
                note="back to home, wheels still stopped"),
        ),
        returns_to=HOME_DEG,
    )


STARTLED_CLIFF_RECOIL = _build_startled_cliff_recoil()

IDLE_BREATHING_AND_LOOK_AROUND = Sequence(
    name="idle_breathing_and_look_around",
    description=("Idle for more than 10 s: 0.25 Hz breathing on J2/J4 (a 4 s period, +/-2.5 "
                 "deg) with a slow glance left and right. Loops until something happens."),
    triggers=("idle",),
    priority=3,
    loop=True,
    keyframes=(
        _kf(0.0, yaw=0.0, shoulder=-15.0, elbow=40.0, wrist=-25.0, roll=0.0, jaw=0.0,
            note="settled"),
        _kf(1.0, yaw=0.0, shoulder=-12.5, elbow=41.0, wrist=-22.5, roll=0.0, jaw=0.0,
            note="inhale"),
        _kf(2.0, yaw=0.0, shoulder=-15.0, elbow=40.0, wrist=-25.0, roll=0.0, jaw=0.0,
            note="exhale"),
        _kf(3.0, yaw=0.0, shoulder=-12.5, elbow=41.0, wrist=-22.5, roll=0.0, jaw=0.0,
            note="inhale"),
        _kf(4.0, yaw=0.0, shoulder=-15.0, elbow=40.0, wrist=-25.0, roll=0.0, jaw=0.0,
            note="exhale"),
        _kf(5.4, yaw=22.0, shoulder=-10.0, elbow=38.0, wrist=-20.0, roll=12.0, jaw=0.0,
            note="glance left"),
        _kf(6.8, yaw=-22.0, shoulder=-10.0, elbow=38.0, wrist=-20.0, roll=-12.0, jaw=0.0,
            note="glance right"),
        _kf(8.0, yaw=0.0, shoulder=-15.0, elbow=40.0, wrist=-25.0, roll=0.0, jaw=0.0,
            note="settled: loop"),
    ),
    returns_to=HOME_DEG,
)

SEQUENCES: Dict[str, Sequence] = {
    s.name: s for s in (CURIOUS_HEAD_TILT, AFFIRMATIVE_NOD, SAD_FAILED_GRASP_DROOP,
                        EXCITED_SUCCESS_WIGGLE, STARTLED_CLIFF_RECOIL,
                        IDLE_BREATHING_AND_LOOK_AROUND)
}


# ---------------------------------------------------------------------------------------
# Failure detection and the director
# ---------------------------------------------------------------------------------------
FAILED_GRASP_MA = 180.0
GRASP_CURRENT_HOLD_S = 0.35             # a real grip holds current; a miss only spikes
IDLE_AFTER_S = 10.0

PSU_BUDGET_MA = 3200.0                  # Phase 1's cutoff: a stalled gripper is not subtle


def looks_like_failed_grasp(current_ma: float, jaw_target_deg: float, jaw_deg: float,
                            held_s: float = GRASP_CURRENT_HOLD_S) -> bool:
    """True when a *closed* jaw is drawing almost nothing, i.e. it closed on air.

    An empty MG90S closing on nothing draws a few tens of mA; a jaw squeezing a 6 mm
    screwdriver shaft draws several hundred. The check is deliberately one-sided: it reports
    a miss, it never declares a success, because a jammed jaw also draws nothing.
    """
    if held_s < GRASP_CURRENT_HOLD_S:
        return False
    closed = jaw_target_deg <= 5.0 and jaw_deg <= 12.0
    return closed and current_ma < FAILED_GRASP_MA


@dataclass
class DirectorState:
    """What the director is doing right now. Exposed so the logs and the viewer can see it."""

    sequence: Optional[Sequence] = None
    started: float = 0.0
    event: str = "boot"
    halted: bool = False
    hold_joints: Tuple[float, ...] = HOME_DEG
    last_motion_t: float = 0.0
    log: List[Tuple[float, str]] = field(default_factory=list)


class PersonalityDirector:
    """Chooses gestures from events, and answers with an 8-channel command at any time.

    Priority rules (lower number wins):

        0   cliff reflex - interrupts everything, including another reflex
        2   command acknowledgement, grasp outcome, halt
        3   wake word, new object, idle

    A halt is not a gesture. `halt()` returns a single immediate frame - wheels to 1500 us
    and the arm frozen at its last commanded pose - and the director stays halted until
    `resume()` is called. There is no minimum-jerk blend on a halt: blending would move the
    wheels *after* you asked them to stop.
    """

    def __init__(self, t0: float = 0.0, idle_after_s: float = IDLE_AFTER_S) -> None:
        self.idle_after_s = idle_after_s
        self.state = DirectorState(last_motion_t=t0)
        self.state.sequence = None
        self._t0 = t0

    # -- selection ----------------------------------------------------------------------
    def submit(self, event: str, t: float, current_ma: Optional[float] = None,
               jaw_target_deg: Optional[float] = None, jaw_deg: Optional[float] = None) -> Optional[Sequence]:
        """Offer an event to the director. Returns the sequence that starts, or None."""
        if self.state.halted and event != "stop":
            self._log(t, f"{event} ignored: halted")
            return None
        if event == "stop":
            self.halt(t)
            return None
        if event == "grasp_check":
            if current_ma is None or jaw_target_deg is None or jaw_deg is None:
                raise ValueError("grasp_check needs current_ma, jaw_target_deg and jaw_deg")
            event = "grasp_failed" if looks_like_failed_grasp(current_ma, jaw_target_deg,
                                                             jaw_deg) else "grasp_success"
        candidates = [s for s in SEQUENCES.values() if event in s.triggers]
        if not candidates:
            self._log(t, f"{event} has no gesture")
            return None
        choice = min(candidates, key=lambda s: s.priority)
        current = self.state.sequence
        if current is not None and not current.loop:
            running = t - self.state.started < current.duration
            if running and current.priority <= choice.priority and event != "cliff":
                self._log(t, f"{event} dropped: {current.name} (priority {current.priority}) "
                             f"is still playing")
                return None
            if running and current.priority == choice.priority and current.name != choice.name:
                self._log(t, f"{event} dropped: {current.name} is still playing")
                return None
        self.state.sequence = choice
        self.state.started = t
        self.state.event = event
        self.state.last_motion_t = t
        self._log(t, f"start {choice.name} ({choice.duration:.2f} s, priority {choice.priority})")
        return choice

    def halt(self, t: float) -> Frame:
        """Immediate software stop. Wheels to 1500 us, arm frozen where it was commanded."""
        hold = self.frame(t).joints if self.state.sequence is not None else self.state.hold_joints
        self.state.halted = True
        self.state.sequence = None
        self.state.hold_joints = tuple(hold)
        self.state.event = "stop"
        self._log(t, "HALT: wheels 1500 us, arm pose held")
        return Frame(t, tuple(hold), (WHEEL_STOP_US, WHEEL_STOP_US))

    def resume(self, t: float) -> None:
        self.state.halted = False
        self.state.last_motion_t = t
        self._log(t, "resume")

    # -- output -------------------------------------------------------------------------
    def frame(self, t: float) -> Frame:
        """The command to send at `t`, whatever the current state is."""
        if self.state.halted:
            return Frame(t, tuple(self.state.hold_joints), (WHEEL_STOP_US, WHEEL_STOP_US))
        seq = self.state.sequence
        if seq is None:
            return Frame(t, tuple(self.state.hold_joints), (WHEEL_STOP_US, WHEEL_STOP_US))
        elapsed = t - self.state.started
        if seq.loop:
            elapsed = elapsed % seq.duration
        elif elapsed >= seq.duration:
            pose = seq.returns_to or seq.keyframes[-1].joints
            self.state.hold_joints = tuple(pose)
            return Frame(t, tuple(pose), (WHEEL_STOP_US, WHEEL_STOP_US))
        return seq.at(elapsed)

    def wheels_are_moving(self, t: float) -> bool:
        frame = self.frame(t)
        return any(abs(us - WHEEL_STOP_US) > 1.0 for us in frame.wheels)

    def idle_seconds(self, t: float) -> float:
        return t - self.state.last_motion_t

    def check_idle(self, t: float) -> Optional[Sequence]:
        """Call once per control tick. Starts the breathing loop after `idle_after_s`."""
        if self.state.halted or self.state.sequence is not None:
            return None
        if self.idle_seconds(t) < self.idle_after_s:
            return None
        return self.submit("idle", t)

    # -- bookkeeping --------------------------------------------------------------------
    def _log(self, t: float, message: str) -> None:
        self.state.log.append((t, message))


# ---------------------------------------------------------------------------------------
# Self test
# ---------------------------------------------------------------------------------------
def _self_test() -> int:
    print("B.U.D.-E Phase 7 personality engine self test")
    print(f"  sample rate {SAMPLE_HZ:.0f} Hz, {N_CHANNELS} channels, "
          f"min-jerk segments with zero velocity and acceleration at every keyframe")
    failures = 0
    for name, seq in SEQUENCES.items():
        try:
            seq.validate()
        except ValueError as exc:
            failures += 1
            print(f"  FAIL {name}: {exc}")
            continue
        frames = seq.sample()
        peak = [0.0] * N_JOINTS
        for a, b in zip(frames, frames[1:]):
            dt = b.t - a.t
            if dt <= 0:
                continue
            for i in range(N_JOINTS):
                peak[i] = max(peak[i], abs(b.joints[i] - a.joints[i]) / dt)
        moving = [f for f in frames if any(abs(w - WHEEL_STOP_US) > 1 for w in f.wheels)]
        print(f"  OK   {name:<30} {seq.duration:5.2f} s {len(frames):4d} frames "
              f"priority {seq.priority} speed x{seq.speed_scale:g}"
              f"{f' wheels for {moving[0].t:.2f}..{moving[-1].t:.2f} s' if moving else ''}")
        for i, (pk, limit) in enumerate(zip(peak, BASE_SPEED_DEG_S)):
            if pk > limit * seq.speed_scale + 1e-6:
                failures += 1
                print(f"       FAIL peak {JOINT_NAMES[i]} {pk:.2f} deg/s over "
                      f"{limit * seq.speed_scale:.1f}")
    # The director's stated behaviours.
    d = PersonalityDirector(t0=0.0)
    d.submit("wake_word", 0.1)
    if d.state.sequence is not CURIOUS_HEAD_TILT:
        failures += 1
        print("  FAIL wake word did not start the head tilt")
    d.submit("cliff", 0.5)
    if d.state.sequence is not STARTLED_CLIFF_RECOIL:
        failures += 1
        print("  FAIL the cliff reflex did not pre-empt the head tilt")
    f = d.frame(0.6)                       # 0.1 s into the recoil: wheels must be reversing
    if not (f.wheels[0] < WHEEL_STOP_US and f.wheels[1] < WHEEL_STOP_US):
        failures += 1
        print("  FAIL the cliff recoil is not reversing both wheels")
    d.halt(1.0)
    f = d.frame(1.0)
    if f.wheels != (WHEEL_STOP_US, WHEEL_STOP_US) or d.state.sequence is not None:
        failures += 1
        print("  FAIL halt did not stop the wheels and clear the gesture")
    d.resume(2.0)
    if not looks_like_failed_grasp(90.0, 0.0, 3.0, held_s=0.5):
        failures += 1
        print("  FAIL a 90 mA closed jaw was not flagged as a missed grasp")
    if looks_like_failed_grasp(600.0, 0.0, 3.0, held_s=0.5):
        failures += 1
        print("  FAIL a 600 mA closed jaw was flagged as a missed grasp")
    if looks_like_failed_grasp(90.0, 45.0, 40.0, held_s=0.5):
        failures += 1
        print("  FAIL an OPEN jaw was treated as a closed one")
    total_ms = sum(seq.duration * 1000.0 for seq in SEQUENCES.values())
    print(f"  {len(SEQUENCES)} gestures, {total_ms:.0f} ms of scripted body language")
    print("  PASS" if failures == 0 else f"  {failures} FAILURES")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    import sys

    sys.exit(_self_test())
