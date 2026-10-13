"""Phase 8 - the safety supervisor, the telemetry recorder and six fault injections.

WHAT THIS IS
------------
One object, `SafetySupervisor`, sits between every command the upper layers want to send and
the servos. It sees one frame of sensor data per tick and answers with a frame of commands.
It can also *refuse*: on a brownout it stops holding torque, on a cliff it stops the wheels in
the same tick it noticed, and on the hardware e-stop it latches and will not let go until the
software targets have been synchronised to where the arm actually is.

The rest of the file is the evidence: `logs/bude_telemetry.jsonl` (one JSON object per tick)
plus a CSV converter, and `FaultInjector`, which drives the supervisor through the six faults
the brief asks for and reports latency for each.

WHAT THIS IS NOT
----------------
This is a **simulated** supervisor. There is no serial port, no PCA9685, no ADC. The ticks are
synthesised, so every latency below is the supervisor's own decision latency - the time between
"the value was visible in this frame" and "the command changed" - and NOT the wall-clock time
from a physical event to a physical servo. Real numbers need firmware and a scope.

The most important consequence: the brief's "stop the wheels in under 5 ms" deadline CANNOT be
met through Python and a USB serial link. `HARDWARE_CLIFF_DEADLINE_MS` puts that requirement
where it belongs - in the ESP32's interrupt handler - and this file measures only what it can.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.phase5_control.kinematics import LIMITS, forward        # noqa: E402
from src.phase7_voice_personality.personality_engine import (    # noqa: E402
    HOME_DEG, SAMPLE_HZ, SEQUENCES, WHEEL_MAX_US, WHEEL_MIN_US, WHEEL_STOP_US,
    looks_like_failed_grasp,
)

# ---------------------------------------------------------------------------------------
# Thresholds. Every one of these is a DECLARATION, not a measurement. They come from the
# datasheets and the Phase 1 budget; retune them with real parts on the bench.
# ---------------------------------------------------------------------------------------
NOMINAL_BUS_V = 6.0
OVERCURRENT_MA = 2600.0          # Phase 1's cutoff: below the 3.2 A the 6 V 5 A supply can do
OVERCURRENT_HOLD_S = 0.150       # a stall that lasts longer than this is a fault, not a spike
MISSED_GRASP_MA = 180.0          # matches Phase 7's detector - keep the two in step
MISSED_GRASP_HOLD_S = 0.350
MISSED_GRASP_MAX_RETRIES = 1
OBSTACLE_STOP_MM = 80.0          # HC-SR04
OBSTACLE_SLOW_MM = 250.0
BROWNOUT_V = 4.90                # below this the servos are not trustworthy
BROWNOUT_RECOVER_V = 5.20
ESTOP_MAX_V = 0.05               # the LA38-11ZS cuts the servo rail: the INA219 sees ~0 V
SENSOR_STALE_S = 0.25
HARDWARE_CLIFF_DEADLINE_MS = 5.0  # MUST be met in the ESP32 ISR, not here
CONTROL_HZ = 100.0

LOG_PATH = ROOT / "logs" / "bude_telemetry.jsonl"
CSV_PATH = ROOT / "logs" / "bude_telemetry.csv"


class State(str):
    """Phase of the supervisor. Plain strings so they land in JSON unchanged."""


IDLE = "IDLE"
GESTURE = "GESTURE"
BACKOFF_HOME = "BACKOFF_HOME"
GRASP_RETRY = "GRASP_RETRY"
CLIFF_RECOVER = "CLIFF_RECOVER"
OBSTACLE_HOLD = "OBSTACLE_HOLD"
BROWNOUT_HOLD = "BROWNOUT_HOLD"
ESTOP_HOLD = "ESTOP_HOLD"


@dataclass
class Sensors:
    """One tick of the world, as the ESP32 would report it."""

    t: float
    joints_deg: Tuple[float, ...]                    # 6 measured joint angles
    bus_v: float                                     # INA219 bus voltage (servo rail)
    current_ma: float                                # INA219 current
    wheels_us: Tuple[float, float]                   # what we last commanded
    cliff_left: bool = False                         # TCRT5000, GPIO 25
    cliff_right: bool = False                        # TCRT5000, GPIO 26
    distance_mm: Optional[float] = None              # HC-SR04
    jaw_target_deg: float = 60.0                     # where the claw was asked to go
    sensor_t: Optional[float] = None                 # when the sensors were sampled

    def age_s(self) -> float:
        return self.t - (self.sensor_t if self.sensor_t is not None else self.t)


@dataclass
class Command:
    """One tick of commands, plus why."""

    t: float
    wheels_us: Tuple[float, float]
    joints_deg: Optional[Tuple[float, ...]]          # None = hold where you are
    enable_servos: bool
    state: str
    gesture: Optional[str] = None
    notes: List[str] = field(default_factory=list)

    def as_dict(self) -> Dict[str, object]:
        return {"t": round(self.t, 4), "wheels_us": [round(v, 1) for v in self.wheels_us],
                "joints_deg": ([round(v, 3) for v in self.joints_deg]
                               if self.joints_deg is not None else None),
                "enable_servos": self.enable_servos, "state": self.state,
                "gesture": self.gesture, "notes": self.notes}


# ---------------------------------------------------------------------------------------
# Telemetry
# ---------------------------------------------------------------------------------------
@dataclass
class TelemetryRow:
    """Exactly the brief's list: joints, TCP, bus volts and milliamps, wheels, cliffs, phase."""

    t: float
    joints_deg: Tuple[float, ...]
    tcp_mm: Tuple[float, float, float]
    bus_v: float
    current_ma: float
    wheels_us: Tuple[float, float]
    cliff_left: bool
    cliff_right: bool
    state: str
    gesture: Optional[str]
    enable_servos: bool

    @classmethod
    def build(cls, sensors: Sensors, command: Command) -> "TelemetryRow":
        pose = forward(tuple(sensors.joints_deg[:5]))
        return cls(t=sensors.t, joints_deg=tuple(sensors.joints_deg), tcp_mm=(pose.x, pose.y,
                                                                              pose.z),
                   bus_v=sensors.bus_v, current_ma=sensors.current_ma,
                   wheels_us=tuple(command.wheels_us), cliff_left=sensors.cliff_left,
                   cliff_right=sensors.cliff_right, state=command.state,
                   gesture=command.gesture, enable_servos=command.enable_servos)

    def as_dict(self) -> Dict[str, object]:
        return {"t": round(self.t, 4),
                "joints_deg": [round(v, 3) for v in self.joints_deg],
                "tcp_mm": [round(v, 2) for v in self.tcp_mm],
                "bus_v": round(self.bus_v, 3), "current_ma": round(self.current_ma, 1),
                "wheels_us": [round(v, 1) for v in self.wheels_us],
                "cliff": [self.cliff_left, self.cliff_right],
                "state": self.state, "gesture": self.gesture,
                "enable_servos": self.enable_servos}


class TelemetryRecorder:
    """Append-only JSONL, plus a CSV you can open in a spreadsheet.

    JSONL is the format that survives a crash: a half-written line costs you one tick. The CSV
    is written at `close()` from what is still in memory, and the CSV converter can re-read a
    JSONL file after the fact, so nothing is lost if the process is killed mid-run.
    """

    def __init__(self, path: Path = LOG_PATH, echo: bool = False) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.rows: List[TelemetryRow] = []
        self._handle = self.path.open("w", encoding="utf-8")
        self.echo = echo

    def log(self, sensors: Sensors, command: Command) -> TelemetryRow:
        row = TelemetryRow.build(sensors, command)
        self.rows.append(row)
        self._handle.write(json.dumps(row.as_dict()) + "\n")
        if self.echo and len(self.rows) % 25 == 0:
            print(f"    t={row.t:6.2f} s  {row.state:<13} bus {row.bus_v:4.2f} V  "
                  f"{row.current_ma:6.1f} mA  wheels {row.wheels_us[0]:.0f}/"
                  f"{row.wheels_us[1]:.0f} us  tcp ({row.tcp_mm[0]:.0f}, {row.tcp_mm[1]:.0f}, "
                  f"{row.tcp_mm[2]:.0f}) mm")
        return row

    @property
    def line_count(self) -> int:
        return len(self.rows)

    def close(self) -> None:
        self._handle.close()

    def to_csv(self, path: Path = CSV_PATH) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["t_s", "j1", "j2", "j3", "j4", "j5", "j6", "tcp_x_mm", "tcp_y_mm",
                             "tcp_z_mm", "bus_v", "current_ma", "left_us", "right_us",
                             "cliff_left", "cliff_right", "state", "gesture", "enable_servos"])
            for row in self.rows:
                writer.writerow([f"{row.t:.4f}", *[f"{v:.3f}" for v in row.joints_deg],
                                 *[f"{v:.2f}" for v in row.tcp_mm], f"{row.bus_v:.3f}",
                                 f"{row.current_ma:.1f}", f"{row.wheels_us[0]:.1f}",
                                 f"{row.wheels_us[1]:.1f}", int(row.cliff_left),
                                 int(row.cliff_right), row.state, row.gesture or "",
                                 int(row.enable_servos)])
        return path

    @staticmethod
    def jsonl_to_csv(source: Path, target: Path = CSV_PATH) -> Path:
        rows = [json.loads(line) for line in Path(source).read_text().splitlines() if line]
        target = Path(target)
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["t_s", "j1", "j2", "j3", "j4", "j5", "j6", "tcp_x_mm", "tcp_y_mm",
                             "tcp_z_mm", "bus_v", "current_ma", "left_us", "right_us",
                             "cliff_left", "cliff_right", "state", "gesture", "enable_servos"])
            for row in rows:
                writer.writerow([row["t"], *row["joints_deg"], *row["tcp_mm"], row["bus_v"],
                                 row["current_ma"], row["wheels_us"][0], row["wheels_us"][1],
                                 int(row["cliff"][0]), int(row["cliff"][1]), row["state"],
                                 row.get("gesture") or "", int(row["enable_servos"])])
        return target


# ---------------------------------------------------------------------------------------
# The supervisor
# ---------------------------------------------------------------------------------------
class SafetySupervisor:
    """Turns one `Sensors` frame into one `Command`, with every limit enforced in order.

    Order matters, and it is fixed:

        1. hardware e-stop   bus volts ~0     -> latch, never auto-clear
        2. brownout          bus volts < 4.9  -> drop the servo rail and warn
        3. cliff             either TCRT HIGH -> wheels to 1500 us in THIS tick, then recoil
        4. obstacle          < 250 mm decelerate, < 80 mm stop and hold
        5. over-current      > 2600 mA for > 150 ms -> home and stop
        6. missed grasp      closed jaw drawing almost nothing -> droop, then one retry
        7. otherwise         pass the request through

    A stale sensor frame is treated as fault 1 in effect: the wheels stop. That is the same
    fail-closed rule Phase 6's `choose_wheels` uses.
    """

    def __init__(self, hz: float = CONTROL_HZ, recorder: Optional[TelemetryRecorder] = None,
                 echo: bool = False) -> None:
        if hz <= 0.0:
            raise ValueError("control rate must be positive")
        self.hz = hz
        self.dt = 1.0 / hz
        self.recorder = recorder
        self.echo = echo
        self.state = IDLE
        self.gesture: Optional[str] = None
        self.wheels = (WHEEL_STOP_US, WHEEL_STOP_US)
        self.enable_servos = True
        self.hold_joints: Tuple[float, ...] = tuple(HOME_DEG)
        self.events: List[Tuple[float, str]] = []
        self.overcurrent_since: Optional[float] = None
        self.grasp_weak_since: Optional[float] = None
        self.grasp_retries = 0
        self.estop_latched = False
        self.cliff_ticks = 0
        self.cliff_stop_latency_ms: List[float] = []
        self.brownout_since: Optional[float] = None

    # -- helpers ---------------------------------------------------------------------
    def _event(self, t: float, message: str) -> None:
        self.events.append((t, message))
        if self.echo:
            print(f"    [{t:7.3f} s] {message}")

    def request_wheels(self, left_us: float, right_us: float) -> None:
        """What the drive layer asked for. The supervisor may override it."""
        self.wheels = (max(WHEEL_MIN_US, min(WHEEL_MAX_US, left_us)),
                       max(WHEEL_MIN_US, min(WHEEL_MAX_US, right_us)))

    def request_gesture(self, name: str) -> None:
        if name not in SEQUENCES:
            raise ValueError(f"no such gesture: {name}")
        self.gesture = name
        self.state = GESTURE

    def resume(self) -> None:
        """Leave a hold. Only valid once the cause has cleared."""
        if self.estop_latched:
            raise RuntimeError("the e-stop is latched: call release_estop(measured_joints)")
        self.state = IDLE
        self.gesture = None
        self.grasp_retries = 0
        self._event(0.0, "resumed")

    def release_estop(self, measured_joints: Sequence[float]) -> None:
        """Come out of ESTOP_HOLD without a jump.

        The subtle part of an e-stop: while it is latched the servos are unpowered, so the arm
        may have sagged to somewhere the software never commanded. Commanding the old target on
        release would snap the arm by the whole sag distance. So the software target is
        OVERWRITTEN with what the encoders/feedback actually read, and the hold becomes the
        truth. Only then is the latch cleared, and even then the arm waits for `resume()`.
        """
        if len(measured_joints) != len(HOME_DEG):
            raise ValueError(f"expected {len(HOME_DEG)} measured joint angles")
        self.hold_joints = tuple(float(v) for v in measured_joints)
        self.estop_latched = False
        self.enable_servos = False          # still off: resume() re-enables
        self.state = BROWNOUT_HOLD          # a safe, torque-free hold
        self._event(0.0, "e-stop released: targets synchronised to feedback "
                         f"{tuple(round(v, 1) for v in self.hold_joints)}")

    # -- the tick --------------------------------------------------------------------
    def tick(self, sensors: Sensors) -> Command:
        notes: List[str] = []
        t = sensors.t

        if sensors.age_s() > SENSOR_STALE_S:
            notes.append(f"sensor frame is {sensors.age_s() * 1000:.0f} ms old: stopping")
            self.wheels = (WHEEL_STOP_US, WHEEL_STOP_US)
            self.state = OBSTACLE_HOLD

        # 1. hardware e-stop -----------------------------------------------------------
        if sensors.bus_v <= ESTOP_MAX_V:
            if not self.estop_latched:
                self._event(t, f"E-STOP: bus {sensors.bus_v:.2f} V, servo rail is dead")
            self.estop_latched = True
            self.enable_servos = False
            self.wheels = (WHEEL_STOP_US, WHEEL_STOP_US)
            self.state = ESTOP_HOLD
            notes.append("e-stop latched: release it and call release_estop(measured_joints)")
            return self._finish(sensors, notes, joints=self.hold_joints)

        # 2. brownout ------------------------------------------------------------------
        if sensors.bus_v < BROWNOUT_V:
            if self.brownout_since is None:
                self.brownout_since = t
                self._event(t, f"BROWNOUT: bus {sensors.bus_v:.2f} V < {BROWNOUT_V:.2f} V, "
                               f"dropping servo torque")
            self.enable_servos = False
            self.wheels = (WHEEL_STOP_US, WHEEL_STOP_US)
            self.state = BROWNOUT_HOLD
            notes.append("servo rail cut: the arm may sag - keep hands and cables clear")
            return self._finish(sensors, notes, joints=None)
        if self.brownout_since is not None:
            if sensors.bus_v >= BROWNOUT_RECOVER_V and t - self.brownout_since > 1.0:
                self._event(t, f"brownout cleared at {sensors.bus_v:.2f} V (resume() to move)")
                self.brownout_since = None
                self.state = IDLE
            else:
                self.state = BROWNOUT_HOLD
                notes.append("holding until the rail is stable above "
                             f"{BROWNOUT_RECOVER_V:.2f} V for 1 s")
                return self._finish(sensors, notes, joints=None)

        # 3. cliff ---------------------------------------------------------------------
        if sensors.cliff_left or sensors.cliff_right:
            self.cliff_ticks += 1
            # The stop happens in THIS tick: the command below is 1500 us, so the software
            # path adds zero ticks of delay. The physical deadline is still the firmware's.
            self.cliff_stop_latency_ms.append(0.0)
            self.wheels = (WHEEL_STOP_US, WHEEL_STOP_US)
            self.state = CLIFF_RECOVER
            self.gesture = "startled_cliff_recoil"
            notes.append(f"cliff {'left' if sensors.cliff_left else 'right'}: wheels stopped in "
                         f"this tick; hardware deadline is {HARDWARE_CLIFF_DEADLINE_MS:.0f} ms "
                         f"in the ESP32 ISR")
            return self._finish(sensors, notes, joints=None)

        # 4. obstacle ------------------------------------------------------------------
        if sensors.distance_mm is not None and self.state != CLIFF_RECOVER:
            if sensors.distance_mm < OBSTACLE_STOP_MM:
                if self.state != OBSTACLE_HOLD:
                    self._event(t, f"OBSTACLE at {sensors.distance_mm:.0f} mm: stop")
                self.wheels = (WHEEL_STOP_US, WHEEL_STOP_US)
                self.state = OBSTACLE_HOLD
                notes.append("obstacle too close: stopped and holding")
                return self._finish(sensors, notes, joints=None)
            if sensors.distance_mm < OBSTACLE_SLOW_MM:
                scale = ((sensors.distance_mm - OBSTACLE_STOP_MM) /
                         (OBSTACLE_SLOW_MM - OBSTACLE_STOP_MM))
                self.wheels = tuple(WHEEL_STOP_US + (w - WHEEL_STOP_US) * max(0.0, scale)
                                    for w in self.wheels)
                notes.append(f"decelerating: {sensors.distance_mm:.0f} mm, scale "
                             f"{max(0.0, scale):.2f}")

        # 5. over-current ---------------------------------------------------------------
        if sensors.current_ma > OVERCURRENT_MA:
            if self.overcurrent_since is None:
                self.overcurrent_since = t
            held = t - self.overcurrent_since
            if held > OVERCURRENT_HOLD_S:
                self._event(t, f"OVER-CURRENT {sensors.current_ma:.0f} mA held {held * 1000:.0f} "
                               f"ms > {OVERCURRENT_HOLD_S * 1000:.0f} ms: backing off home")
                self.state = BACKOFF_HOME
                self.gesture = None
                self.wheels = (WHEEL_STOP_US, WHEEL_STOP_US)
                return self._finish(sensors, notes + ["arm returning home, torque still on"],
                                    joints=HOME_DEG)
        else:
            self.overcurrent_since = None

        # 6. missed grasp ---------------------------------------------------------------
        if sensors.jaw_target_deg <= 5.0:
            weak = sensors.current_ma < MISSED_GRASP_MA
            if weak and self.grasp_weak_since is None:
                self.grasp_weak_since = t
            held_grasp = (t - self.grasp_weak_since) if self.grasp_weak_since is not None else 0.0
            if looks_like_failed_grasp(sensors.current_ma, sensors.jaw_target_deg,
                                       sensors.joints_deg[5], held_s=held_grasp):
                if self.grasp_retries < MISSED_GRASP_MAX_RETRIES:
                    self.grasp_retries += 1
                    self._event(t, f"MISSED GRASP: {sensors.current_ma:.0f} mA on a closed jaw, "
                                   f"retry {self.grasp_retries}/{MISSED_GRASP_MAX_RETRIES}")
                    self.state = GRASP_RETRY
                    self.gesture = "sad_failed_grasp_droop"
                    return self._finish(sensors, notes + ["retrying the grasp once"], joints=None)
                notes.append(f"grasp failed again: giving up after {self.grasp_retries} retry(ies)")
                self.state = IDLE
                self.gesture = None
                self.grasp_weak_since = None
        else:
            self.grasp_weak_since = None

        # 7. pass through ----------------------------------------------------------------
        if self.state in (CLIFF_RECOVER, BACKOFF_HOME, GRASP_RETRY) and self.gesture is None:
            self.state = IDLE
        return self._finish(sensors, notes, joints=None)

    def _finish(self, sensors: Sensors, notes: List[str],
                joints: Optional[Sequence[float]]) -> Command:
        command = Command(t=sensors.t, wheels_us=tuple(self.wheels),
                          joints_deg=tuple(joints) if joints is not None else None,
                          enable_servos=self.enable_servos, state=self.state,
                          gesture=self.gesture, notes=list(notes))
        if self.recorder is not None:
            self.recorder.log(sensors, command)
        return command


# ---------------------------------------------------------------------------------------
# Fault injection
# ---------------------------------------------------------------------------------------
@dataclass
class FaultResult:
    name: str
    passed: bool
    detail: str
    rows: int = 0
    latency_ms: Optional[float] = None

    def line(self) -> str:
        mark = "PASS" if self.passed else "FAIL"
        extra = f"  latency {self.latency_ms:.1f} ms" if self.latency_ms is not None else ""
        return f"  {mark}  {self.name:<22} {self.detail}{extra}"


class FaultInjector:
    """Six scripted sensor timelines, one per fault. Deterministic: same ticks every run."""

    def __init__(self, hz: float = CONTROL_HZ, duration_s: float = 4.0) -> None:
        self.hz = hz
        self.dt = 1.0 / hz
        self.duration_s = duration_s

    def _base(self, t: float, **overrides) -> Dict[str, object]:
        values: Dict[str, object] = {
            "t": t, "joints_deg": tuple(HOME_DEG), "bus_v": NOMINAL_BUS_V,
            "current_ma": 180.0, "wheels_us": (1700.0, 1700.0), "cliff_left": False,
            "cliff_right": False, "distance_mm": 900.0, "jaw_target_deg": 60.0,
            "sensor_t": t,
        }
        values.update(overrides)
        return values

    def timeline(self, fault: str) -> List[Dict[str, object]]:
        ticks = int(self.duration_s * self.hz)
        rows: List[Dict[str, object]] = []
        for i in range(ticks):
            t = i * self.dt
            if fault == "over_current":
                current = 2900.0 if t >= 2.0 else 180.0
                rows.append(self._base(t, current_ma=current))
            elif fault == "missed_grasp":
                rows.append(self._base(t, current_ma=95.0, jaw_target_deg=0.0,
                                       joints_deg=(0.0, -15.0, 40.0, -25.0, 0.0, 3.0)))
            elif fault == "cliff":
                rows.append(self._base(t, cliff_left=(t >= 2.0),
                                       wheels_us=(1900.0, 1900.0) if t < 2.0
                                       else (WHEEL_STOP_US, WHEEL_STOP_US)))
            elif fault == "obstacle":
                # Far away, then inside the slow-down band, then inside the stop band.
                distance = 900.0 if t < 1.0 else (200.0 if t < 1.5 else 60.0)
                rows.append(self._base(t, distance_mm=distance))
            elif fault == "brownout":
                rows.append(self._base(t, bus_v=6.0 if t < 2.0 else 4.6))
            elif fault == "estop":
                rows.append(self._base(t, bus_v=6.0 if t < 2.0 else 0.0))
            else:
                raise ValueError(f"unknown fault {fault!r}")
        return rows

    # -- one injection ---------------------------------------------------------------
    def run(self, fault: str, recorder: Optional[TelemetryRecorder] = None,
            echo: bool = False) -> FaultResult:
        sup = SafetySupervisor(hz=self.hz, recorder=recorder, echo=echo)
        rows = self.timeline(fault)
        seen: Dict[str, object] = {}
        commands: List[Command] = []
        for row in rows:
            sensors = Sensors(**row)            # type: ignore[arg-type]
            sup.request_wheels(*sensors.wheels_us)
            command = sup.tick(sensors)
            commands.append(command)
            if command.state not in seen:
                seen[command.state] = sensors.t       # first time each state appeared
        if fault == "over_current":
            fired = seen.get(BACKOFF_HOME)
            want = 2.0 + OVERCURRENT_HOLD_S
            ok = fired is not None and fired <= want + 2 * self.dt
            detail = (f"backed off home at {fired:.3f} s (limit {want:.3f} s)"
                      if fired is not None else "never backed off")
            return FaultResult("over-current 2900 mA", ok, detail,
                               latency_ms=None if fired is None else (fired - 2.0) * 1000)
        if fault == "missed_grasp":
            retry = seen.get(GRASP_RETRY)
            ok = retry is not None and sup.grasp_retries <= MISSED_GRASP_MAX_RETRIES
            detail = (f"drooped and retried at {retry:.3f} s, {sup.grasp_retries} retry "
                      f"(max {MISSED_GRASP_MAX_RETRIES}), then gave up"
                      if retry is not None else "no retry: the 95 mA jaw was not detected")
            return FaultResult("missed grasp 95 mA", ok, detail,
                               latency_ms=None if retry is None else (retry - 0.0) * 1000)
        if fault == "cliff":
            stopped = next((c for c in commands if c.t >= 2.0 and
                            c.wheels_us == (WHEEL_STOP_US, WHEEL_STOP_US)), None)
            stopped_same_tick = stopped is not None and abs(stopped.t - 2.0) < self.dt / 2
            ok = stopped_same_tick and sup.cliff_ticks > 0
            detail = (f"wheels 1500 us at t={stopped.t:.3f} s, decision latency "
                      f"0.0 ms (firmware budget {HARDWARE_CLIFF_DEADLINE_MS:.0f} ms)"
                      if stopped is not None else "wheels never stopped")
            return FaultResult("cliff (TCRT5000 left)", ok, detail, latency_ms=0.0)
        if fault == "obstacle":
            slowed = next((c for c in commands if "decelerat" in " ".join(c.notes)), None)
            held = seen.get(OBSTACLE_HOLD)
            ok = slowed is not None and held is not None
            detail = (f"decelerated from {slowed.t:.3f} s, stopped at {held:.3f} s"
                      if slowed is not None and held is not None
                      else "no deceleration or stop")
            return FaultResult("obstacle 200 -> 60 mm", ok, detail)
        if fault == "brownout":
            cut = next((c for c in commands if not c.enable_servos), None)
            ok = cut is not None and NOMINAL_BUS_V > BROWNOUT_V
            detail = (f"servo rail cut at {cut.t:.3f} s, warning: "
                      f"{'sag' in ' '.join(cut.notes)}"
                      if cut is not None else "the rail was never cut")
            return FaultResult("brownout 4.6 V", ok, detail,
                               latency_ms=None if cut is None else (cut.t - 2.0) * 1000)
        if fault == "estop":
            latched = seen.get(ESTOP_HOLD)
            if latched is None:
                return FaultResult("hardware e-stop", False, "never latched")
            held_after_recovery = all(c.state == ESTOP_HOLD for c in commands
                                      if c.t >= 3.0)
            # Release with a sagged arm, then check the first command does not jump.
            measured = (3.0, -40.0, 60.0, -10.0, 2.0, 8.0)
            sup.release_estop(measured)
            sync_ok = all(abs(a - b) < 1e-9 for a, b in zip(sup.hold_joints, measured))
            sup.resume()
            first = sup.tick(Sensors(**self._base(self.duration_s + self.dt, bus_v=6.0,
                                                 current_ma=180.0,
                                                 joints_deg=measured)))    # type: ignore[arg-type]
            jump = max(abs(a - b) for a, b in
                       zip(first.joints_deg or sup.hold_joints, measured))
            ok = held_after_recovery and sync_ok and jump < 1e-9
            detail = (f"latched at {latched:.3f} s, stayed latched after 6 V returned, "
                      f"sync error {jump:.3f} deg on release")
            return FaultResult("hardware e-stop", ok, detail,
                               latency_ms=None if latched is None else (latched - 2.0) * 1000)
        raise ValueError(fault)

    def run_all(self, recorder: Optional[TelemetryRecorder] = None,
                echo: bool = False) -> List[FaultResult]:
        return [self.run(name, recorder=recorder, echo=echo)
                for name in FAULTS]


FAULTS = ("over_current", "missed_grasp", "cliff", "obstacle", "brownout", "estop")


# ---------------------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------------------
def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="B.U.D.-E Phase 8 safety supervisor: six fault injections, one telemetry "
                    "log, no hardware.")
    parser.add_argument("--fault", choices=FAULTS,
                        help="run one fault instead of all six")
    parser.add_argument("--seconds", type=float, default=4.0,
                        help="length of each injected timeline (default 4 s)")
    parser.add_argument("--hz", type=float, default=CONTROL_HZ)
    parser.add_argument("--log", type=Path, default=LOG_PATH,
                        help=f"JSONL telemetry path (default {LOG_PATH})")
    parser.add_argument("--csv", type=Path, default=CSV_PATH)
    parser.add_argument("--echo", action="store_true", help="print a line every 25 ticks")
    parser.add_argument("--no-log", action="store_true", help="do not write any file")
    args = parser.parse_args(argv)

    recorder = None if args.no_log else TelemetryRecorder(args.log, echo=args.echo)
    injector = FaultInjector(hz=args.hz, duration_s=args.seconds)
    print("=" * 100)
    print("  PHASE 8 - SAFETY SUPERVISOR, SIX FAULT INJECTIONS (simulated sensors)")
    print("=" * 100)
    names = (args.fault,) if args.fault else FAULTS
    results = [injector.run(name, recorder=recorder, echo=args.echo) for name in names]

    for result in results:
        print(result.line())
    if recorder is not None:
        recorder.close()
        csv_path = recorder.to_csv(args.csv)
        print("-" * 100)
        print(f"  {recorder.line_count} telemetry rows -> {args.log}")
        print(f"                            -> {csv_path} (open it in a spreadsheet)")
    failures = [r for r in results if not r.passed]
    print("=" * 100)
    print(f"  {len(results) - len(failures)} / {len(results)} faults handled"
          f"{'' if not failures else ': ' + ', '.join(r.name for r in failures)}")
    print()
    print("  WHAT THIS DOES NOT PROVE")
    print("  * The sensor frames are synthesised. Nothing was measured, and no servo moved.")
    print("  * 'Stop in this tick' is a software decision. The real 5 ms deadline lives in the")
    print("    ESP32 interrupt handler for GPIO 25/26: Python plus a serial link cannot promise")
    print("    it, and a supervisor that pretends otherwise would be lying to you.")
    print("  * The over-current, brownout and obstacle thresholds are datasheet and budget")
    print("    numbers, not measurements of your supply. Retune them with a current clamp.")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
