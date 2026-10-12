"""OFFLINE mobile navigation policy prototype. Produces abstract wheel speeds only.

Not wired to motors or safe for tabletop deployment. Camera alone cannot see
all hazards; no cliff sensor is sufficient as sole protection against falling.
"""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class Reading:
    now_s: float
    sensor_time_s: float
    target_bearing_rad: float | None
    obstacle_m: float | None
    left_floor_ok: bool
    right_floor_ok: bool
    estop_ok: bool
    arm_stowed: bool


def choose_wheels(reading: Reading) -> tuple[float,float]:
    """Return normalized left/right commands (-0.15..0.15). Fail closed."""
    if (not reading.estop_ok or not reading.arm_stowed or
        not reading.left_floor_ok or not reading.right_floor_ok or
        reading.now_s < reading.sensor_time_s or
        reading.now_s-reading.sensor_time_s > 0.1 or
        reading.obstacle_m is None or not math.isfinite(reading.obstacle_m) or
        reading.obstacle_m < 0.30 or reading.target_bearing_rad is None or
        not math.isfinite(reading.target_bearing_rad)):
        return (0.,0.)
    turn=max(-0.12,min(0.12, reading.target_bearing_rad*0.2))
    fwd=0.08 if abs(reading.target_bearing_rad)<0.3 else 0.
    return (max(-0.15,min(0.15,fwd-turn)), max(-0.15,min(0.15,fwd+turn)))


if __name__=='__main__':
    ok=Reading(10.,9.95,0.1,0.8,True,True,True,True)
    print('normal offline command:',choose_wheels(ok))
    for unsafe in (Reading(10.,9.,0.1,0.8,True,True,True,True),
                   Reading(10.,9.95,0.1,0.8,False,True,True,True),
                   Reading(10.,9.95,0.1,0.1,True,True,True,True)):
        assert choose_wheels(unsafe)==(0.,0.)
    print('stale, cliff and obstacle checks: STOP (offline only)')
