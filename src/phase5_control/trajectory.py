"""Quintic joint interpolation with conservative *schematic* tabletop checks.

This does not do real mesh self-collision, dynamic loading or human avoidance.
Never connect it to physical servos without independently verified sensing/limits.
"""
import math
from .kinematics import LIMITS, forward

SPEED = (35., 25., 35., 40., 40.)  # illustrative deg/s, tune on real hardware

def check(q):
    if len(q) != 5 or any(not math.isfinite(a) or a < lo or a > hi
                          for a,(lo,hi) in zip(q, LIMITS)):
        raise ValueError('non-finite angle or joint limit violated')
    p = forward(q)
    if min(p.z, p.elbow_z, p.wrist_z) < 20:
        raise ValueError('schematic link centre below 20 mm tabletop margin')
    return p

def plan(start, goal, sample_hz=50):
    check(start); check(goal)
    if not (1 <= sample_hz <= 200): raise ValueError('unsupported sample rate')
    # Quintic max derivative = 1.875. Endpoint speeds are zero.
    duration = max(0.5, *(1.875*abs(b-a)/v for a,b,v in zip(start,goal,SPEED)))
    count = math.ceil(duration*sample_hz)
    result = []
    for i in range(count+1):
        t = i/count
        s = t*t*t*(10+t*(-15+6*t))
        q = tuple(a+(b-a)*s for a,b in zip(start,goal))
        check(q)  # raises; caller must discard entire path, not run partial plan
        result.append((duration*t, q))
    return result

if __name__ == '__main__':
    home = (0,-15,40,-25,0)
    path = plan(home, (15,0,35,-15,20))
    print(f'Planned {len(path)} schematic samples in {path[-1][0]:.2f}s')
