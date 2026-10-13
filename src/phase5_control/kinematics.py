"""Schematic 5-DOF kinematics in millimetres and degrees, matching urdf/bude_arm.urdf.

No assertion here establishes that the printed parts, pulse centres or camera agree with it.
IK rejects unreachable/unsafe requests rather than silently moving to a different target.
"""
import math
from dataclasses import dataclass

H, UPPER, FOREARM, TOOL = 55.0, 80.0, 75.0, 70.0
LIMITS = ((-90., 90.), (-75., 85.), (-90., 90.), (-85., 85.), (-90., 90.))

@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    pitch: float  # degrees from horizontal, + means pointing up
    roll: float
    elbow_z: float
    wrist_z: float


def forward(q):
    """q=(yaw, shoulder, elbow, wrist_pitch, wrist_roll) in degrees."""
    if len(q) != 5 or not all(math.isfinite(x) for x in q):
        raise ValueError('expected five finite angles')
    yaw, shoulder, elbow, wrist, roll = map(math.radians, q)
    a, b, c = shoulder, shoulder + elbow, shoulder + elbow + wrist
    r = UPPER*math.sin(a) + FOREARM*math.sin(b) + TOOL*math.sin(c)
    ez = H + UPPER*math.cos(a)
    wz = ez + FOREARM*math.cos(b)
    z = wz + TOOL*math.cos(c)
    return Pose(r*math.cos(yaw), r*math.sin(yaw), z,
                90-math.degrees(c), math.degrees(roll), ez, wz)


def inverse(x, y, z, pitch, roll=0):
    """Solve a desired TCP and pitch. Return valid elbow branches; raise if none.

    A yaw has two equivalent orientations for negative radial reach; this solver
    only admits forward radial poses (r>=0). Choose a different approach if needed.
    """
    vals = (x, y, z, pitch, roll)
    if not all(math.isfinite(v) for v in vals): raise ValueError('non-finite target')
    yaw = math.degrees(math.atan2(y, x))
    r = math.hypot(x, y)
    phi = math.radians(pitch)
    dx, dz = r - TOOL*math.cos(phi), z - H - TOOL*math.sin(phi)
    cosine = (dx*dx + dz*dz - UPPER*UPPER - FOREARM*FOREARM)/(2*UPPER*FOREARM)
    if cosine < -1-1e-9 or cosine > 1+1e-9:
        raise ValueError('target outside two-link wrist workspace')
    solutions = []
    for sign in (1, -1):
        elbow = sign*math.degrees(math.acos(max(-1, min(1, cosine))))
        # In horizontal coordinates elbow subtracts from shoulder's first-link angle.
        theta2 = math.atan2(dz, dx) + math.atan2(FOREARM*math.sin(math.radians(elbow)),
                                                 UPPER+FOREARM*math.cos(math.radians(elbow)))
        shoulder = 90-math.degrees(theta2)
        wrist = 90-pitch-shoulder-elbow
        q = (yaw, shoulder, elbow, wrist, roll)
        if any(a < lo-1e-7 or a > hi+1e-7 for a, (lo, hi) in zip(q, LIMITS)):
            continue
        p = forward(q)
        if math.dist((p.x,p.y,p.z),(x,y,z)) < 1e-5:
            solutions.append(q)
    if not solutions: raise ValueError('no solution within joint limits at requested pitch')
    return solutions


def forward_kinematics_3d(q):
    """Named alias for `forward`. Returns the tool centre point plus intermediate heights."""
    return forward(q)


# Pitch offsets tried, in order, by the adaptive IK search. The search walks outwards from
# the requested approach angle so the closest achievable one wins.
PITCH_ADAPTATION_STEP_DEG = 5.0
PITCH_ADAPTATION_MAX_DEG = 30.0
PITCH_ADAPTATION_LADDER_DEG = tuple(
    sign * PITCH_ADAPTATION_STEP_DEG * step
    for step in range(1, int(PITCH_ADAPTATION_MAX_DEG // PITCH_ADAPTATION_STEP_DEG) + 1)
    for sign in (-1.0, 1.0)
)  # +-5, +-10, ... +-30 deg


def inverse_kinematics_5dof(x, y, z, pitch, roll=0.0, adapt_pitch=False):
    """Named entry point for the 5-DOF inverse solver.

    Default behaviour is FAIL CLOSED: `inverse` raises when the requested pose is outside
    reach or outside joint limits, and this function passes that error straight through.
    That is deliberate. A solver that quietly returns "the nearest pose I could find" will
    move a real arm somewhere the caller did not ask for.

    Pass `adapt_pitch=True` when you genuinely want the nearest reachable approach angle
    (for example, an angled screwdriver on a desk). The search walks the ladder above out
    to +-30 deg and returns every solution it found, each of which reproduces its OWN
    pitch exactly. Callers must read `forward(q).pitch` to learn which pitch they got and
    decide whether that is acceptable; nothing is silently substituted.

    Returns a list of (q, achieved_pitch_deg) sorted by how far the pitch moved from the
    request. Raises ValueError if nothing is reachable.
    """
    if not adapt_pitch:
        return [(q, forward(q).pitch) for q in inverse(x, y, z, pitch, roll)]

    candidates = []
    for delta in (0.0,) + PITCH_ADAPTATION_LADDER_DEG:
        try:
            found = inverse(x, y, z, pitch + delta, roll)
        except ValueError:
            continue
        for q in found:
            # De-duplicate on the JOINT VECTOR, not on the pitch. Two different elbow
            # branches can share an approach angle, and keying on the pitch would throw one
            # of them away - silently removing the choice the caller asked for.
            if any(max(abs(a - b) for a, b in zip(q, existing)) < 1e-9
                   for existing, _ in candidates):
                continue
            candidates.append((q, forward(q).pitch))
    if not candidates:
        raise ValueError(
            f'no reachable solution for ({x}, {y}, {z}) within +-30 deg of pitch {pitch}'
        )
    candidates.sort(key=lambda item: abs(item[1] - pitch))
    return candidates


if __name__ == '__main__':
    for q in [(0,-15,40,-25,0), (20,45,35,0,15), (-25,55,50,25,-30)]:
        p = forward_kinematics_3d(q)
        found = inverse_kinematics_5dof(p.x, p.y, p.z, p.pitch, p.roll)
        assert any(
            max(abs(a-b) for a, b in zip(q, solution)) < 1e-6 for solution, _ in found
        ), (q, found)
        print('FK/IK round trip', q, '=>', (round(p.x,2),round(p.y,2),round(p.z,2)))

    # Fail-closed on an unreachable target, and the opt-in ladder on a tilted approach.
    try:
        inverse_kinematics_5dof(500.0, 0.0, 20.0, 0.0)
    except ValueError as exc:
        print('unreachable target raises:', exc)
    try:
        found = inverse_kinematics_5dof(150.0, 0.0, 30.0, -60.0, adapt_pitch=True)
        print('pitch adaptation found', len(found), 'branch(es); nearest achieved pitch',
              round(found[0][1], 1), 'deg (asked for -60.0)')
    except ValueError as exc:
        print('pitch adaptation found nothing:', exc)
