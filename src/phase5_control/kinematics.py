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


if __name__ == '__main__':
    for q in [(0,-15,40,-25,0), (20,45,35,0,15), (-25,55,50,25,-30)]:
        p = forward(q)
        found = inverse(p.x,p.y,p.z,p.pitch,p.roll)
        assert any(max(abs(a-b) for a,b in zip(q,s)) < 1e-6 for s in found), (q,found)
        print('FK/IK round trip', q, '=>', (round(p.x,2),round(p.y,2),round(p.z,2)))
