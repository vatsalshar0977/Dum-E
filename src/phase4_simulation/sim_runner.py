#!/usr/bin/env python3
"""Schematic FK demonstration or optional actual PyBullet URDF run.

The stdlib demo is NOT physics. PyBullet uses simple boxes, not the unverified STLs.
"""
import argparse
import math
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from src.phase5_control.kinematics import forward
from src.phase5_control.trajectory import plan
URDF = ROOT / 'urdf' / 'bude_arm.urdf'
HOME = (0., -15., 40., -25., 0.)
GOAL = (15., 0., 35., -15., 20.)

def demo():
    for angle in (15., -15., 0.):
        q = (angle,) + HOME[1:]
        p = forward(q)
        print(f'kinematic J1 {angle:+.0f} deg: TCP ({p.x:.1f}, {p.y:.1f}, {p.z:.1f}) mm')
    path = plan(HOME, GOAL)
    print(f'kinematic example: {len(path)} checked centreline samples, {path[-1][0]:.2f}s')
    print('NOT a dynamics/collision/real-hardware validation.')

def bullet(gui=False):
    try: import pybullet as pb
    except ImportError as exc: raise SystemExit('Install optional dependency: pip install pybullet') from exc
    import time
    cid = pb.connect(pb.GUI if gui else pb.DIRECT)
    try:
        robot = pb.loadURDF(str(URDF), useFixedBase=True)
        ids = {pb.getJointInfo(robot,i)[1].decode(): i for i in range(pb.getNumJoints(robot))}
        names = ['base_yaw','shoulder_pitch','elbow_pitch','wrist_pitch','wrist_roll']
        assert set(names).issubset(ids) and 'tcp_fixed' in ids
        pb.setGravity(0,0,-9.81)
        q0 = tuple(math.radians(a) for a in HOME)
        for name,a in zip(names,q0): pb.resetJointState(robot, ids[name], a)
        path = plan(HOME, GOAL, sample_hz=50)
        # Physically commanded position motors, step at 240 Hz. These illustrative
        # effort limits do NOT establish real servo capabilities or safe collision forces.
        duration = path[-1][0]; step = 1/240
        for n in range(math.ceil(duration/step)+1):
            t = min(duration,n*step)
            j = min(round(t*50),len(path)-1)
            for name,deg in zip(names,path[j][1]):
                pb.setJointMotorControl2(robot, ids[name], pb.POSITION_CONTROL,
                                         targetPosition=math.radians(deg), force=0.08)
            pb.stepSimulation()
            if gui: time.sleep(step)
        pos = pb.getLinkState(robot, ids['tcp_fixed'])[4]
        wanted = forward(GOAL)
        print('PyBullet stepped', n+1, 'ticks. TCP measured (mm):', tuple(round(1000*v,1) for v in pos))
        print('Schematic FK target (mm):', tuple(round(v,1) for v in (wanted.x,wanted.y,wanted.z)))
        error = math.dist(tuple(1000*v for v in pos), (wanted.x,wanted.y,wanted.z))
        print(f'Proxy tracking error: {error:.2f} mm (not a grasp/collision/pass certificate)')
        if error > 10: raise RuntimeError('proxy simulator did not track commanded pose')
    finally: pb.disconnect(cid)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', choices=['kinematic','pybullet'], default='kinematic')
    parser.add_argument('--gui', action='store_true')
    args = parser.parse_args()
    if args.gui and args.backend != 'pybullet': parser.error('--gui requires --backend pybullet')
    demo() if args.backend == 'kinematic' else bullet(args.gui)
