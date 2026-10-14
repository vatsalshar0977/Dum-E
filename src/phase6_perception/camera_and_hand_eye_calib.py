#!/usr/bin/env python3
"""Camera calibration and measured ray-to-plane projection (mm). No assumed camera pose.

Use `calibrate --images path/to/chessboard/*.png` for intrinsics; measured
camera-to-base extrinsics must be provided separately. Intrinsics are camera-specific.
"""
import argparse
import glob
import json
import math
from pathlib import Path
import sys

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from src.phase5_control.kinematics import forward  # noqa: E402


def intersect_table(u, v, intrinsics, transform, z_mm=0.):
    """Return XYZ in base mm; transform={'R': 3x3 camera->base, 't_mm': 3-vector}.
    Coordinates: OpenCV optical x right, y down, z forward; base z up.
    """
    fx,fy,cx,cy = (float(intrinsics[k]) for k in ('fx','fy','cx','cy'))
    if fx <= 0 or fy <= 0: raise ValueError('focal lengths must be positive')
    R, t = transform['R'],transform['t_mm']
    if len(R)!=3 or any(len(row)!=3 for row in R) or len(t)!=3:
        raise ValueError('expected 3x3 R and 3-vector t')
    if not all(math.isfinite(v) for v in (u,v,z_mm,fx,fy,cx,cy,*t,*(e for row in R for e in row))):
        raise ValueError('non-finite calibration')
    # Verify rotation is orthonormal, otherwise arbitrary transforms look convincing.
    for i in range(3):
        for j in range(3):
            dot=sum(R[i][k]*R[j][k] for k in range(3))
            if abs(dot-(1 if i==j else 0))>1e-3: raise ValueError('R not orthonormal')
    det = (R[0][0]*(R[1][1]*R[2][2]-R[1][2]*R[2][1])-
           R[0][1]*(R[1][0]*R[2][2]-R[1][2]*R[2][0])+
           R[0][2]*(R[1][0]*R[2][1]-R[1][1]*R[2][0]))
    if abs(det-1)>1e-3: raise ValueError('R must be proper rotation')
    ray=((u-cx)/fx,(v-cy)/fy,1.)
    direction=[sum(R[i][k]*ray[k] for k in range(3)) for i in range(3)]
    if abs(direction[2])<1e-9: raise ValueError('ray parallel to tabletop')
    scale=(z_mm-t[2])/direction[2]
    if scale <= 0: raise ValueError('plane behind camera')
    return tuple(t[i]+scale*direction[i] for i in range(3))


def calibrate(paths, cols, rows, square_mm):
    """Calibrate intrinsics from a measured planar chessboard; return K and RMS error.
    cols/rows are INNER corners. Bad images are skipped, <8 valid frames rejected.
    """
    try: import cv2
    except ImportError as e: raise RuntimeError('Install: pip install opencv-python') from e
    import numpy as np
    if cols<3 or rows<3 or square_mm<=0: raise ValueError('invalid chessboard dimensions')
    pattern=np.zeros((cols*rows,3),np.float32)
    pattern[:,:2]=np.mgrid[0:cols,0:rows].T.reshape(-1,2)*square_mm
    objpoints=[]; imgpoints=[]; size=None
    for path in paths:
        img=cv2.imread(str(path))
        if img is None: continue
        gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        if size is not None and (gray.shape[1],gray.shape[0])!=size:
            raise ValueError('all photos must use same resolution')
        size=gray.shape[1],gray.shape[0]
        ok, corners=cv2.findChessboardCorners(gray,(cols,rows))
        if ok:
            corners=cv2.cornerSubPix(gray,corners,(5,5),(-1,-1),
                (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,30,0.001))
            objpoints.append(pattern); imgpoints.append(corners)
    if len(objpoints)<8: raise ValueError(f'Need >=8 varied sharp board views; found {len(objpoints)}')
    rms,K,dist,_,_=cv2.calibrateCamera(objpoints,imgpoints,size,None,None)
    return {'fx':float(K[0,0]),'fy':float(K[1,1]),'cx':float(K[0,2]),
            'cy':float(K[1,2]),'distortion':dist.ravel().tolist(),
            'rms_px':float(rms),'frames_used':len(objpoints),'image_size':size}

# ---------------------------------------------------------------------------------------
# NOMINAL DESIGN-MODEL CAMERA (not a calibration!)
# ---------------------------------------------------------------------------------------
# `nominal_intrinsics` and `camera_pose_from_fk` exist so the simulator and the dry-run
# pipeline have something self-consistent to run on before any camera is bought. The values
# they produce describe the DESIGN, and a real camera will differ - field of view varies
# between units of the same part number, focus changes with the lens, and every mounting
# tolerance shifts the extrinsics. Replace them with `calibrate` output and a measured
# hand-eye transform before you trust a single pixel to millimetre conversion.

NOMINAL_IMAGE_SIZE_PX = (640, 480)
NOMINAL_HFOV_DEG = 62.0

# Camera offset from the tool centre point, expressed in the TCP frame as
# (along x_T, along y_T, along z_T) in mm, where z_T is the tool axis pointing out past the
# claw and y_T is the in-plane "up" direction.
#
# Derivation from 08_gripper_body_and_cam_mount.stl, in that part's own coordinates: the
# palm is at x = 0, the fingers reach to about x = 46, the camera bezel spans x = -13..-3
# and is raised to z = 38. Taking the grasp point near the finger mid-span, the bezel sits
# roughly 43 mm BEHIND the grasp point (-z_T) and about 30 mm above it (+y_T).
#
# THIS IS A DESIGN INTENT, NOT A CALIPER READING, and the printed part has never been
# measured. Put the real bracket under calipers and update these three numbers before you
# localise anything with a camera.
CAMERA_MOUNT_OFFSET_MM = (0.0, 30.0, -43.0)


def nominal_intrinsics(width_px: int = NOMINAL_IMAGE_SIZE_PX[0],
                       height_px: int = NOMINAL_IMAGE_SIZE_PX[1],
                       hfov_deg: float = NOMINAL_HFOV_DEG) -> dict:
    """Pinhole intrinsics implied by a claimed sensor width and horizontal field of view.

    fx = fy = (width/2) / tan(hfov/2) assumes square pixels and no skew - true enough for
    the design study, and exactly the kind of assumption that a real calibration replaces.
    """
    if width_px <= 0 or height_px <= 0:
        raise ValueError('image size must be positive')
    if not 1.0 <= hfov_deg <= 179.0:
        raise ValueError('hfov_deg must be between 1 and 179')
    fx = (width_px / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    return {'fx': fx, 'fy': fx, 'cx': width_px / 2.0, 'cy': height_px / 2.0,
            'image_size': (width_px, height_px), 'hfov_deg': hfov_deg,
            'source': 'nominal design model - NOT a calibration'}


def _tcp_frame(pose):
    """Orthonormal TCP frame in base coordinates: z along the tool, x horizontal.

    Base frame is z-up with x forward at zero yaw. Returns (x_axis, y_axis, z_axis).
    """
    pitch, yaw = math.radians(pose.pitch), math.radians(math.atan2(pose.y, pose.x))
    z_axis = (math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw),
              math.sin(pitch))
    x_axis = (-math.sin(yaw), math.cos(yaw), 0.0)
    y_axis = (z_axis[1] * x_axis[2] - z_axis[2] * x_axis[1],
              z_axis[2] * x_axis[0] - z_axis[0] * x_axis[2],
              z_axis[0] * x_axis[1] - z_axis[1] * x_axis[0])
    return x_axis, y_axis, z_axis


def camera_pose_from_fk(joint_angles_deg, mount_offset_mm=CAMERA_MOUNT_OFFSET_MM):
    """Camera transform implied by the DESIGN at a given joint configuration.

    Builds the camera-to-base rotation R and the camera origin t in base mm, in the
    OpenCV convention (x right, y down, z along the optical axis). The optical axis is
    taken to look along the tool direction past the claw, which is how the printed bezel
    is drawn.

    THIS IS A KINEMATIC MODEL, NOT HAND-EYE CALIBRATION. It is only as good as the link
    lengths, the mount offset and the zero offsets of your servos - none of which have been
    measured. A real hand-eye transform must be estimated from images of a fiducial taken
    at several poses with recorded joint states.
    """
    if len(joint_angles_deg) != 5:
        raise ValueError('expected five joint angles in degrees')
    pose = forward(tuple(joint_angles_deg))
    x_axis, y_axis, z_axis = _tcp_frame(pose)
    offset = tuple(float(v) for v in mount_offset_mm)
    if len(offset) != 3 or any(not math.isfinite(v) for v in offset):
        raise ValueError('mount offset must be three finite numbers')
    origin = tuple((pose.x, pose.y, pose.z)[i]
                   + offset[0] * x_axis[i] + offset[1] * y_axis[i] + offset[2] * z_axis[i]
                   for i in range(3))
    R = [[x_axis[i], y_axis[i], z_axis[i]] for i in range(3)]
    return {'R': R, 't_mm': list(origin),
            'source': 'design-model FK extrinsics - NOT hand-eye calibration'}


def project_point_xyz(point_mm, intrinsics, transform):
    """Project a base-frame point to a pixel. Inverse of `intersect_table` for a pinhole.

    Provided so calibration code can be checked for self-consistency (project a known point,
    then project the ray back), and so the simulator can draw what a camera would see.
    """
    fx, fy = float(intrinsics['fx']), float(intrinsics['fy'])
    cx, cy = float(intrinsics['cx']), float(intrinsics['cy'])
    R, t = transform['R'], transform['t_mm']
    d = [point_mm[i] - t[i] for i in range(3)]
    # R maps CAMERA coordinates to BASE coordinates (its columns are the camera axes
    # expressed in the base frame), so going the other way uses the transpose.
    cam = [sum(R[k][i] * d[k] for k in range(3)) for i in range(3)]
    if cam[2] <= 1e-9:
        raise ValueError('point is behind the camera')
    if not all(math.isfinite(v) for v in cam):
        raise ValueError('non-finite projection')
    return fx * cam[0] / cam[2] + cx, fy * cam[1] / cam[2] + cy


def self_test_projection_round_trip():
    """Check the algebra: FK -> extrinsics -> pixel -> ray -> plane must close the loop.

    Returns (worst_residual_mm, points_checked). A tight residual says the CODE is
    self-consistent. It says nothing whatsoever about whether a real camera, a real bracket
    or a real desk agrees with the model - only measured photos can say that.
    """
    worst = 0.0
    checked = 0
    intrinsics = nominal_intrinsics()
    # Configurations that aim the tool DOWN at the workbench, taken from IK solutions for
    # real desk targets, so the desk points are actually in front of the lens.
    looking_down = ((0.0, 45.4, 47.3, 72.3, 0.0),      # TCP pitch -75 deg
                    (0.0, 40.3, 57.5, 62.2, 0.0),      # TCP pitch -70 deg
                    (0.0, 32.7, 74.1, 43.2, 0.0),      # TCP pitch -60 deg
                    (22.8, 47.6, 65.7, 31.7, 25.0))    # 
    for q in looking_down:
        transform = camera_pose_from_fk(q)
        for dz in (0.0, 12.0, 30.0):
            # Sample around wherever the optical axis actually meets this plane. A 62 deg
            # lens aimed steeply down sees a small patch, so testing a fixed list of desk
            # points would mostly test points outside the frame.
            axis = [transform['R'][i][2] for i in range(3)]   # camera z in base coords
            if axis[2] >= -1e-6:
                continue                                      # looking up; no desk hit
            scale = (dz - transform['t_mm'][2]) / axis[2]
            hit = tuple(transform['t_mm'][i] + scale * axis[i] for i in range(3))
            for dxy in ((0.0, 0.0), (25.0, 20.0), (-30.0, -15.0), (0.0, 40.0)):
                point = (hit[0] + dxy[0], hit[1] + dxy[1], dz)
                u, v = project_point_xyz(point, intrinsics, transform)
                if not (0.0 <= u < 640.0 and 0.0 <= v < 480.0):
                    continue                      # outside the frame; not a valid check
                back = intersect_table(u, v, intrinsics, transform, z_mm=dz)
                worst = max(worst, math.dist(point, back))
                checked += 1
    if checked < 12:
        raise RuntimeError(f'only {checked} points landed inside the frame; '
                           'the self-test is not exercising the projection')
    return worst, checked


if __name__=='__main__':
    if len(sys.argv) == 1:
        k = nominal_intrinsics()
        print('NOMINAL design-model intrinsics (NOT a calibration):')
        print(f"  fx = fy = {k['fx']:.2f} px at {k['image_size'][0]}x{k['image_size'][1]} "
              f"and {k['hfov_deg']:.0f} deg HFOV")
        print(f"  cx, cy = {k['cx']:.1f}, {k['cy']:.1f}")
        print()
        print('Design-model camera pose at the home configuration:')
        t = camera_pose_from_fk((0.0, -15.0, 40.0, -25.0, 0.0))
        print('  origin (mm):', [round(v, 2) for v in t['t_mm']])
        print()
        residual, count = self_test_projection_round_trip()
        print(f"Projection round-trip: {count} points, worst residual {residual:.3e} mm")
        print('That number certifies the ALGEBRA only. A real camera needs real photos:')
        print('  python src/phase6_perception/camera_and_hand_eye_calib.py calibrate \\')
        print("      --images 'calibration_photos/*.png' --square-mm <measured> \\")
        print('      --cols 9 --rows 6 --output camera_intrinsics.json')
        sys.exit(0)

    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest='cmd',required=True)
    c=sub.add_parser('calibrate'); c.add_argument('--images',required=True,help='quoted glob of photos'); c.add_argument('--cols',type=int,default=9); c.add_argument('--rows',type=int,default=6)
    c.add_argument('--square-mm',type=float,required=True); c.add_argument('--output',default='camera_intrinsics.json')
    a=p.parse_args()
    result=calibrate(glob.glob(a.images),a.cols,a.rows,a.square_mm)
    Path(a.output).write_text(json.dumps(result,indent=2)+'\n')
    print('saved',a.output,'RMS px:',result['rms_px'],'frames:',result['frames_used'])
