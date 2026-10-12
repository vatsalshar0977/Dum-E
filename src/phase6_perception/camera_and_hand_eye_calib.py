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

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest='cmd',required=True)
    c=sub.add_parser('calibrate'); c.add_argument('--images',required=True,help='quoted glob of photos'); c.add_argument('--cols',type=int,default=9); c.add_argument('--rows',type=int,default=6)
    c.add_argument('--square-mm',type=float,required=True); c.add_argument('--output',default='camera_intrinsics.json')
    a=p.parse_args()
    result=calibrate(glob.glob(a.images),a.cols,a.rows,a.square_mm)
    Path(a.output).write_text(json.dumps(result,indent=2)+'\n')
    print('saved',a.output,'RMS px:',result['rms_px'],'frames:',result['frames_used'])
