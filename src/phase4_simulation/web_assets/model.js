/* B.U.D.-E schematic model + renderer.
 *
 * The forward kinematics here MUST agree with src/phase5_control/kinematics.py. It is
 * checked at load time against /api/model, and `tests/test_web_model.py` compares it with
 * the Python implementation using node when node is available. If the numbers below ever
 * drift from the Python, the viewer is lying about the robot.
 *
 * This is a SCHEMATIC. Nothing is measured, nothing is physical, and the renderer is a
 * painter's-algorithm approximation, not a mesh renderer.
 */
'use strict';

const MODEL = {
  H: 55.0,
  UPPER: 80.0,
  FOREARM: 75.0,
  TOOL: 70.0,
  LIMITS: [[-90, 90], [-75, 85], [-90, 90], [-85, 85], [-90, 90]],
  HOME: [0, -15, 40, -25, 0],
  NAMED: {
    home: [0, -15, 40, -25, 0],
    curious_tilt: [15, 0, 35, -15, 20],
    handover: [0, 11.8, 38.6, 19.5, 0],
    sad_droop: [0, 25.8, 84.8, 39.4, 0]
  },
  // Camera mount offset in the TCP frame (along x_T, y_T, z_T). Mirrors
  // CAMERA_MOUNT_OFFSET_MM in src/phase6_perception/camera_and_hand_eye_calib.py.
  CAM_OFFSET: [0.0, 30.0, -43.0],
  CAM_HFOV_DEG: 62.0,
  // Rover geometry, from the Phase 2 generator constants.
  WHEEL_D: 48.0,
  AXLE_Z: 16.0,
  DECK_TOP_Z: 25.0,
  GROUND_GAP: 8.0
};

const DEG = Math.PI / 180.0;

/** Joint angles in degrees -> key points of the arm in base-local millimetres. */
function fk(q) {
  const yaw = q[0] * DEG;
  const a = q[1] * DEG;
  const b = (q[1] + q[2]) * DEG;
  const c = (q[1] + q[2] + q[3]) * DEG;
  const rElbow = MODEL.UPPER * Math.sin(a);
  const zElbow = MODEL.H + MODEL.UPPER * Math.cos(a);
  const rWrist = rElbow + MODEL.FOREARM * Math.sin(b);
  const zWrist = zElbow + MODEL.FOREARM * Math.cos(b);
  const rTcp = rWrist + MODEL.TOOL * Math.sin(c);
  const zTcp = zWrist + MODEL.TOOL * Math.cos(c);
  const at = (r, z) => [r * Math.cos(yaw), r * Math.sin(yaw), z];
  return {
    base: [0, 0, 0],
    shoulder: at(0, MODEL.H),
    elbow: at(rElbow, zElbow),
    wrist: at(rWrist, zWrist),
    tcp: at(rTcp, zTcp),
    pitch: 90.0 - (q[1] + q[2] + q[3]),
    roll: q[4],
    yaw: q[0]
  };
}

/** TCP frame axes in base-local coordinates: x horizontal, y in-plane up, z along the tool. */
function tcpFrame(pts) {
  const yaw = Math.atan2(pts.tcp[1], pts.tcp[0]);
  const p = pts.pitch * DEG;
  const z = [Math.cos(p) * Math.cos(yaw), Math.cos(p) * Math.sin(yaw), Math.sin(p)];
  const x = [-Math.sin(yaw), Math.cos(yaw), 0];
  const y = [
    z[1] * x[2] - z[2] * x[1],
    z[2] * x[0] - z[0] * x[2],
    z[0] * x[1] - z[1] * x[0]
  ];
  return { x, y, z };
}

if (typeof module !== 'undefined' && module.exports) {
  module.exports = { MODEL, fk, tcpFrame };
}
