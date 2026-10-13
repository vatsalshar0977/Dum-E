# Phase 6 — perception and a **future** mobile-base option

Live YOLO/hand tracking, grasping, hand-eye calibration and autonomous driving are **not
implemented**. What exists is a measured-calibration tool, a design-model camera pipeline, a
fail-closed offline navigation policy, and a simulation harness that tests that policy. This
document is explicit about which is which.

## 1. Camera intrinsic calibration, step by step

1. Choose a real UVC USB camera that works on your actual macOS/Windows machine and measure
   its PCB, lens and cable before designing a mount. A generic OV2640 camera board is **not
   necessarily UVC USB**; verify the seller's USB interface with your OS. Mount without
   obstructing the gripper.
2. Print a flat chessboard (e.g. **9×6 inner corners**); measure the actual square size with a
   ruler (e.g. 20.0 mm, not assumed). Fix the focus/resolution and disable automatic zoom.
   Collect 15–25 sharp photos across the FOV at varied distance/tilt; save them outside Git as
   `calibration_photos/*.png`.
3. Install `python -m pip install opencv-python numpy`, then on Windows PowerShell:
   ```powershell
   python src\phase6_perception\camera_and_hand_eye_calib.py calibrate --images 'calibration_photos\*.png' --square-mm 20 --cols 9 --rows 6 --output camera_intrinsics.json
   ```
   The tool rejects fewer than 8 usable photos and writes measured fx, fy, cx, cy, distortion,
   RMS reprojection error and image size. Collect held-out shots and measure real localization
   error with a ruler. Do not claim a zero-error calibration on synthetic points.
4. **Hand-eye extrinsics still required.** The camera-to-base rotation and translation must be
   estimated from images of a fiducial taken at several poses with the joint states recorded.
   The transform changes whenever the wrist moves.

## 2. The design-model camera (and why it is not calibration)

`nominal_intrinsics()` computes `fx = fy = (width/2)/tan(hfov/2)`. For 640×480 at a claimed
62° horizontal FOV that is **fx = fy = 532.57 px**, `cx, cy = 320, 240`. This is a *design
placeholder*: a real camera's FOV varies between units of the same part number and focus moves
with the lens. It exists so the viewer and the dry-run pipeline have something self-consistent
to run on. The returned dict says so in its `source` field.

`camera_pose_from_fk(q)` builds a camera-to-base transform in OpenCV convention (x right,
y down, z along the optical axis) from the schematic FK plus a declared mount offset. The
offset, `CAMERA_MOUNT_OFFSET_MM = (0, 30, −43)` mm in the TCP frame, was derived by reading the
bezel position out of `08_gripper_body_and_cam_mount.stl`: roughly 43 mm behind the grasp point
and 30 mm above it. **It has never been measured on a printed part.** Put your bracket under
calipers and update it.

`project_point_xyz` and `intersect_table` are inverses for a pinhole. `self_test_projection_round_trip()`
projects points around the optical axis into pixels and back, checking that the whole chain
(FK → extrinsics → pixel → ray → plane) closes. It currently closes to **2.9 × 10⁻¹⁴ mm over
41 points**, which certifies the **algebra and nothing else**. The test suite enforces a
residual under 1e-6 mm; if someone breaks the rotation convention this test fails immediately.

Still not inferable from a single view: object height. `intersect_table` needs the plane height
you supply, so a target of unknown height localises to the wrong place. Use a second view, a
known object, or a depth camera if that matters.

## 3. Detection plan, not a working pipeline

Start with colour fiducials/AprilTags (OpenCV), then a manually labelled detector (e.g. an
Ultralytics model if its licence fits your deployment) and a **separate** hand detector. Keep
people out of the moving arm's workspace; camera-only hand tracking is not collision
protection. A candidate must clear a confidence threshold and stay stable across several
frames; an unknown object means no motion. No LLM-generated direct motion commands.

`principal_axis_roll_deg(points)` estimates the long axis of a segmented object by PCA so the
wrist roll can align the claw with a screwdriver lying at an angle. It returns an **elongation**
ratio and refuses to be usable below 1.5×, because a symmetric blob has no long axis and
"aligning" to one snaps the wrist to an arbitrary angle. It also inherits the detector's errors:
a bad segmentation gives a confident, wrong angle. Try it:

```sh
python3 src/phase6_perception/vision_mobile_navigator.py
```

## 4. Mobile base: scope and safety

A mobile base is an optional redesign, and the Phase 2 appendix records what was corrected:
the rover does not bolt to the old "(-52, ±8) mm" holes (they fall outside the 92 mm base
flange and inside the E-stop housing), and 64 mm wheels do not fit a 25 mm chassis with a
standard micro servo. The build uses 48 mm wheels and a declared 4× M3 circle at r = 36 mm.

The physics you still have to do yourself: measure mass and centre of gravity with the arm
extended, find the tipping and braking margins, and test redundant edge cutoff with one sensor
deliberately disconnected. Two uncalibrated reflectance sensors alone cannot guarantee
tabletop cliff protection — lighting, dark or glossy surfaces, sensor failure, latency and
braking distance all defeat them, and they fail silently. **Never autonomously drive a
top-heavy arm on a desk edge.** Begin on the floor with the arm stowed, in a contained area,
with a supervisor, a physical bumper and a hard e-stop on the drive rail.

### What is actually implemented

`vision_mobile_navigator.py` provides:

* `choose_wheels(reading)` — the only function allowed to produce a drive command. It fails
  **closed**: stale data, either cliff reading, an obstacle inside 300 mm, a released e-stop,
  a non-stowed arm, a missing obstacle distance or a missing target bearing all return `(0, 0)`.
* `cliff_reflex(reading)` — a cliff reading produces a reverse-and-turn **command pair**, not
  just a stop, because stopping on the edge is already too late. Whether the drive can reverse
  before the wheel reaches the edge depends on sensing distance and speed that only you can
  measure.
* `declared_latency_budget_ms(...)` — the sum of declared sensor, control-period and
  command-write latencies. This is arithmetic on assumptions. Excludes servo slew, drivetrain
  inertia and braking.
* `plan_fetch_sequence(object_mm, ...)` — a **unicycle simulation** that runs the same policy
  the hardware would use, including its fail-closed branches, and reports whether the robot
  reached the declared 185 mm grasp standoff. It has no wheel slip, no inertia, no braking
  distance and no sensor noise, so success means the state machine works — not that a real
  rover will arrive.

## 5. Viewer

`python3 src/phase4_simulation/web_sim_server.py` then open `http://localhost:8080`. It draws
the arm from the same constants as the Python model (the browser's FK is cross-checked against
`src/phase5_control/kinematics.py` by the test suite using node), solves IK through `/api/ik`,
plans through `/api/plan`, animates the offline drive trace through `/api/fetch`, and lets you
inspect all 13 STL files client-side. It is a viewer: no physics, no collision, no measurements.

See [verification status](verification_status.md) for the claims that have *not* been demonstrated.
