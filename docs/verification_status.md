# Verification status and corrections to earlier claims

The older chat messages described Phase 3–6 files and v3 STL geometry, but **they were not present in the repository when this session started**. These files are a fresh, conservative rebuild, not recovered copies. The only checked-in CAD/STLs on arrival were the original Phase 2 designs. Neither a successful STL integrity check nor coordinate algebra proves physical assembly.

## Stop-work issues with current Phase 1/2 material

- The Phase 2 mesh generator uses `add_box` to create *solid* plates and `add_servo_cradle` to add walls on top. It does **not subtract** servo pockets from intersecting solids. Several apparent holes are standalone tubes intersecting solid boxes, not through-holes. J2–J4 passive tubes use `add_tube_z` (vertical Z axis) even though the pitch axes need to be horizontal. The two gripper 'gears' are smooth rings, not interlocking teeth. **Do not print the full arm as a functional mechanism yet.** Open the files in a slicer and inspect layer cross sections; redo parts using Boolean CAD with measured servo and horn drawings, then print a mating-coupon set and verify with real components.
- Existing `cad/stl/11_*` and `12_*` rover chassis/wheels are **not present**. The previous $8 mobile conversion/60-second swap was not verified. A mobile bench arm is a *new project* with mass, tipping, power and edge safety requirements; see Phase 6 doc.
- Earlier torque figures used advertised stall torque as a safety factor; a hobby servo's sustainable torque, actual product dimensions, flange spacing and supply current must be measured. The old `config/robot_config.yaml`, README and Phase 1/2 docs contain optimistic estimates. **Treat those as proposals, not validation.**
- A common INA219 on the whole servo rail cannot tell which joint stalled; I²C polling is not a reliable human collision safety function. Existing Phase 1 firmware uses naïve substring JSON parsing and can clear an overcurrent latch by changing the limit. **Do not use that sketch on assembled hardware**. Use the new *bench-only* Phase 3 sketch for first testing.
- The previous simulated torque/collision/0.000 mm calibration precision claims were not measured. Schematic link models in `urdf/` do not represent the existing meshes or prove gripping, collision-free trajectories or payload. Neither voice nor mobile autonomous fetch is implemented.

## What was tested in this session

Run `python3 -m unittest discover -s tests -v`; this checks URDF structure, FK/IK, path rejection, ray-plane geometry and offline mobile policy. `python3 src/phase4_simulation/sim_runner.py` demonstrates **kinematics only** (not physics). `--backend pybullet` is optional real rigid-body simulation of *box proxies* and is distinct from the stdlib demo. Physical hardware, motor firmware compilation, CAD mating and camera calibration have **not** been tested here.

## Gates before handling objects or people

1. Obtain real servo/camera/switch datasheets and measure every critical dimension. Redesign CAD; print fit coupons and manually assemble with power off. Secure the bench using a **rated metal clamp** pending load testing; do not trust a printed clamp to carry a working arm.
2. Measure no-load and stall/inrush currents with appropriate rated instruments; size supply, switch, wiring and fuse from those measurements. Verify e-stop physical interruption and controlled re-enable. Brace arm before cutting power (gravity drop).
3. Bench-run one servo unloaded, then one joint without a payload with slow limits; calibrate PWM centre, actual travel, direction, mechanical hard stops and current.
4. For perception, photograph an actual measured chessboard, check reprojection error on held-out images, measure hand-eye extrinsics and depth/height assumptions with a ruler. Ask for human confirmation before object pickup.
5. Mobile version: start on a **floor** in a contained test area, arm stowed, supervised, with a separate bumper and hard e-stop; do not run on a desk/near people. Require validated independent edge prevention before tabletop use.

## Added with the Phase 5/6 work

- `cad/stl/11_mobile_rover_chassis.stl` and `12_drive_wheels_and_caster.stl` now exist and pass
  the mesh audit (valid binary STL, seated on the bed, fits a 180 mm plate, no degenerate
  triangles). Two claims in the earlier write-up were **wrong and have been corrected**:
  `(-52.0, +-8.0) mm` lies outside the 92 mm base flange (r = 52.6 mm) and inside the E-stop
  housing, so the interface is now 4x M3 on an r = 36 mm bolt circle; and 64 mm wheels cannot
  fit a 25 mm chassis with a standard micro servo, so the build uses 48 mm wheels with an 8 mm
  ground gap. `verify_print_parts.py` measures the bolt circle in the vertex data of both
  parts, which is a mesh fact, not a fit test.
- The generator now prints a **mesh-volume column** next to the declared filament weights.
  Those declared weights were never measured; the mesh volume is an upper bound that ignores
  infill and double-counts overlaps. `verify_print_parts.py` no longer asserts a `z_min` of
  zero from a table - it measures it.
- `src/phase5_control/trajectory_and_collision.py` adds a minimum-jerk planner that checks
  every sample (an intermediate-sample rejection case is in the test suite) and analytic
  keep-out volumes. The e-stop keep-out is a **box**, not the sphere the brief specified: the
  brief's r = 30 mm sphere at (56, 0, 26) leaves a 6.4 mm shell of real housing outside it.
- `ros2_ws/src/bude_moveit_config/` contains SRDF and controller drafts that are **valid XML
  and YAML but have never been loaded by MoveIt 2**, plus a bridge whose ROS 2 mode has never
  been executed. There is no gripper group because the URDF has no finger joints, and no
  hardware interface because none has been written.
- `src/phase6_perception/camera_and_hand_eye_calib.py` gained a design-model camera. The
  projection round trip closes to 2.9e-14 mm over 41 points, which validates the **algebra**.
  `fx = 532.57 px` is what a 62 deg HFOV implies at 640x480 - it is not a measurement, and the
  mount offset (0, 30, -43) mm was read off the CAD, not off a caliper.
- `src/phase6_perception/vision_mobile_navigator.py` added a PCA grasp-roll estimator (which
  refuses to be usable below 1.5x elongation) and a unicycle simulation harness for the
  fail-closed drive policy. Neither has been near hardware.
- The interactive viewer is new. Its browser forward kinematics are compared against the
  Python implementation by the test suite when `node` is installed; if you edit one, edit both.

## Still not demonstrated

No part has been printed or measured. No servo has been powered. No camera photo has been
taken. No serial port has been opened. No torque, current, temperature, backlash, repeatability
or stopping-distance figure in this repository has been measured on hardware. Every number
marked *declared*, *nominal* or *schematic* is a design intent.
