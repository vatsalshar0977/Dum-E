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

## Task 1 upgrade (true hollow / coaxial / straddle)

- `generate_stl_parts.py` no longer carves cavities with overlapping solids: holes and bores
  come from `add_hollow_pocket`, `add_tube_z`, `add_hex_pocket_z` and `add_plate_with_holes_z`,
  so a cavity is empty by construction and the mesh writer emits no material inside it.
- Every driven link now uses the datasheet spline offset (`+10.35 mm` MG996R, `+5.60 mm`
  MG90S). `verify_assembly_fit.py` measures each bore's own vertex ring against its axis:
  **25 / 25 rows at 0.0000 mm**, and the servo case centre is 0.0013 mm from the position that
  puts the spline on the joint axis - that residual is the audit's ray nudge, not the model.
- Telescoping straddle clearances are +0.60 mm total (+0.30 mm per side) at J2, J3 and J4,
  measured from bore rings: `44.000 / 44.600`, `42.000 / 42.600`, `29.000 / 29.600`.
- A real defect was found and fixed: the hex nut traps in part `03` were rectangular because
  the hole carver's radial fan collapsed on a regular hexagon. See phase 2 Appendix B.2.
- `verify_assembly_fit.py` now reports **206 / 206 PASS**; `verify_print_parts.py` PASSes; the
  unit suite is 33 tests, OK.
- The STLs are exported in the **assembly frame** (a joint axis is the origin). Parts `05`-`08`
  therefore start below `z = 0`; the checks assert a *flat base* rather than `z_min == 0`, and
  the slicer's drop-to-bed is a documented step.

## Added with the Phase 7/8 work

- `src/phase7_voice_personality/personality_engine.py` builds six gestures as 100 Hz
  minimum-jerk keyframes over 6 joints + 2 wheels. Every segment is validated at construction
  against a per-channel speed budget (`T >= 1.875 * |dq| / v_max`), so an infeasible gesture
  raises instead of running. The two speed budgets (transport from Phase 5, gesture for
  empty-handed expressions) are separate and documented.
- `voice_assistant.py` parses text deterministically with the standard library only: a machine
  with no microphone, no model and no internet still gets every command. Speech (Whisper
  `tiny.en`/`base.en`) is optional, opt-in per utterance, and never a background stream.
  "Stop" is matched first, on the raw sentence, and is not a substitute for the e-stop.
- The viewer gained a Voice & personality panel that plays the server's own samples through the
  same joints the sliders drive, so the drawing and the command stream cannot disagree.
- `safety_supervisor_and_logger.py` evaluates six faults in a fixed order (hardware e-stop,
  brownout, cliff, obstacle, over-current, missed grasp) plus stale sensors, and logs every tick
  to `logs/bude_telemetry.jsonl` with joints, TCP xyz, bus volts, milliamps, wheel pulse widths,
  cliff states and the supervisor phase. All six injectors pass.
- The e-stop hold latches and, on release, **overwrites the software targets with the measured
  joint angles before clearing the latch**, so a sagged arm cannot snap. Measured residual in
  the injector: 0.000 deg.
- The 5 ms cliff deadline is explicitly assigned to the ESP32 interrupt handler. The Python
  supervisor stops the wheels in the same tick it sees the sensor, which is zero software
  delay — not 5 ms of end-to-end hardware latency, and this repository does not claim it.
- `run_full_acceptance_suite.py` runs all eight phases and reports **43 / 43 checks PASS** in
  about 26 s. The unit suite is 72 tests (33 + 39), all passing.

## Still not demonstrated

No part has been printed or measured. No servo has been powered. No camera photo has been
taken. No serial port has been opened. No torque, current, temperature, backlash, repeatability
or stopping-distance figure in this repository has been measured on hardware. Every number
marked *declared*, *nominal* or *schematic* is a design intent.

Specifically still missing: a gesture vocabulary checked against collision (the Phase 5 keep-outs
cover task poses, not expressions); a wall-clock latency budget from speech to first servo
command; measured thresholds for the over-current, brownout, obstacle and missed-grasp
detectors; wheel dead-band calibration for the two continuous-rotation servos; a firmware
watchdog that cuts PWM when the host stops talking (FMEA #11); an I²C bus-reset path
(FMEA #12); and any fatigue or drop test on the printed links.
