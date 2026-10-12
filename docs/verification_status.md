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
