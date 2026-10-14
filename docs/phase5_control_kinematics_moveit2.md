# Phase 5 — control foundations

`src/phase5_control/kinematics.py` computes FK and two geometric IK branches for the
**schematic** arm. Joint order is yaw, shoulder, elbow, wrist pitch, roll (degrees). Zero
pitch joints point straight up; positive pitch folds toward the work area. Tool pitch is
measured from horizontal.

## 1. Inverse kinematics, and why it refuses

`inverse(x, y, z, pitch, roll)` raises when the requested pose is outside reach or outside
joint limits. It does not silently substitute a "nearest" pose, because a solver that does
that will move a real arm somewhere the caller never asked for.

`inverse_kinematics_5dof(x, y, z, pitch, roll, adapt_pitch=False)` is the named entry point
used by the bridge and the viewer:

* **Default (`adapt_pitch=False`) is fail-closed.** Errors propagate unchanged.
* **`adapt_pitch=True`** walks a ladder of pitch offsets (±5° … ±30°, `PITCH_ADAPTATION_LADDER_DEG`)
  and returns every distinct joint solution it finds, **each labelled with the pitch it
  actually achieves**, sorted by how far that pitch is from your request. Nothing is
  substituted silently: read `forward(q).pitch` (or the `achieved_pitches` field from the
  viewer API) before commanding motion.

Solutions are de-duplicated on the **joint vector**, not on the pitch — two elbow branches can
share an approach angle, and keying on pitch would discard one of them.

Still not checked: the orientation of the real gripper fingers, obstructed approaches, cable
snag, and desk contact by any point that is not one of the four schematic body samples.

## 2. Minimum-jerk planning and keep-out volumes

`trajectory_and_collision.py`:

* `plan_min_jerk_trajectory(start, goal, sample_hz=100)` time-scales the move with the quintic
  `s(τ) = 10τ³ − 15τ⁴ + 6τ⁵` (zero velocity **and** zero acceleration at both endpoints) until
  no joint exceeds its declared speed limit, using `T ≥ 1.875·|Δq|/v_max`. It checks **every
  sample** before returning and raises rather than handing back a partial path. This matters:
  `start = (26.4, −67.3, −59.2, 67.7, 0.4)` and `goal = (−36.9, −47.8, −65.4, −82.7, −67.9)`
  each pass the keep-outs on their own, and the straight-line joint interpolation between them
  still drives the tool to z = 9.8 mm on the way. That case is in the test suite.
* `check_pose_safety(q)` evaluates four schematic body samples (TCP, palm, wrist centre, elbow
  centre) against a desk floor, the base turret cylinder and the E-stop housing. It collects
  **all** violations instead of stopping at the first, so a log shows the whole picture.

### Why the keep-out is a box, not the sphere in the brief

The design brief specified a keep-out sphere at `(+56, 0, 26) mm` with `r = 30 mm`. The E-stop
housing in `03_base_lower_slewing_ring.stl` actually spans `x = 41…75`, `y = −17…17`,
`z = 0…32 mm`. The farthest corner of that box is **36.4 mm** from the sphere's centre, so the
sphere leaves a **6.4 mm shell of real printed material outside its own keep-out**. A sphere is
only conservative if its radius covers the far corner. `check_pose_safety` therefore tests the
box by default; `use_estop_sphere=True` adds the sphere on top if you want it.

`declared_loop_travel_mm(speed, loop_hz)` is **one term** of a stopping distance (distance
travelled in one control period), not a stopping distance. Sensor latency, transport latency,
servo slew, drivetrain inertia and braking are all unmeasured; measure them before letting
anything move near a person or an edge.

## 3. ROS 2 / MoveIt 2 — drafts, not a controller

`ros2_ws/src/bude_moveit_config/config/bude_arm.srdf` and `ros2_controllers.yaml` are
hand-written and have **never been loaded by a real MoveIt 2 install**. `ros2_moveit_bridge.py`
has a `--mode ros2` path that is a template which has never been executed.

What the drafts deliberately do *not* pretend to have:

* **No gripper group.** `urdf/bude_arm.urdf` models the claw as one rigid `hand` link with a
  fixed `tcp_fixed` joint. There are no finger joints, so a `GripperActionController` would
  have nothing to command. The controller YAML shows the shape of that config commented out.
* **No hardware interface.** Nothing connects these controllers to an ESP32. Whoever writes
  that interface must reject stale commands, require an explicit re-arm after an e-stop, and
  report **measured** joint positions — which unmodified hobby servos cannot do. Commanded
  position is not measured position.
* **No rover base.** Continuous-rotation servos have no position feedback and no wheel
  encoders exist, so the base has no odometry and must not appear as a controllable mobile
  base in a planner.

To explore this properly you need a ROS 2 distribution (normally Ubuntu, on a separate machine
or VM) with `moveit2` and `ros2_control` installed. Start against mock hardware, load the URDF
in RViz, and only then write a serial bridge. The four named poses in the SRDF are generated
from the schematic solver and checked by `tests/test_phase5_phase6.py`, which fails if a pose
in the file drifts from the code or violates a keep-out.

## 4. Verify what is here

```sh
python3 -m unittest discover -s tests -v
python3 src/phase5_control/kinematics.py
python3 src/phase5_control/trajectory_and_collision.py
python3 src/phase5_control/ros2_moveit_bridge.py
python3 src/phase4_simulation/web_sim_server.py    # interactive, then open localhost:8080
```
