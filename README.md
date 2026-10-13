# B.U.D.-E — experimental desktop robotic arm assistant

A compact, Python-first arm concept with original personality, inspired by workshop robots. **Prototype / design study, not an assemble-and-operate kit.** Start with [verification status](docs/verification_status.md) before buying, printing or powering anything. The older Phase 1/2 documents contain unverified claims about price, payload, mechanics and safety; the existing STLs are **not verified to fit or function**. Do not operate near people or edges. Neither camera tracking nor current monitoring is a human safety system.

## Phases and files

| Phase | Files | Status |
|---|---|---|
| 1 — concept | `config/robot_config.yaml`, `docs/phase1_requirements_and_design.md`, `docs/bom_phase1.csv`, `src/phase1_sizing/` | Budget/geometry **estimates only**. The `$58.50` stationary / `$66.50` mobile totals are dated estimates, not quoted prices. |
| 2 — mechanical | `cad/`, `docs/phase2_mechanical_and_3d_printing.md` | 13 generated STLs. Meshes **add** primitives and never subtract, so several "pockets" are pockets only because of the walls around them. **Not ready for full assembly**; see Appendix A for two corrected interface errors. |
| 3 — electronics | `docs/phase3_electronics_and_wiring.md`, `firmware/phase3_bench_diagnostic/` | One-servo **unloaded diagnostic only**. Firmware has not been flashed/tested on real boards. |
| 4 — simulation | `urdf/bude_arm.urdf`, `src/phase4_simulation/` | Box-proxy URDF, a stdlib kinematic demo, an optional PyBullet path, and an interactive **schematic viewer** (`web_sim_server.py` + `web_assets/`). No grasp or collision certification. |
| 5 — controls | `src/phase5_control/`, `ros2_ws/src/bude_moveit_config/` | FK, fail-closed IK with an opt-in pitch ladder, minimum-jerk planning, analytic keep-outs, and MoveIt 2/`ros2_control` **config drafts that have never been loaded**. No hardware driver. |
| 6 — perception/mobile | `src/phase6_perception/` | Measured-intrinsics calibration tool, a design-model camera pipeline, PCA grasp-roll estimation, and a fail-closed offline drive policy with a test harness. No live detection, no autonomous driving, no built rover. |
| 7/8 — voice, personality, safety trials | future | Not built. |

The mathematical geometry is base height 55 mm, upper 80 mm, forearm 75 mm, wrist and tool 70 mm (nominal max radial reach 225 mm). Actual reach and payload depend on a **redesigned and measured** assembly. No physical component alignment has been verified.

## Run offline checks from the repository root

On Windows PowerShell use `python` where examples say `python3`:

```sh
python3 -m unittest discover -s tests -v
python3 src/phase2_mechanical/generate_stl_parts.py     # regenerate all 13 STLs
python3 src/phase2_mechanical/verify_print_parts.py     # mesh audit + measured bolt circle
python3 src/phase4_simulation/sim_runner.py
python3 src/phase5_control/kinematics.py
python3 src/phase5_control/trajectory_and_collision.py
python3 src/phase5_control/ros2_moveit_bridge.py
python3 src/phase6_perception/camera_and_hand_eye_calib.py
python3 src/phase6_perception/vision_mobile_navigator.py
```

Interactive schematic viewer (standard library only, no third-party JavaScript):

```sh
python3 src/phase4_simulation/web_sim_server.py         # then open http://localhost:8080
```

For *optional* box-proxy rigid-body simulation, create a virtual environment then install `numpy pybullet` and run `python src/phase4_simulation/sim_runner.py --backend pybullet` (add `--gui` for a desktop GUI). No installation is needed for the default offline tests. Camera calibration needs `numpy opencv-python` and **your own chessboard photos**. The viewer's browser model is cross-checked against the Python kinematics by the test suite when `node` is available.

## Next physical milestone

Redesign a **single joint** in proper Boolean CAD from measured servo dimensions, print a mating test coupon, inspect cross-sections in a slicer and test-fit **unpowered**. Use a rated metal clamp rather than the existing printed clamp for any supported arm. Then follow [Phase 3](docs/phase3_electronics_and_wiring.md) for a single *unloaded* servo and a physically verified, DC-rated power cutoff. Do **not** trust a shared INA219 to protect people from collisions. If you build the rover, start on the **floor** with the arm stowed and a supervisor.

Open-source references: [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100), [ROBOTIS OpenManipulator](https://github.com/ROBOTIS-GIT/open_manipulator), [PyBullet](https://github.com/bulletphysics/bullet3), [MoveIt 2](https://github.com/moveit/moveit2) and [OpenCV](https://github.com/opencv/opencv). No third-party code or CAD is copied into this repository by the new phases.
