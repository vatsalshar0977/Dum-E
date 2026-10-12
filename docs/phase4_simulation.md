# Phase 4 — simulation first

`urdf/bude_arm.urdf` is a **schematic**, meter/radian URDF with base yaw, three pitch axes (URDF +Y, with 0 = straight up), roll and a fixed TCP. It uses boxes to approximate volumes and estimated link masses; there is no modeled meshed gripper, payload, cable, camera image or e-stop. It deliberately **does not reference the old STLs**: `docs/verification_status.md` explains why they cannot yet be trusted as assembly geometry. Update mass/inertia and replace collision proxies *after measuring the printed redesign*. A collision/contact pass in this model cannot certify safety.

From the repo root, on Windows PowerShell use `python` instead of `python3`:

```sh
python3 -m unittest discover -s tests -v
python3 src/phase4_simulation/sim_runner.py
# Optional rigid-body simulation of proxy geometry:
python3 -m pip install numpy pybullet
python3 src/phase4_simulation/sim_runner.py --backend pybullet
python3 src/phase4_simulation/sim_runner.py --backend pybullet --gui
```

The default runner is a **pure-Python kinematic demonstration**, not a 240 Hz physics simulation. The optional PyBullet path loads the URDF, sets gravity, and steps its position motors at 240 Hz; inspect actual following error, contacts, and required torque before treating it as a useful dynamics test. It does not automatically pick up a screwdriver. For visual mesh inspection, use a slicer and inspect each layer rather than taking a scripted 'PASS' on faith.
