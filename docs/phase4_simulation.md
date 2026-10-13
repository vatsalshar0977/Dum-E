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

## Interactive viewer (added with the Phase 5/6 work)

```sh
python3 src/phase4_simulation/web_sim_server.py --port 8080
```

Standard library only, no third-party JavaScript and no CDN, so it works offline. It serves:

| Route | What it returns |
|---|---|
| `/` | the viewer page |
| `/api/model` | link lengths, joint limits, named poses, camera model, demo targets |
| `/api/ik?x=&y=&z=&pitch=&roll=&adapt=` | every IK branch with its achieved pitch and keep-out verdict |
| `/api/plan?x=&y=&z=&pitch=&from=` | the minimum-jerk samples the page plays back |
| `/api/fetch?x=&y=` | the offline drive-policy trace |
| `/api/parts` | the 13 STLs with triangle counts |
| `/api/stl/<file>` | the raw binary STL, parsed and shaded in the browser |

You can orbit the scene, drag the five joint sliders, jump to the four named poses, ask for a
Cartesian target and watch the refusal when it is out of reach, switch on the rover base, run
the offline "mobile vision nav & fetch" trace, and inspect any STL file beside the bench.

The browser re-implements the forward kinematics so it can animate smoothly without a round
trip per frame. `tests/test_phase5_phase6.py` runs that implementation under node and compares
it with `src/phase5_control/kinematics.py` to 1e-6 mm, so the picture cannot silently drift
away from the model. If node is not installed the test skips.

Reminder: this is a **schematic viewer**. No physics, no collision, no measured geometry. A
green result means the model has no objection.
