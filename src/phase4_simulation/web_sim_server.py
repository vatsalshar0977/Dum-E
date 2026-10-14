#!/usr/bin/env python3
"""Interactive 3D viewer for the B.U.D.-E schematic model. Standard library only.

    python3 src/phase4_simulation/web_sim_server.py            # http://localhost:8080
    python3 src/phase4_simulation/web_sim_server.py --port 9000

WHAT IT SHOWS
  * The 5-DOF arm drawn from the same kinematic constants as
    `src/phase5_control/kinematics.py`, with sliders, named poses and an IK box.
  * The minimum-jerk plan and keep-out verdict for any target you ask for.
  * An optional 2WD rover that moves along the offline drive policy from
    `src/phase6_perception/vision_mobile_navigator.py`.
  * Every STL in `cad/stl/`, parsed and shaded in the browser. The geometry you inspect
    here is the same file you would send to a slicer.

WHAT IT DOES NOT SHOW
  * Physics. Nothing is dynamic, nothing collides, nothing has mass. The planner checks
    analytic keep-out volumes, and the drive trace is a unicycle integration.
  * Calibrated anything. Link lengths, mount offsets and limits are DECLARED, not measured.
  * Safety. A green checkmark in this viewer means "the schematic model has no objection".
    It is not a licence to run a real arm near a person.

No third-party JavaScript is loaded: the renderer is a small hand-written painter's
algorithm so the page works offline and nothing can be silently replaced under you.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.phase4_simulation.sim_runner import HOME  # noqa: E402
from src.phase5_control.kinematics import (  # noqa: E402
    FOREARM,
    H,
    LIMITS,
    TOOL,
    UPPER,
    forward,
    inverse_kinematics_5dof,
)
from src.phase5_control.trajectory_and_collision import (  # noqa: E402
    VELOCITY_LIMITS_DEG_S,
    check_pose_safety,
    plan_min_jerk_trajectory,
)
from src.phase6_perception.camera_and_hand_eye_calib import (  # noqa: E402
    CAMERA_MOUNT_OFFSET_MM,
    nominal_intrinsics,
)
from src.phase6_perception.vision_mobile_navigator import (  # noqa: E402
    plan_fetch_sequence,
)
from src.phase7_voice_personality.personality_engine import (  # noqa: E402
    HOME_DEG, SAMPLE_HZ, SEQUENCES, PersonalityDirector,
)
from src.phase7_voice_personality.voice_assistant import (  # noqa: E402
    HOME_PERCHED_DEG, KNOWN_PHRASES, TOOL_SYNONYMS, Assistant, SimRobotLink, parse,
)

ASSETS = Path(__file__).resolve().parent / "web_assets"
STL_DIR = ROOT / "cad" / "stl"

NAMED_POSES = {
    "home": list(HOME),
    "curious_tilt": [15.0, 0.0, 35.0, -15.0, 20.0],
    "handover": [0.0, 11.8, 38.6, 19.5, 0.0],
    "sad_droop": [0.0, 25.8, 84.8, 39.4, 0.0],
}

# Declared targets for the demo scene, in base-fixed and world millimetres.
DEMO_TARGETS = [
    {"name": "screwdriver", "rgb": [220, 70, 70], "pos": [185.0, 0.0, 6.0], "len": 90.0},
    {"name": "cup", "rgb": [70, 150, 220], "pos": [150.0, 95.0, 0.0], "len": 60.0},
    {"name": "screws", "rgb": [200, 190, 90], "pos": [210.0, -70.0, 2.0], "len": 20.0},
]


def _q(value: str | None, default: float) -> float:
    try:
        out = float(value) if value is not None else default
    except (TypeError, ValueError):
        raise ValueError(f"could not read a number from {value!r}")
    if not math.isfinite(out):
        raise ValueError("non-finite number")
    return out


def _parse_q_list(raw: str | None, default):
    if not raw:
        return tuple(default)
    parts = [p for p in raw.replace(" ", "").split(",") if p]
    if len(parts) != 5:
        raise ValueError("expected five comma-separated joint angles")
    return tuple(float(p) for p in parts)


def _pose_payload(q) -> dict:
    pose = forward(q)
    report = check_pose_safety(q)
    return {
        "q": list(q),
        "tcp": [pose.x, pose.y, pose.z],
        "pitch": pose.pitch,
        "roll": pose.roll,
        "elbow_z": pose.elbow_z,
        "wrist_z": pose.wrist_z,
        "safe": report.ok,
        "reasons": list(report.reasons),
    }


class Handler(BaseHTTPRequestHandler):
    server_version = "BudeSim/0.1"

    # -- helpers ---------------------------------------------------------------------
    def _send(self, code: int, body: bytes, content_type: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        # No framing or caching restrictions: the page is meant to be embedded in a preview
        # pane, and stale geometry would be worse than a refetch.
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, payload: dict, code: int = 200) -> None:
        self._send(code, json.dumps(payload).encode("utf-8"), "application/json")

    def _error(self, message: str) -> None:
        self._json({"ok": False, "error": message}, 400)

    def log_message(self, fmt, *args):        # keep the console readable
        if self.server.verbose:               # type: ignore[attr-defined]
            super().log_message(fmt, *args)

    # -- routes ----------------------------------------------------------------------
    def do_GET(self) -> None:                 # noqa: N802
        parsed = urlparse(self.path)
        route = parsed.path.rstrip("/") or "/"
        query = parse_qs(parsed.query)
        try:
            if route == "/":
                self._serve_asset("index.html")
            elif route.startswith("/static/"):
                self._serve_asset(route[len("/static/"):])
            elif route == "/api/model":
                self._api_model()
            elif route == "/api/ik":
                self._api_ik(query)
            elif route == "/api/plan":
                self._api_plan(query)
            elif route == "/api/fetch":
                self._api_fetch(query)
            elif route == "/api/personality":
                self._api_personality()
            elif route == "/api/personality/sequence":
                self._api_sequence(query)
            elif route == "/api/voice":
                self._api_voice(query)
            elif route == "/api/parts":
                self._api_parts()
            elif route.startswith("/api/stl/"):
                self._api_stl(route[len("/api/stl/"):])
            else:
                self._send(404, b"not found", "text/plain")
        except ValueError as exc:
            self._error(str(exc))
        except Exception as exc:              # noqa: BLE001 - never kill the preview thread
            self._json({"ok": False, "error": f"server error: {exc}"}, 500)

    # -- static ----------------------------------------------------------------------
    def _serve_asset(self, name: str) -> None:
        target = (ASSETS / name).resolve()
        if ASSETS.resolve() not in target.parents and target != ASSETS.resolve():
            self._send(403, b"forbidden", "text/plain")
            return
        if not target.is_file():
            self._send(404, b"missing asset", "text/plain")
            return
        ctype = {".html": "text/html; charset=utf-8",
                 ".js": "application/javascript; charset=utf-8",
                 ".css": "text/css; charset=utf-8"}.get(target.suffix, "application/octet-stream")
        self._send(200, target.read_bytes(), ctype)

    # -- api -------------------------------------------------------------------------
    def _api_model(self) -> None:
        self._json({
            "ok": True,
            "link_lengths_mm": {"H": H, "UPPER": UPPER, "FOREARM": FOREARM, "TOOL": TOOL},
            "limits_deg": [list(pair) for pair in LIMITS],
            "velocity_limits_deg_s": list(VELOCITY_LIMITS_DEG_S),
            "home": list(HOME),
            "named_poses": NAMED_POSES,
            "targets": DEMO_TARGETS,
            "camera": {"intrinsics": nominal_intrinsics(),
                       "mount_offset_mm": list(CAMERA_MOUNT_OFFSET_MM)},
            "note": ("DECLARED schematic geometry. Nothing here has been measured on a "
                     "printed arm, and this viewer does no physics and no collision."),
        })

    def _api_ik(self, query) -> None:
        x = _q(query.get("x", [None])[0], 185.0)
        y = _q(query.get("y", [None])[0], 0.0)
        z = _q(query.get("z", [None])[0], 12.0)
        pitch = _q(query.get("pitch", [None])[0], -60.0)
        roll = _q(query.get("roll", [None])[0], 0.0)
        adapt = query.get("adapt", ["1"])[0] not in ("0", "false", "no")
        try:
            found = inverse_kinematics_5dof(x, y, z, pitch, roll, adapt_pitch=adapt)
        except ValueError as exc:
            self._json({"ok": False, "target": [x, y, z], "pitch_requested": pitch,
                        "error": str(exc),
                        "hint": ("Fail-closed by design. Tick 'allow pitch adaptation' to "
                                 "search the nearest reachable approach angle.")})
            return
        payload = [_pose_payload(q) for q, _ in found]
        achieved = [p["pitch"] for p in payload]
        self._json({
            "ok": True,
            "target": [x, y, z],
            "pitch_requested": pitch,
            "roll": roll,
            "solutions": payload,
            "achieved_pitches": achieved,
            "pitch_adapted": any(abs(a - pitch) > 1e-6 for a in achieved),
            "note": "solved from DECLARED link lengths; TCP is model output, not a measurement",
        })

    def _api_plan(self, query) -> None:
        x = _q(query.get("x", [None])[0], 185.0)
        y = _q(query.get("y", [None])[0], 0.0)
        z = _q(query.get("z", [None])[0], 12.0)
        pitch = _q(query.get("pitch", [None])[0], -60.0)
        roll = _q(query.get("roll", [None])[0], 0.0)
        start = _parse_q_list(query.get("from", [None])[0], HOME)
        try:
            found = inverse_kinematics_5dof(x, y, z, pitch, roll, adapt_pitch=True)
        except ValueError as exc:
            self._json({"ok": False, "error": str(exc)})
            return
        chosen, achieved = None, None
        for q, pitch_achieved in found:
            if check_pose_safety(q).ok:
                chosen, achieved = q, pitch_achieved
                break
        if chosen is None:
            self._json({"ok": False,
                        "error": "every IK branch for this target fails a keep-out volume",
                        "branches": len(found)})
            return
        try:
            plan = plan_min_jerk_trajectory(start, chosen)
        except ValueError as exc:
            self._json({"ok": False, "error": str(exc),
                        "hint": "the planner refuses the whole path, never a partial one"})
            return
        self._json({
            "ok": True,
            "target": [x, y, z],
            "start": list(start),
            "goal": list(chosen),
            "achieved_pitch": achieved,
            "duration_s": plan.duration_s,
            "min_tcp_z_mm": plan.min_tcp_z_mm,
            "peak_velocity_deg_s": list(plan.peak_velocity_deg_s),
            "limiting_joint": plan.scaled_by_index + 1 if plan.scaled_by_index >= 0 else None,
            "samples": [[t, list(q)] for t, q in plan.samples],
            "note": "minimum-jerk joint interpolation with analytic keep-outs; no physics",
        })

    def _api_fetch(self, query) -> None:
        x = _q(query.get("x", [None])[0], 520.0)
        y = _q(query.get("y", [None])[0], 140.0)
        result = plan_fetch_sequence((x, y))
        self._json({
            "ok": True,
            "target": [x, y],
            "arrived": result["arrived"],
            "ticks": result["ticks"],
            "final_pose": list(result["final_pose"]),
            "distance_to_standoff_mm": result["distance_to_standoff_mm"],
            "trace": [list(row) for row in result["trace"]],
            "note": ("OFFLINE unicycle simulation of the fail-closed drive policy. No "
                     "traction, braking, tipping or edge-distance model."),
        })

    def _api_parts(self) -> None:
        parts = []
        for path in sorted(STL_DIR.glob("*.stl")):
            raw = path.read_bytes()
            tris = int.from_bytes(raw[80:84], "little") if len(raw) >= 84 else 0
            parts.append({"name": path.name, "bytes": len(raw), "triangles": tris})
        self._json({"ok": True, "parts": parts,
                    "note": ("Cavities are true voids: this generator emits walls and bores "
                             "around empty space instead of subtracting overlap, and "
                             "verify_assembly_fit.py re-measures every one of them from these "
                             "bytes. Still confirm the first article in a slicer.")})

    def _api_personality(self) -> None:
        """The Phase 7 gesture library, so the panel's buttons are built from the source."""
        sequences = []
        for name, seq in SEQUENCES.items():
            sequences.append({
                "name": name, "duration_s": seq.duration, "priority": seq.priority,
                "speed_scale": seq.speed_scale, "loop": seq.loop,
                "triggers": list(seq.triggers), "description": seq.description,
                "frames": int(round(seq.duration * SAMPLE_HZ)) + 1,
                "returns_to": list(seq.returns_to) if seq.returns_to else None,
            })
        self._json({
            "ok": True, "sample_hz": SAMPLE_HZ, "sequences": sequences,
            "home_deg": list(HOME_DEG), "home_perched_deg": list(HOME_PERCHED_DEG),
            "known_phrases": list(KNOWN_PHRASES),
            "tools": {k: list(v) for k, v in TOOL_SYNONYMS.items()},
            "note": ("Gesture timings are validated against declared joint speed budgets at "
                     "100 Hz minimum jerk. Nothing here has run on a servo, and the wheel "
                     "commands are open-loop pulse widths with no measured dead band."),
        })

    def _api_sequence(self, query) -> None:
        """Every 100 Hz frame of one gesture, ready for the canvas to play."""
        name = query.get("name", [""])[0]
        if name not in SEQUENCES:
            self._error(f"unknown sequence {name!r}; try one of {sorted(SEQUENCES)}")
            return
        seq = SEQUENCES[name]
        frames = seq.sample()
        peak = [0.0] * len(frames[0].joints)
        for a, b in zip(frames, frames[1:]):
            dt = b.t - a.t
            if dt <= 0.0:
                continue
            for i, (x, y) in enumerate(zip(a.joints, b.joints)):
                peak[i] = max(peak[i], abs(y - x) / dt)
        self._json({
            "ok": True, "name": name, "sample_hz": SAMPLE_HZ, "duration_s": seq.duration,
            "priority": seq.priority, "speed_scale": seq.speed_scale, "loop": seq.loop,
            "description": seq.description, "triggers": list(seq.triggers),
            "peak_speed_deg_s": [round(v, 2) for v in peak],
            "frames": [f.as_dict() for f in frames],
        })

    def _api_voice(self, query) -> None:
        """Parse one typed sentence and run it against the simulated robot."""
        text = query.get("text", [""])[0]
        intent = parse(text)
        robot = SimRobotLink(verbose=False)
        report = Assistant(robot).handle(text)
        payload = report.as_dict()
        payload["ok"] = True
        payload["action_log"] = robot.log
        payload["note"] = ("Text parsing is deterministic and offline. Speech recognition is a "
                           "separate, optional front end: see "
                           "src/phase7_voice_personality/voice_assistant.py --mic")
        self._json(payload)

    def _api_stl(self, name: str) -> None:
        safe = Path(name).name
        if safe != name or not safe.endswith(".stl"):
            self._send(400, b"bad name", "text/plain")
            return
        path = STL_DIR / safe
        if not path.is_file():
            self._send(404, b"no such part", "text/plain")
            return
        self._send(200, path.read_bytes(), "model/stl")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="0.0.0.0",
                        help="bind address (0.0.0.0 so a preview pane can reach it)")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    httpd.verbose = not args.quiet          # type: ignore[attr-defined]
    print(f"B.U.D.-E schematic viewer on http://{args.host}:{args.port}")
    print("Reminder: schematic geometry, no physics, no collision, no measurements.")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        httpd.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
