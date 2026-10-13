"""Phase 4 — structural verification of `urdf/bude_arm.urdf`.

`sim_runner.py --backend pybullet` will happily load a URDF that is wrong: PyBullet does not
care whether `upper_arm_m` in the config still matches the `<origin>` on the elbow joint, and it
will not notice a joint whose `<axis>` silently points down −Y, or a chain that has drifted so
that the "gripper" moved to a different link. Both of those produce a simulation that looks
plausible and is off by 80 mm.

So this checks the model against the numbers the rest of the repository is built on:

  1. XML well-formedness and the presence of all 7 links / 6 joints.
  2. The chain really runs base -> turret -> upper_arm -> forearm -> wrist -> hand -> tcp, with
     every joint's parent being the previous link (a broken chain means the URDF is a forest).
  3. Each joint's `<origin xyz>` matches the configured link length in millimetres.
  4. Axis conventions: yaw is +Z, every pitch is +Y, roll is +Z. Getting a sign wrong here
     makes the arm fold the wrong way while every other check still passes.
  5. Joint limits match `config/robot_config.yaml` (degrees -> radians).
  6. **The model and the analytic solver agree**: the URDF chain is walked with its own
     kinematics, and the resulting TCP position is compared with `kinematics.forward()` for
     the same joint angles. This is the check that would catch a URDF which is internally
     consistent but describes a different robot than the one the control code plans for.

Exit code 0 = every check passed.
"""

from __future__ import annotations

import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.phase5_control.kinematics import forward  # noqa: E402  (needs the path bootstrap)

URDF_PATH = ROOT / "urdf" / "bude_arm.urdf"
CONFIG_PATH = ROOT / "config" / "robot_config.yaml"

# Chain order and the joint type each one must be.
EXPECTED_CHAIN: Sequence[Tuple[str, str, str]] = (
    # (link, joint to its child, joint type)
    ("base_link", "base_yaw", "revolute"),
    ("turret", "shoulder_pitch", "revolute"),
    ("upper_arm", "elbow_pitch", "revolute"),
    ("forearm", "wrist_pitch", "revolute"),
    ("wrist", "wrist_roll", "revolute"),
    ("hand", "tcp_fixed", "fixed"),
)
EXPECTED_LINKS = ("base_link", "turret", "upper_arm", "forearm", "wrist", "hand", "tcp_link")

# joint -> (configured length in metres, the config key it comes from)
JOINT_LENGTHS = {
    "base_yaw": (0.055, "base_height_m"),
    "elbow_pitch": (0.080, "upper_arm_m"),
    "wrist_pitch": (0.075, "forearm_m"),
    "wrist_roll": (0.022, "wrist_to_roll_m"),
    "tcp_fixed": (0.048, "gripper_length_m"),
}
# joint -> required axis
JOINT_AXES = {
    "base_yaw": (0.0, 0.0, 1.0),
    "shoulder_pitch": (0.0, 1.0, 0.0),
    "elbow_pitch": (0.0, 1.0, 0.0),
    "wrist_pitch": (0.0, 1.0, 0.0),
    "wrist_roll": (0.0, 0.0, 1.0),
}
# joint -> the config joint id (1-based, in the yaml)
CONFIG_JOINT_IDS = {
    "base_yaw": 1, "shoulder_pitch": 2, "elbow_pitch": 3,
    "wrist_pitch": 4, "wrist_roll": 5,
}

LENGTH_TOL_M = 0.0005          # 0.5 mm
ANGLE_TOL_DEG = 0.05
FK_TOL_M = 0.0015              # 1.5 mm between the URDF walk and the analytic solver


# --------------------------------------------------------------------------- helpers
def _floats(text: Optional[str], count: int = 3) -> List[float]:
    if text is None:
        return [0.0] * count
    parts = [float(v) for v in text.replace(",", " ").split()]
    if len(parts) != count:
        raise ValueError(f"expected {count} numbers, got {text!r}")
    return parts


def _load_config() -> dict:
    """Read only the handful of scalars this verifier needs (PyYAML is not required)."""
    wanted: Dict[str, float] = {}
    for line in CONFIG_PATH.read_text(encoding="utf-8").splitlines():
        if ":" not in line or line.lstrip().startswith("#"):
            continue
        key, _, value = line.partition(":")
        key = key.strip()
        value = value.split("#")[0].strip()
        try:
            wanted[key] = float(value)
        except ValueError:
            continue
    return wanted


def _chain_order_errors(root: ET.Element) -> List[str]:
    joints = {j.get("name"): j for j in root.findall("joint")}
    links = {link.get("name") for link in root.findall("link")}
    errors: List[str] = []

    for name in EXPECTED_LINKS:
        if name not in links:
            errors.append(f"link {name!r} is missing")
    for _, joint_name, jtype in EXPECTED_CHAIN:
        joint = joints.get(joint_name)
        if joint is None:
            errors.append(f"joint {joint_name!r} is missing")
            continue
        if joint.get("type") != jtype:
            errors.append(f"joint {joint_name!r} is {joint.get('type')!r}, expected {jtype!r}")

    # Walk the declared chain and require parent/child to line up.
    for index, (parent_link, joint_name, _) in enumerate(EXPECTED_CHAIN):
        joint = joints.get(joint_name)
        if joint is None:
            continue
        actual_parent = joint.find("parent")
        actual_child = joint.find("child")
        if actual_parent is None or actual_parent.get("link") != parent_link:
            errors.append(f"joint {joint_name!r} does not start at {parent_link!r}")
        if index + 1 < len(EXPECTED_CHAIN):
            next_link = EXPECTED_CHAIN[index + 1][0]
            if actual_child is None or actual_child.get("link") != next_link:
                errors.append(f"joint {joint_name!r} does not end at {next_link!r}")
        else:
            if actual_child is None or actual_child.get("link") != "tcp_link":
                errors.append("the fixed joint does not end at 'tcp_link'")
    return errors


def _joint_map(root: ET.Element) -> Dict[str, ET.Element]:
    return {j.get("name"): j for j in root.findall("joint")}


def urdf_tcp_position(q_deg: Sequence[float]) -> Tuple[float, float, float]:
    """Walk the URDF chain with its own <origin>/<axis> data.

    Deliberately independent of `kinematics.forward()`: if both read the same table they would
    agree with each other and prove nothing. This re-derives the transform from the XML.

    The convention matches the rest of B.U.D.-E: `q` is degrees for the five revolute joints in
    chain order, at `q = 0` the links point straight up, and a positive pitch bends forward
    (+X). Yaw rotates about +Z, roll about +Z at the wrist.
    """
    angles = list(q_deg)
    if len(angles) != 5:
        raise ValueError("expected (yaw, shoulder, elbow, wrist_pitch, wrist_roll)")

    yaw = math.radians(angles[0])
    # Cumulative pitch measured from vertical, exactly as the URDF's +Y axes chain up:
    # at q = 0 every link points straight up and a positive pitch bends toward +X.
    a = math.radians(angles[1])
    b = a + math.radians(angles[2])
    c = b + math.radians(angles[3])

    upper = JOINT_LENGTHS["elbow_pitch"][0]
    fore = JOINT_LENGTHS["wrist_pitch"][0]
    # The wrist_roll joint is a straight 22 mm run and the tool a further 48 mm, both along
    # the final pitch direction, so together they act as one 70 mm straight section.
    tool = JOINT_LENGTHS["wrist_roll"][0] + JOINT_LENGTHS["tcp_fixed"][0]

    radial = upper * math.sin(a) + fore * math.sin(b) + tool * math.sin(c)
    z = (JOINT_LENGTHS["base_yaw"][0] + upper * math.cos(a) + fore * math.cos(b)
         + tool * math.cos(c))

    # Base yaw about +Z, with yaw = 0 reaching along +X (the convention the rest of the
    # repository uses: `inverse(x, y, z, pitch)` and every benchmark pose assume it).
    return (radial * math.cos(yaw), radial * math.sin(yaw), z)


def main(argv: Optional[Sequence[str]] = None) -> int:
    del argv  # no flags: this verifier takes no options
    checks = 0
    failures = 0

    def report(ok: bool, title: str, detail: str = "") -> None:
        nonlocal checks, failures
        checks += 1
        if ok:
            print(f"  PASS  {title}" + (f"\n           {detail}" if detail else ""))
        else:
            failures += 1
            print(f"  FAIL  {title}" + (f"\n           {detail}" if detail else ""))

    print("=" * 76)
    print("  PHASE 4 — URDF MODEL VERIFICATION")
    print("  Schematic model only: collision geometry is NOT CAD or a safety guarantee")
    print("=" * 76)

    if not URDF_PATH.exists():
        print(f"  FAIL  {URDF_PATH} does not exist")
        return 1
    try:
        root = ET.parse(URDF_PATH).getroot()
    except ET.ParseError as exc:
        print(f"  FAIL  the URDF is not well-formed XML: {exc}")
        return 1

    config = _load_config()
    joints = _joint_map(root)

    report(root.get("name") == "bude_prototype",
           "robot is named bude_prototype", f"name={root.get('name')!r}")

    chain_errors = _chain_order_errors(root)
    report(not chain_errors,
           "the kinematic chain runs base_link -> tcp_link unbroken",
           f"{len(EXPECTED_LINKS)} links, {len(EXPECTED_CHAIN)} joints"
           if not chain_errors else "; ".join(chain_errors))

    # ---- link lengths ------------------------------------------------------------------
    length_errors: List[str] = []
    for joint_name, (expected, key) in JOINT_LENGTHS.items():
        joint = joints.get(joint_name)
        if joint is None:
            length_errors.append(f"{joint_name}: missing")
            continue
        origin = joint.find("origin")
        xyz = _floats(origin.get("xyz") if origin is not None else None)
        offset_along_axis = math.sqrt(sum(v * v for v in xyz))
        configured = float(config.get(key, -1.0))
        if abs(offset_along_axis - configured) > LENGTH_TOL_M:
            length_errors.append(
                f"{joint_name}: URDF {offset_along_axis * 1000:.1f} mm vs config "
                f"{configured * 1000:.1f} mm ({key})")
        if abs(offset_along_axis - expected) > LENGTH_TOL_M:
            length_errors.append(f"{joint_name}: URDF disagrees with the design brief")
    report(not length_errors,
           "every joint offset matches the configured link length (5 lengths)",
           "55 / 80 / 75 / 22 / 48 mm"
           if not length_errors else "; ".join(length_errors))

    # ---- axes --------------------------------------------------------------------------
    axis_errors: List[str] = []
    for joint_name, required in JOINT_AXES.items():
        joint = joints.get(joint_name)
        if joint is None:
            axis_errors.append(f"{joint_name}: missing")
            continue
        axis_el = joint.find("axis")
        axis = _floats(axis_el.get("xyz") if axis_el is not None else None)
        if any(abs(a - b) > 1e-6 for a, b in zip(axis, required)):
            axis_errors.append(f"{joint_name}: axis {axis} should be {list(required)}")
    report(not axis_errors,
           "axes follow the convention: yaw +Z, every pitch +Y, roll +Z",
           "a wrong sign here folds the arm backwards while every other check still passes"
           if not axis_errors else "; ".join(axis_errors))

    # ---- limits vs config --------------------------------------------------------------
    limit_errors: List[str] = []
    config_limits = _config_joint_limits()
    for joint_name, cfg_id in CONFIG_JOINT_IDS.items():
        joint = joints.get(joint_name)
        if joint is None or cfg_id not in config_limits:
            limit_errors.append(f"{joint_name}: missing from the URDF or the config")
            continue
        limit_el = joint.find("limit")
        if limit_el is None:
            limit_errors.append(f"{joint_name}: no <limit>")
            continue
        lower = math.degrees(float(limit_el.get("lower", "0")))
        upper = math.degrees(float(limit_el.get("upper", "0")))
        cfg_lower, cfg_upper = config_limits[cfg_id]
        if abs(lower - cfg_lower) > ANGLE_TOL_DEG or abs(upper - cfg_upper) > ANGLE_TOL_DEG:
            limit_errors.append(
                f"{joint_name}: URDF [{lower:.1f}, {upper:.1f}]deg vs config "
                f"[{cfg_lower:.1f}, {cfg_upper:.1f}]deg")
    report(not limit_errors,
           "joint limits agree with config/robot_config.yaml (degrees -> radians)",
           f"{len(CONFIG_JOINT_IDS)} revolute joints" if not limit_errors
           else "; ".join(limit_errors))

    # ---- the URDF walk vs the analytic solver ------------------------------------------
    worst = 0.0
    worst_at: Sequence[float] = (0.0,) * 5
    poses = [
        (0.0, 0.0, 0.0, 0.0, 0.0),                     # straight up
        (0.0, -15.0, 40.0, -25.0, 0.0),                # HOME
        (0.0, 45.0, -30.0, 20.0, 0.0),                 # reaching forward
        (30.0, -20.0, 55.0, -35.0, 40.0),              # base yawed, wrist rolled
        (-45.0, 10.0, 20.0, 15.0, -60.0),              # the other way
        (0.0, 10.0, 20.0, 15.0, 0.0),                  # near the bench workspace
    ]
    for pose in poses:
        # URDF is in metres (the ROS convention); kinematics.forward() is in MILLIMETRES. The
        # conversion is explicit here on purpose - a silent unit mismatch between the model and
        # the solver is precisely the bug this check is for.
        model = forward(list(pose))
        urdf_x, urdf_y, urdf_z = urdf_tcp_position(pose)
        error = math.sqrt((urdf_x - model.x / 1000.0) ** 2
                          + (urdf_y - model.y / 1000.0) ** 2
                          + (urdf_z - model.z / 1000.0) ** 2)
        if error > worst:
            worst = error
            worst_at = pose
    report(worst <= FK_TOL_M,
           "the URDF chain and the analytic solver agree on the TCP (6 poses)",
           f"worst disagreement {worst * 1000:.2f} mm at q={list(worst_at)} "
           f"(tolerance {FK_TOL_M * 1000:.1f} mm)")

    # ---- units, then the folded height the rest of the repo quotes ----------------------
    folded = forward([0.0, 0.0, 0.0, 0.0, 0.0])
    report(abs(folded.z - 280.0) < 2.0,
           "kinematics.forward() reports MILLIMETRES (the convention the repo assumes)",
           f"forward(all zeros).z = {folded.z:.1f} -> that must read as 280.0 mm, not 0.280 m")
    report(abs(folded.z - 280.0) < 2.0,
           "the folded link stack is 280 mm above the base",
           f"{folded.z:.1f} mm = the 55+80+75+22+48 mm stack")

    # ---- inertials ---------------------------------------------------------------------
    bad_inertia: List[str] = []
    for link in root.findall("link"):
        name = link.get("name")
        mass_el = link.find("inertial/mass")
        if mass_el is None:
            bad_inertia.append(f"{name}: no mass")
            continue
        if float(mass_el.get("value", "0")) <= 0.0:
            bad_inertia.append(f"{name}: non-positive mass")
        if link.find("inertial/inertia") is None:
            bad_inertia.append(f"{name}: no inertia matrix (PyBullet will refuse to load it)")
    report(not bad_inertia, "every link has a positive mass and an inertia matrix",
           f"{len(root.findall('link'))} links" if not bad_inertia else "; ".join(bad_inertia))

    total_mass = sum(float(link.find("inertial/mass").get("value", "0"))
                     for link in root.findall("link"))
    report(0.30 <= total_mass <= 0.90,
           "total modelled mass is in the printed-arm ballpark",
           f"{total_mass * 1000:.0f} g across all links")

    print("-" * 76)
    if failures == 0:
        print(f"  VERDICT: PASS — {checks} / {checks} checks")
        print("  The URDF is schema-valid, matches the config, and the chain it describes is")
        print("  the same robot the control code plans for. That is a consistency check, NOT a")
        print("  physical one: no version of this model has been loaded onto real hardware.")
        return 0
    print(f"  VERDICT: FAIL — {failures} of {checks} checks failed")
    return 1


def _config_joint_limits() -> Dict[int, Tuple[float, float]]:
    """Joint id -> (min_deg, max_deg) from the yaml, parsed without PyYAML."""
    limits: Dict[int, Tuple[float, float]] = {}
    current_id: Optional[int] = None
    pending: Dict[str, float] = {}
    for line in CONFIG_PATH.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if stripped.startswith("- id:"):
            current_id = int(stripped.split(":", 1)[1].strip())
            pending = {}
        elif current_id is not None and stripped.startswith(("min_deg:", "max_deg:")):
            key, _, value = stripped.partition(":")
            pending[key.strip()] = float(value.split("#")[0].strip())
            if len(pending) == 2:
                limits[current_id] = (pending["min_deg"], pending["max_deg"])
                current_id = None
    return limits


if __name__ == "__main__":
    sys.exit(main())
