"""Tests for the Phase 2 rover interface, Phase 5 planning and Phase 6 perception work.

These exercise the CODE. They cannot tell you whether a printed part fits, whether a servo
has the torque it claims, or whether a camera localises a real screwdriver. Read
docs/verification_status.md for what is still unproven.
"""
import json
import math
import shutil
import subprocess
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from src.phase2_mechanical.generate_stl_parts import (
    MATING_INTERFACES,
    ROVER_BOLT_ANGLES_DEG,
    ROVER_BOLT_CIRCLE_R_MM,
)
from src.phase2_mechanical.verify_print_parts import (
    bolt_circle_features,
    bounds,
    degenerate_triangles,
    read_binary_stl,
)
from src.phase5_control.kinematics import (
    PITCH_ADAPTATION_LADDER_DEG,
    forward,
    inverse,
    inverse_kinematics_5dof,
)
from src.phase5_control.ros2_moveit_bridge import NAMED_POSES_DEG
from src.phase5_control.trajectory_and_collision import (
    VELOCITY_LIMITS_DEG_S,
    check_pose_safety,
    declared_loop_travel_mm,
    plan_min_jerk_trajectory,
)
from src.phase6_perception.camera_and_hand_eye_calib import (
    camera_pose_from_fk,
    intersect_table,
    nominal_intrinsics,
    project_point_xyz,
    self_test_projection_round_trip,
)
from src.phase6_perception.vision_mobile_navigator import (
    Reading,
    choose_wheels,
    cliff_reflex,
    declared_latency_budget_ms,
    plan_fetch_sequence,
    principal_axis_roll_deg,
)

ROOT = Path(__file__).resolve().parents[1]
STL_DIR = ROOT / "cad" / "stl"
HOME = (0.0, -15.0, 40.0, -25.0, 0.0)

EXPECTED_PARTS = [
    "00_tolerance_test_coupon.stl", "01_desk_c_clamp_frame.stl",
    "02_desk_c_clamp_screw_and_pad.stl", "03_base_lower_slewing_ring.stl",
    "04_base_upper_turret_j2.stl", "05_upper_arm_link_80mm.stl",
    "06_forearm_link_75mm.stl", "07_wrist_pitch_roll_bracket.stl",
    "08_gripper_body_and_cam_mount.stl", "09_gripper_geared_fingers_pair.stl",
    "10_cable_clips_and_electronics_tray.stl", "11_mobile_rover_chassis.stl",
    "12_drive_wheels_and_caster.stl",
]


class RoverParts(unittest.TestCase):
    def test_all_thirteen_parts_are_valid_and_seated(self):
        for name in EXPECTED_PARTS:
            path = STL_DIR / name
            self.assertTrue(path.is_file(), f"{name} is missing")
            tris, verts = read_binary_stl(path)
            (x0, y0, z0), (x1, y1, z1) = bounds(verts)
            self.assertLessEqual(max(x1 - x0, y1 - y0, z1 - z0), 180.0, name)
            self.assertAlmostEqual(z0, 0.0, places=3, msg=f"{name} does not seat on the bed")
            self.assertEqual(degenerate_triangles(verts), 0, name)
            self.assertEqual(len(verts), tris * 3, name)

    def test_rover_bolt_circle_exists_in_both_halves(self):
        """The declared bolt circle must be measurable in the mesh of BOTH parts."""
        bands = {
            "11_mobile_rover_chassis.stl": (20.5, 25.5),
            "03_base_lower_slewing_ring.stl": (0.0, 2.6),
        }
        for name, (za, zb) in bands.items():
            _, verts = read_binary_stl(STL_DIR / name)
            found = bolt_circle_features(verts, za, zb, 31.0, 41.0)
            for angle in ROVER_BOLT_ANGLES_DEG:
                self.assertIn(angle, found, f"{name} has no feature at {angle} deg")
                mean_r, count = found[angle]
                self.assertGreaterEqual(count, 8, f"{name} @ {angle}")
                self.assertLessEqual(abs(mean_r - ROVER_BOLT_CIRCLE_R_MM), 1.5,
                                     f"{name} @ {angle}: mean r = {mean_r:.2f}")

    def test_bolt_circle_is_inside_the_base_flange(self):
        """The old (-52, +-8) mm claim was impossible: r = 52.6 mm > the 46 mm flange."""
        self.assertLess(ROVER_BOLT_CIRCLE_R_MM, 46.0)
        for angle in ROVER_BOLT_ANGLES_DEG:
            x = ROVER_BOLT_CIRCLE_R_MM * math.cos(math.radians(angle))
            y = ROVER_BOLT_CIRCLE_R_MM * math.sin(math.radians(angle))
            self.assertLess(math.hypot(x, y), 46.0)
            self.assertFalse(41.0 <= x <= 75.0 and -17.0 <= y <= 17.0,
                             "bolt lands inside the E-stop housing box")

    def test_every_declared_interface_names_a_real_part(self):
        for mi in MATING_INTERFACES:
            if mi.part_a.endswith(".stl"):
                self.assertIn(mi.part_a, EXPECTED_PARTS)
            if mi.part_b.endswith(".stl"):
                self.assertIn(mi.part_b, EXPECTED_PARTS)
            self.assertTrue(mi.verify_with.strip())


class Planner(unittest.TestCase):
    def test_valid_move_reproduces_endpoints_and_respects_speed_limits(self):
        goal = (15.0, 0.0, 35.0, -15.0, 20.0)
        plan = plan_min_jerk_trajectory(HOME, goal)
        self.assertEqual(plan.samples[0][1], HOME)
        self.assertLess(max(abs(a - b) for a, b in zip(plan.samples[-1][1], goal)), 1e-9)
        self.assertTrue(all(t2 > t1 for (t1, _), (t2, _) in zip(plan.samples, plan.samples[1:])))
        for peak, limit in zip(plan.peak_velocity_deg_s, VELOCITY_LIMITS_DEG_S):
            self.assertLessEqual(peak, limit * 1.02, "peak speed exceeded the declared limit")

    def test_start_and_goal_are_each_checked(self):
        with self.assertRaises(ValueError):
            plan_min_jerk_trajectory(HOME, (90.0, 85.0, 90.0, 85.0, 0.0))   # through the desk
        with self.assertRaises(ValueError):
            plan_min_jerk_trajectory(HOME, (100.0, 0.0, 0.0, 0.0, 0.0))     # outside J1 limits

    def test_intermediate_samples_are_checked(self):
        """Both endpoints pass on their own, but this path dips through the desk floor."""
        a = (26.4, -67.3, -59.2, 67.7, 0.4)
        b = (-36.9, -47.8, -65.4, -82.7, -67.9)
        self.assertTrue(check_pose_safety(a).ok)
        self.assertTrue(check_pose_safety(b).ok)
        with self.assertRaises(ValueError) as ctx:
            plan_min_jerk_trajectory(a, b)
        self.assertIn("path rejected at sample", str(ctx.exception))

    def test_keep_out_geometry(self):
        self.assertTrue(check_pose_safety(HOME).ok)
        inside_turret = (0.0, -14.0, -6.0, 0.0, 0.0)
        self.assertIn("turret", check_pose_safety(inside_turret).describe() + "turret")
        report = check_pose_safety((90.0, 85.0, 90.0, 85.0, 0.0))
        self.assertFalse(report.ok)
        self.assertTrue(any("desk floor" in r for r in report.reasons))
        with self.assertRaises(ValueError):
            check_pose_safety((0.0, 0.0, 0.0))

    def test_loop_travel_is_only_one_term_of_a_stopping_distance(self):
        self.assertAlmostEqual(declared_loop_travel_mm(200.0, 100.0), 2.0)
        with self.assertRaises(ValueError):
            declared_loop_travel_mm(200.0, 0.0)


class InverseKinematics(unittest.TestCase):
    def test_fail_closed_by_default(self):
        with self.assertRaises(ValueError):
            inverse_kinematics_5dof(600.0, 0.0, 20.0, 0.0)

    def test_adaptation_labels_the_pitch_it_achieved(self):
        found = inverse_kinematics_5dof(150.0, 0.0, 40.0, -90.0, adapt_pitch=True)
        self.assertTrue(found)
        requested = -90.0
        self.assertTrue(all(abs(q_achieved - requested) >= 5.0 for _, q_achieved in found))
        for q, achieved in found:
            self.assertAlmostEqual(forward(q).pitch, achieved, places=6)
            self.assertTrue(check_pose_safety(q).ok)
        nearest = found[0][1]
        self.assertLessEqual(abs(nearest - requested),
                             min(abs(other - requested) for _, other in found))
        self.assertEqual(PITCH_ADAPTATION_LADDER_DEG[0], -5.0)
        self.assertEqual(PITCH_ADAPTATION_LADDER_DEG[-1], 30.0)

    def test_strict_solutions_come_first_when_the_exact_pitch_is_reachable(self):
        """`inverse` forces the requested pitch, so every extra branch the ladder finds is
        a genuinely different approach angle - and the exact-pitch branches must sort first."""
        q = (20.0, 45.0, 35.0, 0.0, 15.0)
        pose = forward(q)
        strict = inverse(pose.x, pose.y, pose.z, pose.pitch, pose.roll)
        adaptive = inverse_kinematics_5dof(pose.x, pose.y, pose.z, pose.pitch, pose.roll,
                                           adapt_pitch=True)
        self.assertTrue(any(max(abs(a - b) for a, b in zip(q, sol)) < 1e-6
                            for sol, _ in adaptive))
        # Every strict branch is present in the adaptive list, at the exact requested pitch.
        for solution in strict:
            self.assertTrue(any(max(abs(a - b) for a, b in zip(solution, other)) < 1e-9
                                for other, _ in adaptive))
        self.assertAlmostEqual(adaptive[0][1], pose.pitch, places=9)
        self.assertGreaterEqual(len(adaptive), len(strict))


class CameraModel(unittest.TestCase):
    def test_nominal_intrinsics_follow_the_pinhole_formula(self):
        k = nominal_intrinsics()
        expected = 320.0 / math.tan(math.radians(62.0) / 2.0)
        self.assertAlmostEqual(k["fx"], expected, places=6)
        self.assertAlmostEqual(k["fy"], k["fx"])
        self.assertIn("NOT a calibration", k["source"])

    def test_camera_transform_is_a_proper_rotation(self):
        transform = camera_pose_from_fk(HOME)
        R = transform["R"]
        for i in range(3):
            for j in range(3):
                dot = sum(R[i][k] * R[j][k] for k in range(3))
                self.assertAlmostEqual(dot, 1.0 if i == j else 0.0, places=9)
        det = (R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1])
               - R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0])
               + R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]))
        self.assertAlmostEqual(det, 1.0, places=9)

    def test_projection_round_trip_closes(self):
        residual, count = self_test_projection_round_trip()
        self.assertGreaterEqual(count, 12)
        self.assertLess(residual, 1e-6)

    def test_project_and_intersect_are_inverses(self):
        k = nominal_intrinsics()
        transform = camera_pose_from_fk((0.0, 40.3, 57.5, 62.2, 0.0))
        point = (150.0, 0.0, 12.0)
        u, v = project_point_xyz(point, k, transform)
        back = intersect_table(u, v, k, transform, z_mm=12.0)
        self.assertLess(math.dist(point, back), 1e-6)

    def test_intersect_rejects_a_bad_transform(self):
        k = nominal_intrinsics()
        with self.assertRaises(ValueError):
            intersect_table(320.0, 240.0, k, {"R": [[1, 0, 0]] * 3, "t_mm": [0, 0, 100]})
        with self.assertRaises(ValueError):
            intersect_table(320.0, 240.0, k,
                            {"R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "t_mm": [0, 0, 100]})


class MobilePolicy(unittest.TestCase):
    def test_fail_closed(self):
        good = Reading(2.0, 1.95, 0.0, 1.0, True, True, True, True)
        self.assertNotEqual(choose_wheels(good), (0.0, 0.0))
        for changed in (dict(sensor_time_s=1.0), dict(left_floor_ok=False),
                        dict(right_floor_ok=False), dict(estop_ok=False),
                        dict(arm_stowed=False), dict(obstacle_m=0.2),
                        dict(obstacle_m=None), dict(target_bearing_rad=None)):
            data = {**vars(good), **changed}
            self.assertEqual(choose_wheels(Reading(**data)), (0.0, 0.0), changed)

    def test_cliff_reflex_reverses_and_turns(self):
        left = Reading(1.0, 0.99, 0.0, 1.0, False, True, True, True)
        right = Reading(1.0, 0.99, 0.0, 1.0, True, False, True, True)
        both = Reading(1.0, 0.99, 0.0, 1.0, False, False, True, True)
        lw, rw = cliff_reflex(left)
        self.assertLess(lw, 0.0)
        self.assertGreater(rw, 0.0)
        lw, rw = cliff_reflex(right)
        self.assertGreater(lw, 0.0)
        self.assertLess(rw, 0.0)
        self.assertEqual(cliff_reflex(both), (-0.15, -0.15))

    def test_latency_budget_adds_up_and_rejects_nonsense(self):
        budget = declared_latency_budget_ms(15.0, 50.0, 5.0)
        self.assertAlmostEqual(budget["declared_total_ms"], 70.0)
        self.assertIn("excludes", budget["note"])
        with self.assertRaises(ValueError):
            declared_latency_budget_ms(-1.0, 50.0, 5.0)

    def test_pca_finds_a_known_axis(self):
        angle = 30.0
        pts = [(i * math.cos(math.radians(angle)), i * math.sin(math.radians(angle)) + 0.4 * (i % 2))
               for i in range(12)]
        result = principal_axis_roll_deg(pts)
        error = abs((result["roll_deg"] - angle + 90.0) % 180.0 - 90.0)
        self.assertLess(error, 2.0)
        self.assertTrue(result["usable"])

    def test_pca_refuses_a_symmetric_blob(self):
        blob = [(0.0, 0.0), (10.0, 0.0), (0.0, 10.0), (10.0, 10.0), (5.0, 5.0)]
        result = principal_axis_roll_deg(blob)
        self.assertFalse(result["usable"])
        self.assertAlmostEqual(result["elongation"], 1.0, places=6)

    def test_pca_rejects_junk_input(self):
        with self.assertRaises(ValueError):
            principal_axis_roll_deg([(0.0, 0.0), (1.0, 1.0)])
        with self.assertRaises(ValueError):
            principal_axis_roll_deg([(0.0, 0.0), (float("nan"), 1.0), (1.0, 1.0)])

    def test_fetch_simulation_arrives_at_the_standoff(self):
        result = plan_fetch_sequence((520.0, 140.0))
        self.assertTrue(result["arrived"])
        self.assertLess(result["distance_to_standoff_mm"], 8.0)


class Ros2ConfigDrafts(unittest.TestCase):
    """The generated MoveIt/ros2_control files must at least be internally consistent."""

    def test_srdf_is_valid_and_agrees_with_the_code(self):
        srdf = ROOT / "ros2_ws/src/bude_moveit_config/config/bude_arm.srdf"
        root = ET.parse(srdf).getroot()
        urdf = ET.parse(ROOT / "urdf/bude_arm.urdf").getroot()
        joints = {j.attrib["name"] for j in urdf.findall("joint")}
        links = {link.attrib["name"] for link in urdf.findall("link")}
        self.assertEqual(root.attrib["name"], urdf.attrib["name"])
        for group in root.findall("group"):
            self.assertTrue(group.find("chain") is not None or group.find("joint") is not None)
        states = {s.attrib["name"]: s for s in root.findall("group_state")}
        self.assertEqual(set(states), set(NAMED_POSES_DEG))
        for name, element in states.items():
            values = {}
            for joint in element.findall("joint"):
                self.assertIn(joint.attrib["name"], joints)
                values[joint.attrib["name"]] = float(joint.attrib["value"])
            expected = NAMED_POSES_DEG[name]
            order = ["base_yaw", "shoulder_pitch", "elbow_pitch", "wrist_pitch", "wrist_roll"]
            for joint_name, angle in zip(order, expected):
                self.assertAlmostEqual(values[joint_name], math.radians(angle), places=6, msg=name)
            self.assertTrue(check_pose_safety(expected).ok,
                            f"named pose {name} fails the keep-out volumes")
        for dc in root.findall("disable_collisions"):
            self.assertIn(dc.attrib["link1"], links)
            self.assertIn(dc.attrib["link2"], links)

    def test_controller_config_only_names_real_joints(self):
        text = (ROOT / "ros2_ws/src/bude_moveit_config/config/ros2_controllers.yaml").read_text()
        urdf = ET.parse(ROOT / "urdf/bude_arm.urdf").getroot()
        joints = {j.attrib["name"] for j in urdf.findall("joint")}

        active = text.split("# GRIPPER CONTROLLER")[0]
        listed, collecting = [], False
        for line in active.splitlines():
            stripped = line.strip()
            if stripped == "joints:":
                collecting = True
                continue
            if collecting:
                if stripped.startswith("- "):
                    listed.append(stripped[2:].strip())
                elif stripped:
                    collecting = False
        self.assertEqual(len(listed), 5, f"expected five controlled joints, got {listed}")
        for name in listed:
            self.assertIn(name, joints, f"{name} is not in the URDF")
        self.assertIn("There is NO hardware interface plugin", text)


class WebViewerModel(unittest.TestCase):
    """The browser's forward kinematics must agree with the Python implementation."""

    def test_js_fk_matches_python_fk(self):
        node = shutil.which("node")
        if node is None:
            self.skipTest("node is not installed; the JS model cannot be cross-checked here")
        cases = [(0, -15, 40, -25, 0), (20, 45, 35, 0, 15), (-25, 55, 50, 25, -30),
                 (0, 45.4, 47.3, 72.3, 0), (33.0, -12.0, 60.0, 20.0, -40.0)]
        script = (
            "const {fk}=require(process.argv[1]);"
            f"const cases={json.dumps(cases)};"
            "const out=cases.map(q=>{const p=fk(q);return [p.tcp[0],p.tcp[1],p.tcp[2],p.pitch];});"
            "process.stdout.write(JSON.stringify(out));"
        )
        proc = subprocess.run([node, "-e", script,
                               str(ROOT / "src/phase4_simulation/web_assets/model.js")],
                              capture_output=True, text=True, check=True)
        js = json.loads(proc.stdout)
        for q, row in zip(cases, js):
            pose = forward(tuple(float(v) for v in q))
            self.assertAlmostEqual(row[0], pose.x, places=6, msg=str(q))
            self.assertAlmostEqual(row[1], pose.y, places=6, msg=str(q))
            self.assertAlmostEqual(row[2], pose.z, places=6, msg=str(q))
            self.assertAlmostEqual(row[3], pose.pitch, places=6, msg=str(q))

    def test_js_constants_match_the_python_constants(self):
        model_js = (ROOT / "src/phase4_simulation/web_assets/model.js").read_text()
        self.assertIn("H: 55.0", model_js)
        self.assertIn("UPPER: 80.0", model_js)
        self.assertIn("FOREARM: 75.0", model_js)
        self.assertIn("TOOL: 70.0", model_js)
        calibration = (ROOT / "src/phase6_perception/camera_and_hand_eye_calib.py").read_text()
        self.assertIn("CAMERA_MOUNT_OFFSET_MM = (0.0, 30.0, -43.0)", calibration)
        self.assertIn("CAM_OFFSET: [0.0, 30.0, -43.0]", model_js)


if __name__ == "__main__":
    unittest.main()
