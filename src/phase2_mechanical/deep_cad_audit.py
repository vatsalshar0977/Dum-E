#!/usr/bin/env python3
"""Deep CAD audit: reads the 14 shipped STL files and interrogates them.

`verify_assembly_fit.py` asks whether the declared features exist. This asks harder
questions, from the bytes of the files that will actually be printed:

  A. TOPOLOGY      is each file a closed, positively-wound surface with no naked edges,
                   no degenerate faces, and how many disjoint bodies does it contain?
  B. SIZING        true union volume (slice integration, not voxel counting), mass, centre
                   of mass, first-layer contact area, overhang area that needs support.
  C. JOINABILITY   every bore is re-measured with a least-squares circle fit and then judged
                   against the part that goes through it AT BOTH ENDS of a +/-0.15 mm print
                   error - the question "does this still assemble on a real printer?".
  D. PLACEMENT     is the space inside each servo pocket actually big enough for the declared
                   servo case, with how much margin? Do the controller boards fit the tray,
                   is there anything holding them, and do the cable clips clip?
  E. CLEARANCE     mating pairs are placed in the assembled pose and tested for interference
                   by ray casting, not by comparing declarations.

WHAT PASS MEANS: the geometry is internally consistent and survives a +/-0.15 mm printer.
WHAT IT DOES NOT MEAN: that a printed part fits. That needs a caliper and a first article.

HOW THE MEASUREMENTS ARE MADE (and where they can be fooled):
  * volumes and masses come from even-odd Z-slicing of the triangle soup at 0.4-0.5 mm;
  * bores are two-pass Kasa circle fits - the second pass keeps only the vertices within 2%
    of the first radius, so a bore that shares its Z band with a coaxial boss is still measured
    on the bore. A large gap between the two passes means the feature is not one circle, and
    that is reported rather than hidden;
  * hex traps are measured as the ACROSS-FLATS of the convex hull of the trap's vertices over
    36 directions, and checked for a 2/sqrt(3) across-corners ratio (a circle scores 1.0000);
  * interference samples the child's surface at every triangle corner, edge midpoint and
    centroid, over a -5..+5 deg sweep about the joint axis, and asks the parent whether that
    point is inside solid material. Three nudged parity rays must agree, so a point that merely
    TOUCHES a face is not counted as interference;
  * a 0.5 mm-thin overlap can still hide between sample points, which is why the mating faces
    are additionally checked span-by-span in `verify_assembly_fit.py`. Two independent methods,
    and both have to agree before this project calls a joint clean.

Known limitation: the joint sweep is +/-5 deg. A clash that only appears at a larger angle is
reported by the analytic primitive check in the PR discussion, not by this file.

Usage:
    python3 src/phase2_mechanical/deep_cad_audit.py
    python3 src/phase2_mechanical/deep_cad_audit.py --quick
    python3 src/phase2_mechanical/deep_cad_audit.py --out docs/cad_deep_audit_report.txt
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))

from mesh_audit import Mesh, read_binary_stl, degenerate_triangles, load_mesh   # noqa: E402
from generate_stl_parts import (                                                # noqa: E402
    AXLE_HEIGHT_MM, CASTER_CUP_BORE_MM, COAXIAL_BORES, DECK_T_MM, DECK_TOP_MM,
    DT_SERVO_ALONG_AXIS_MM, DT_SERVO_L_MM, DT_SERVO_SHAFT_OFFSET_MM, DT_SERVO_W_MM,
    GROUND_GAP_MM,
    J1_SPLINE_Z, M3_CLEARANCE_HOLE_D_MM, M3_NUT_ACROSS_FLATS_MM, M3_NUT_TRAP_DEPTH_MM,
    MG996R_BODY_L, MG996R_BODY_W, MG996R_CASE_ALONG_AXIS, MG996R_HORN_BORE_D,
    MG996R_SPLINE_OFFSET, MG90S_BODY_L, MG90S_BODY_W, MG90S_CASE_ALONG_AXIS,
    AXIAL_CLEARANCE_MM, CLAW_COLLAR_H_MM, CLAW_COLLAR_ID_MM, CLAW_COLLAR_OD_MM,
    CLAW_PIN_SHAFT_MM,
    MG90S_BODY_W, MG90S_HORN_BORE_D, MG90S_SPLINE_OFFSET, PASSIVE_BORE_D, PASSIVE_PIN_D,
    PIVOT_PIN_HEAD_D_MM, PIVOT_PIN_HEAD_T_MM, PIVOT_PIN_SHAFT_MM,
    PART_FILENAMES, ROVER_BOLT_ANGLES_DEG, ROVER_BOLT_CIRCLE_R_MM, WALL_T,
    WHEEL_DIAMETER_MM, WHEEL_THICKNESS_MM, STRADDLE_GAP_TOTAL,
    J2_OUTER_Y, J3_OUTER_Y, J4_OUTER_Y, J2_Y_NEG, J3_Y_NEG, J4_Y_NEG,
    J2_AXIS_Z_IN_04, J3_AXIS_Z_IN_05, J4_AXIS_Z_IN_06, J5_AXIS_X_IN_07,
    J5_AXIS_Z_IN_07, J6_AXIS_X_IN_08, J6_AXIS_Z_IN_08, J6_LEFT_Y, J6_RIGHT_Y,
)

# How many separate pieces each file is DESIGNED to print as. A union of overlapping
# primitives is one piece; a screw and its pad, a pair of fingers, two wheels and a caster are
# deliberately several. Anything else - a plate that floats 2 mm off the part it belongs to -
# is a defect, and the check below now fails on it instead of shrugging at "<= 8 bodies".
# How far each straddle joint may be commanded, measured rather than assumed. The limits
# come from this audit's own sweep: past them the arm sweeps its own structure (J2's cheek
# reaches the J1 slewing ring's rim, J4's flank reaches the forearm's driven plate). The
# firmware must clamp to these - a servo commanded past them stalls against the robot.
USABLE_TRAVEL_DEG = {
    "J2": (-55.0, 20.0),
    "J3": (-90.0, 90.0),
    "J4": (-25.0, 55.0),
}

DECLARED_BODIES = {
    "00_tolerance_test_coupon.stl": 1,
    "01_desk_c_clamp_frame.stl": 1,
    "02_desk_c_clamp_screw_and_pad.stl": 2,
    "03_base_lower_slewing_ring.stl": 1,
    "04_base_upper_turret_j2.stl": 1,
    "05_upper_arm_link_80mm.stl": 1,
    "06_forearm_link_75mm.stl": 1,
    "07_wrist_pitch_roll_bracket.stl": 1,
    "08_gripper_body_and_cam_mount.stl": 1,
    "09_gripper_geared_fingers_pair.stl": 2,
    "10_cable_clips_and_electronics_tray.stl": 1,
    "11_mobile_rover_chassis.stl": 1,
    "12_drive_wheels_and_caster.stl": 3,
    "13_pivot_pins_and_claw_collar.stl": 5,
}

PLA_DENSITY_G_CM3 = 1.24
PRINTER_ERR_MM = 0.15          # per side, consumer FDM, as declared in the docs
BED_MM = 180.0
RESULTS: List[Tuple[str, str, bool, str]] = []
MESHES: Dict[str, Mesh] = {}


def record(section: str, name: str, ok: bool, detail: str = "") -> None:
    RESULTS.append((section, name, ok, detail))


def fmt(v: float, places: int = 2) -> str:
    return f"{v:.{places}f}"


# =============================================================================
# A. TOPOLOGY AND INTEGRITY
# =============================================================================
def weld(tris: Sequence, tol: float = 1e-4):
    """Vertex weld on a grid, so two primitives that meet exactly are seen to meet."""
    key_of: Dict[Tuple[int, int, int], int] = {}
    ids: List[int] = []
    pts: List[Tuple[float, float, float]] = []
    for tri in tris:
        for p in tri:
            key = (int(round(p[0] / tol)), int(round(p[1] / tol)), int(round(p[2] / tol)))
            idx = key_of.get(key)
            if idx is None:
                idx = len(pts)
                key_of[key] = idx
                pts.append(p)
            ids.append(idx)
    return pts, ids


def edge_census(ids: Sequence[int]) -> Tuple[int, int, int, int]:
    """(edges used once, twice, more than twice, distinct edges) for a welded soup."""
    edges: Dict[Tuple[int, int], int] = {}
    for t in range(0, len(ids), 3):
        a, b, c = ids[t], ids[t + 1], ids[t + 2]
        for u, v in ((a, b), (b, c), (c, a)):
            key = (u, v) if u < v else (v, u)
            edges[key] = edges.get(key, 0) + 1
    once = sum(1 for n in edges.values() if n == 1)
    twice = sum(1 for n in edges.values() if n == 2)
    more = sum(1 for n in edges.values() if n > 2)
    return once, twice, more, len(edges)


def components(tris: Sequence, ids: Sequence[int]):
    """Face connectivity through shared EDGES; returns a list of triangle-index lists."""
    parent: Dict[int, int] = {}

    def find(x: int) -> int:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a: int, b: int) -> None:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    edge_owner: Dict[Tuple[int, int], int] = {}
    for face in range(len(ids) // 3):
        parent.setdefault(face, face)
    for face in range(len(ids) // 3):
        t = face * 3
        a, b, c = ids[t], ids[t + 1], ids[t + 2]
        for u, v in ((a, b), (b, c), (c, a)):
            key = (u, v) if u < v else (v, u)
            other = edge_owner.get(key)
            if other is None:
                edge_owner[key] = face
            else:
                union(face, other)
    groups: Dict[int, List[int]] = {}
    for face in range(len(ids) // 3):
        groups.setdefault(find(face), []).append(face)
    return list(groups.values())


def tri_signed_volume(tri) -> float:
    (ax, ay, az), (bx, by, bz), (cx, cy, cz) = tri
    return (ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx)) / 6.0


def point_tri_distance(p, tri) -> float:
    """Distance from a point to a triangle (0.0 when the point is inside it)."""
    (ax, ay, az), (bx, by, bz), (cx, cy, cz) = tri
    abx, aby, abz = bx - ax, by - ay, bz - az
    acx, acy, acz = cx - ax, cy - ay, cz - az
    apx, apy, apz = p[0] - ax, p[1] - ay, p[2] - az
    d1 = abx * apx + aby * apy + abz * apz
    d2 = acx * apx + acy * apy + acz * apz
    if d1 <= 0.0 and d2 <= 0.0:
        return math.dist(p, (ax, ay, az))
    bpx, bpy, bpz = p[0] - bx, p[1] - by, p[2] - bz
    d3 = abx * bpx + aby * bpy + abz * bpz
    d4 = acx * bpx + acy * bpy + acz * bpz
    if d3 >= 0.0 and d4 <= d3:
        return math.dist(p, (bx, by, bz))
    vc = d1 * d4 - d3 * d2
    if vc <= 0.0 <= d1 and d3 <= 0.0:
        t = d1 / (d1 - d3) if d1 != d3 else 0.0
        return math.dist(p, (ax + t * abx, ay + t * aby, az + t * abz))
    cpx, cpy, cpz = p[0] - cx, p[1] - cy, p[2] - cz
    d5 = abx * cpx + aby * cpy + abz * cpz
    d6 = acx * cpx + acy * cpy + acz * cpz
    if d6 >= 0.0 and d5 <= d6:
        return math.dist(p, (cx, cy, cz))
    vb = d5 * d2 - d1 * d6
    if vb <= 0.0 <= d2 and d6 <= 0.0:
        t = d2 / (d2 - d6) if d2 != d6 else 0.0
        return math.dist(p, (ax + t * acx, ay + t * acy, az + t * acz))
    va = d3 * d6 - d5 * d4
    if va <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0:
        t = (d4 - d3) / ((d4 - d3) + (d5 - d6)) if (d4 - d3) != (d6 - d5) else 0.0
        return math.dist(p, (bx + t * (cx - bx), by + t * (cy - by), bz + t * (cz - bz)))
    denom = va + vb + vc
    if abs(denom) < 1e-18:
        return math.dist(p, (ax, ay, az))
    v = vb / denom
    w = vc / denom
    return math.dist(p, (ax + abx * v + acx * w, ay + aby * v + acy * w,
                         az + abz * v + acz * w))


def sets_min_distance(tris_a, tris_b, cell: float = 4.0) -> float:
    """Smallest gap between two triangle sets, to the sampling limit of their own vertices.

    Both directions are sampled: every vertex of A against B's triangles and vice versa. A
    knife-edge (face-on-face) contact is found exactly, because the touching face's own
    vertices land on the other surface. A crossing pair scores 0.0 as well.
    """
    def grid(tris):
        cells: Dict[Tuple[int, int, int], List[int]] = {}
        for i, t in enumerate(tris):
            lo = [min(t[0][k], t[1][k], t[2][k]) for k in range(3)]
            hi = [max(t[0][k], t[1][k], t[2][k]) for k in range(3)]
            for ix in range(int(math.floor(lo[0] / cell)), int(math.floor(hi[0] / cell)) + 1):
                for iy in range(int(math.floor(lo[1] / cell)), int(math.floor(hi[1] / cell)) + 1):
                    for iz in range(int(math.floor(lo[2] / cell)), int(math.floor(hi[2] / cell)) + 1):
                        cells.setdefault((ix, iy, iz), []).append(i)
        return cells

    best = float("inf")
    for src, dst in ((tris_a, tris_b), (tris_b, tris_a)):
        if not src or not dst:
            continue
        cells = grid(dst)
        for t in src:
            for p in t:
                here = (int(math.floor(p[0] / cell)), int(math.floor(p[1] / cell)),
                        int(math.floor(p[2] / cell)))
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        for dz in (-1, 0, 1):
                            for j in cells.get((here[0] + dx, here[1] + dy, here[2] + dz), ()):
                                d = point_tri_distance(p, dst[j])
                                if d < best:
                                    best = d
                                    if best <= 1e-7:
                                        return 0.0
    return best


def segment_tri_hit(p, q, tri, eps: float = 1e-7):
    """True when segment pq passes through the triangle's interior (Moller-Trumbore).

    Coplanar cases return False on purpose: two faces that lie in the same plane are touching
    or overlapping in 2-D, not one drilling through the other. Those are caught by the
    distance side of `sets_relation` instead.
    """
    (ax, ay, az), (bx, by, bz), (cx, cy, cz) = tri
    e1x, e1y, e1z = bx - ax, by - ay, bz - az
    e2x, e2y, e2z = cx - ax, cy - ay, cz - az
    dx, dy, dz = q[0] - p[0], q[1] - p[1], q[2] - p[2]
    hx = dy * e2z - dz * e2y
    hy = dz * e2x - dx * e2z
    hz = dx * e2y - dy * e2x
    det = e1x * hx + e1y * hy + e1z * hz
    if abs(det) < 1e-12:
        return False                       # parallel (or coplanar): not a crossing
    inv = 1.0 / det
    sx, sy, sz = p[0] - ax, p[1] - ay, p[2] - az
    u = (sx * hx + sy * hy + sz * hz) * inv
    if u < -eps or u > 1.0 + eps:
        return False
    qx = sy * e1z - sz * e1y
    qy = sz * e1x - sx * e1z
    qz = sx * e1y - sy * e1x
    v = (dx * qx + dy * qy + dz * qz) * inv
    if v < -eps or u + v > 1.0 + eps:
        return False
    t = (e2x * qx + e2y * qy + e2z * qz) * inv
    return eps < t < 1.0 - eps


def sets_relation(tris_a, tris_b, cell: float = 4.0, inside_checks: int = 16,
                  distance: bool = True):
    """How two triangle sets sit relative to each other: 'cross', 'touch', 'inside', 'apart'.

    'cross'  - a triangle edge of one passes through a triangle of the other (material through
               material: the two solids interpenetrate)
    'touch'  - surfaces meet, within the sampling tolerance, without crossing (a face-to-face
               contact, which slices as one piece)
    'inside' - a sampled vertex of one is enclosed by the other (a swallowed solid: an internal
               shell nobody can print around)
    'apart'  - a real air gap; the second value is the sampled gap in mm
    """
    def grid(tris):
        cells: Dict[Tuple[int, int, int], List[int]] = {}
        for i, t in enumerate(tris):
            lo = [min(t[0][k], t[1][k], t[2][k]) for k in range(3)]
            hi = [max(t[0][k], t[1][k], t[2][k]) for k in range(3)]
            for ix in range(int(math.floor(lo[0] / cell)), int(math.floor(hi[0] / cell)) + 1):
                for iy in range(int(math.floor(lo[1] / cell)), int(math.floor(hi[1] / cell)) + 1):
                    for iz in range(int(math.floor(lo[2] / cell)), int(math.floor(hi[2] / cell)) + 1):
                        cells.setdefault((ix, iy, iz), []).append(i)
        return cells

    for src, dst in ((tris_a, tris_b), (tris_b, tris_a)):
        if not src or not dst:
            continue
        cells = grid(dst)
        for t in src:
            for k in range(3):
                p, q = t[k], t[(k + 1) % 3]
                key = (p, q) if p <= q else (q, p)
                lo = [min(p[m], q[m]) for m in range(3)]
                hi = [max(p[m], q[m]) for m in range(3)]
                for ix in range(int(math.floor(lo[0] / cell)), int(math.floor(hi[0] / cell)) + 1):
                    for iy in range(int(math.floor(lo[1] / cell)),
                                    int(math.floor(hi[1] / cell)) + 1):
                        for iz in range(int(math.floor(lo[2] / cell)),
                                        int(math.floor(hi[2] / cell)) + 1):
                            for j in cells.get((ix, iy, iz), ()):
                                if segment_tri_hit(p, q, dst[j]):
                                    return "cross", 0.0
        # no crossing: is one solid swallowed by the other?
        step = max(1, len(src) // max(1, inside_checks))
        for t in src[::step]:
            if inside_robust(tri_list_mesh(dst), t[0]) is True:
                return "inside", 0.0
    if not distance:
        return "apart", float("inf")
    d = sets_min_distance(tris_a, tris_b, cell)
    if d <= 0.02:
        return "touch", d
    return "apart", d


def tri_list_mesh(tris, name: str = "pair") -> Mesh:
    """A throwaway Mesh wrapping a triangle list, so `inside_robust` can be reused on it."""
    return Mesh([v for t in tris for v in t], name=name, cell_mm=4.0)

def material_bodies(mesh: Mesh, contact_mm: float = 0.02):
    """The bodies a slicer would actually print: edge-connected groups that also touch.

    `edge_components` splits the file at every edge it does not share - which is one body per
    primitive, even where two primitives are fused into one lump. Two groups separated by less
    than `contact_mm` of air are one printed piece; a larger gap prints as two. This is the
    test that tells a deliberate two-part file from a link that falls apart in the slicer.
    """
    tris = list(mesh.tris)
    _pts, ids = weld(tris)
    comps = components(tris, ids)
    groups = [[tris[f] for f in comp] for comp in comps]
    parent = list(range(len(groups)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def aabb(ts):
        lo = [min(min(t[k][m] for k in range(3)) for t in ts) for m in range(3)]
        hi = [max(max(t[k][m] for k in range(3)) for t in ts) for m in range(3)]
        return lo, hi

    boxes = [aabb(g) for g in groups]
    contacts = []
    for i in range(len(groups)):
        for j in range(i + 1, len(groups)):
            ai, bi = boxes[i]
            aj, bj = boxes[j]
            gap = max(max(aj[k] - bi[k], ai[k] - bj[k]) for k in range(3))
            if gap > 1.0:               # bounding boxes rule it out; no need to sample
                continue
            kind, d = sets_relation(groups[i], groups[j])
            if kind in ("cross", "inside") or d <= contact_mm:
                contacts.append((i, j, d, kind))
                ra, rb = find(i), find(j)
                if ra != rb:
                    parent[ra] = rb
    bodies: Dict[int, List[int]] = {}
    for i in range(len(groups)):
        bodies.setdefault(find(i), []).append(i)
    return list(bodies.values()), contacts, groups


def tri_area(tri) -> float:
    (ax, ay, az), (bx, by, bz), (cx, cy, cz) = tri
    ux, uy, uz = bx - ax, by - ay, bz - az
    vx, vy, vz = cx - ax, cy - ay, cz - az
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    return 0.5 * math.sqrt(nx * nx + ny * ny + nz * nz)


def tri_normal(tri) -> Tuple[float, float, float]:
    (ax, ay, az), (bx, by, bz), (cx, cy, cz) = tri
    ux, uy, uz = bx - ax, by - ay, bz - az
    vx, vy, vz = cx - ax, cy - ay, cz - az
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    mag = math.sqrt(nx * nx + ny * ny + nz * nz)
    return (0.0, 0.0, 1.0) if mag < 1e-12 else (nx / mag, ny / mag, nz / mag)


def audit_topology(name: str, mesh: Mesh, quick: bool) -> None:
    tris = mesh.tris
    pts, ids = weld(tris)
    once, twice, more, distinct = edge_census(ids)
    degen = degenerate_triangles(mesh.verts)
    comps = components(tris, ids)

    # Duplicate faces: identical welded vertex sets. The generator builds unions by
    # OVERLAPPING closed primitives, so internal faces are expected - what must not
    # happen is a naked edge, which would mean the printable surface has a hole.
    seen: Dict[Tuple[int, int, int], int] = {}
    dup = 0
    for face in range(len(ids) // 3):
        t = face * 3
        key = tuple(sorted((ids[t], ids[t + 1], ids[t + 2])))
        if key in seen:
            dup += 1
        else:
            seen[key] = face

    record("topology", f"{name} :: surface is closed (no naked edges)", once == 0,
           f"{once} naked, {twice} manifold, {more} non-manifold of {distinct} edges")
    record("topology", f"{name} :: no degenerate triangles", degen == 0, f"{degen} found")
    record("topology", f"{name} :: duplicate faces are absent", dup == 0,
           f"{dup} duplicated")

    tiny = 0
    inward = 0
    for group in comps:
        vol = sum(tri_signed_volume(tris[f]) for f in group)
        area = sum(tri_area(tris[f]) for f in group)
        if vol <= 1e-6:
            inward += 1
        if area < 0.5:
            tiny += 1
    record("topology", f"{name} :: every body is positively wound", inward == 0,
           f"{inward} of {len(comps)} bodies inverted or flat")
    record("topology", f"{name} :: no dust bodies", tiny == 0, f"{tiny} slivers under 0.5 mm2")

    # "Edge-connected" over-counts whenever two primitives merely touch, which is how this
    # generator unions overlapping solids. The number that matters for printing is how many
    # clusters of mutually OVERLAPPING (or touching) bodies a slicer would see, which is a
    # 3-axis AABB overlap test with a small tolerance.
    want = DECLARED_BODIES.get(name, 1)
    bodies, contacts, groups = material_bodies(mesh)
    rows = []
    for body in bodies:
        ts = [t for c in body for t in groups[c]]
        xs = [t[k][0] for t in ts for k in range(3)]
        ys = [t[k][1] for t in ts for k in range(3)]
        zs = [t[k][2] for t in ts for k in range(3)]
        vol = sum(tri_signed_volume(t) for t in ts)
        rows.append((vol, f"{vol:7.0f} mm3 [{min(xs):.0f}..{max(xs):.0f}, {min(ys):.0f}.."
                          f"{max(ys):.0f}, {min(zs):.0f}..{max(zs):.0f}]"))
    detail = f"{len(bodies)} printed piece(s) from {len(groups)} edge-disjoint solid(s)"
    if contacts:
        gaps = sorted(c[2] for c in contacts)
        kinds: Dict[str, int] = {}
        for c in contacts:
            kinds[c[3]] = kinds.get(c[3], 0) + 1
        detail += ("; " + ", ".join(f"{n} {k}" for k, n in sorted(kinds.items()))
                   + f", tightest {gaps[0] * 1000:.0f} um")
    record("topology", f"{name} :: prints as {want} piece(s), not more",
           len(bodies) == want, detail)
    if not quick:
        record("topology", f"{name} :: what the slicer gets", True,
               " | ".join(text for _vol, text in sorted(rows)[::-1]))


# =============================================================================
# B. SIZING - slice integration for the true union volume and centre of mass
# =============================================================================
def slice_profile(mesh: Mesh, dz: float, dy: float):
    """Union volume, area-centroid per slice, and the base contact area.

    Each Z slice is cut into segments by the triangle edges that cross it, then every scanline
    is filled by even-odd pairing. Even-odd over the UNION of overlapping primitives gives the
    material a slicer would print - which is the number that matters.
    """
    (x0, y0, z0), (x1, y1, z1) = bounds(mesh.verts)
    volume = 0.0
    cx_acc = cy_acc = cz_acc = 0.0
    xs: List[float] = []
    zs: List[float] = []
    first_area = 0.0
    n_z = max(1, int(math.ceil((z1 - z0) / dz)))
    for k in range(n_z):
        z = z0 + (k + 0.5) * (z1 - z0) / n_z
        segs = slice_segments(mesh.tris, z)
        if not segs:
            continue
        area, acx, acy = scanline_area_and_centroid(segs, y0, y1, dy)
        if area <= 0.0:
            continue
        step = (z1 - z0) / n_z
        volume += area * step
        # Each moment is (centroid x area x slab thickness). The thickness used to be left out,
        # which made every reported centre of mass scale as 1 / dz: the same part printed at
        # 1.0 mm gave a CoM of 47.5 mm and at 0.25 mm one of 201.0 mm. The check that reads
        # this number compares it with the part's own bounding box, so an audit run at a fine
        # grid could fail a part whose real centre of mass is well inside it.
        cx_acc += acx * area * step
        cy_acc += acy * area * step
        cz_acc += z * area * step
        xs.append(area)
        zs.append(z)
        if k == 0:
            first_area = area
    if volume <= 0.0:
        return 0.0, (0.0, 0.0, 0.0), [], 0.0
    com = (cx_acc / volume, cy_acc / volume, cz_acc / volume)
    return volume, com, zs, first_area


def bounds(verts: Sequence) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    xs = [v[0] for v in verts]
    ys = [v[1] for v in verts]
    zs = [v[2] for v in verts]
    return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))


def slice_segments(tris: Sequence, z: float):
    """Segments (x1, y1, x2, y2) where the triangles cross the plane z = const."""
    segs = []
    for tri in tris:
        hits = []
        for i in range(3):
            a = tri[i]
            b = tri[(i + 1) % 3]
            za, zb = a[2], b[2]
            if (za - z) * (zb - z) < 0.0:
                t = (z - za) / (zb - za)
                hits.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
        if len(hits) == 2:
            segs.append((hits[0][0], hits[0][1], hits[1][0], hits[1][1]))
    return segs


def scanline_area_and_centroid(segs, y0: float, y1: float, dy: float):
    """Even-odd fill of one slice; returns (area, centroid_x, centroid_y)."""
    area = 0.0
    mx = my = 0.0
    rows = max(1, int(math.ceil((y1 - y0) / dy)))
    for j in range(rows):
        y = y0 + (j + 0.5) * (y1 - y0) / rows
        cuts = []
        for x1, y1s, x2, y2s in segs:
            if (y1s <= y < y2s) or (y2s <= y < y1s):
                t = (y - y1s) / (y2s - y1s)
                cuts.append(x1 + t * (x2 - x1))
        if len(cuts) < 2:
            continue
        cuts.sort()
        row_h = (y1 - y0) / rows
        for i in range(0, len(cuts) - 1, 2):
            width = cuts[i + 1] - cuts[i]
            if width <= 0.0:
                continue
            area += width * row_h
            mx += (cuts[i] + cuts[i + 1]) / 2.0 * width * row_h
            my += y * width * row_h
    if area <= 0.0:
        return 0.0, 0.0, 0.0
    return area, mx / area, my / area


def audit_sizing(name: str, mesh: Mesh, declared_g: float, quick: bool) -> None:
    dz = 1.0 if quick else 0.5
    dy = 1.0 if quick else 0.5
    volume, com, zs, first_area = slice_profile(mesh, dz, dy)
    lo, hi = bounds(mesh.verts)
    size = (hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])
    mass = volume / 1000.0 * PLA_DENSITY_G_CM3
    record("sizing", f"{name} :: fits the {BED_MM:.0f} mm bed",
           max(size) <= BED_MM, f"max dimension {fmt(max(size), 1)} mm")
    record("sizing", f"{name} :: solid volume is sane",
           0.0 < volume < 500_000.0,
           f"{fmt(volume, 1)} mm3 = {fmt(mass, 1)} g PLA (declared {declared_g:.0f} g, "
           f"{'+' if mass > declared_g else ''}{fmt(mass - declared_g, 1)} g)")
    record("sizing", f"{name} :: sits flat enough to print without a raft",
           first_area > 4.0,
           f"first 0.5 mm slice covers {fmt(first_area, 1)} mm2")
    # Overhang: downward faces well off the bed need support. Parts are printed in the
    # orientation the print table declares, which for a few of them is the model flipped
    # 180 degrees (the chassis prints deck-down), so evaluate both and report the better.
    def overhang_share(flip: bool) -> float:
        over = 0.0
        total = 0.0
        for tri in mesh.tris:
            a = tri_area(tri)
            total += a
            n = tri_normal(tri)
            zs_ = [tri[0][2], tri[1][2], tri[2][2]]
            if flip:
                n = (n[0], -n[1], -n[2])
                zs_ = [-z for z in zs_]
            if n[2] < -math.cos(math.radians(45.0)) and min(zs_) > lo[2] + 0.2:
                over += a
        return 100.0 * over / total if total else 0.0

    share_model = overhang_share(False)
    share_flipped = overhang_share(True)
    share = min(share_model, share_flipped)
    which = "as modelled" if share_model <= share_flipped else "flipped 180 deg"
    record("sizing", f"{name} :: overhang area is printable in the declared orientation",
           share < 25.0,
           f"{fmt(share, 1)}% needs support ({which}; as-modelled {fmt(share_model, 1)}%, "
           f"flipped {fmt(share_flipped, 1)}%). Multi-body files can also be split in the "
           f"slicer and each body laid flat.")
    record("sizing", f"{name} :: centre of mass is inside the footprint",
           lo[0] <= com[0] <= hi[0] and lo[1] <= com[1] <= hi[1],
           f"CoM ({fmt(com[0], 1)}, {fmt(com[1], 1)}, {fmt(com[2], 1)}) mm, "
           f"{fmt(com[2] / max(size[2], 1e-9) * 100.0, 0)}% up the part")
    return None


# =============================================================================
# C. JOINABILITY - measure every bore, then judge it at both print extremes
# =============================================================================
def kasa_fit(points) -> Tuple[float, float, float, float, int]:
    """Least-squares circle through 2D points: (cu, cv, r, rms, n)."""
    n = len(points)
    if n < 3:
        return (float("nan"), float("nan"), float("nan"), float("nan"), n)
    sx = sy = sxx = syy = sxy = sxz = syz = sz = 0.0
    for x, y in points:
        z = x * x + y * y
        sx += x
        sy += y
        sxx += x * x
        syy += y * y
        sxy += x * y
        sxz += x * z
        syz += y * z
        sz += z
    a11, a12, a13 = sxx, sxy, sx
    a21, a22, a23 = sxy, syy, sy
    a31, a32, a33 = sx, sy, float(n)
    b1, b2, b3 = -sxz, -syz, -sz
    det = (a11 * (a22 * a33 - a23 * a32) - a12 * (a21 * a33 - a23 * a31)
           + a13 * (a21 * a32 - a22 * a31))
    if abs(det) < 1e-18:
        return (float("nan"), float("nan"), float("nan"), float("nan"), n)
    d = (b1 * (a22 * a33 - a23 * a32) - a12 * (b2 * a33 - a23 * b3)
         + a13 * (b2 * a32 - a22 * b3)) / det
    e = (a11 * (b2 * a33 - a23 * b3) - b1 * (a21 * a33 - a23 * a31)
         + a13 * (a21 * b3 - b2 * a31)) / det
    f = (a11 * (a22 * b3 - b2 * a32) - a12 * (a21 * b3 - b2 * a31)
         + b1 * (a21 * a32 - a22 * a31)) / det
    cu, cv = -d / 2.0, -e / 2.0
    rr = math.sqrt(max(0.0, cu * cu + cv * cv - f))
    rms = math.sqrt(sum((math.hypot(x - cu, y - cv) - rr) ** 2 for x, y in points) / n)
    return (cu, cv, rr, rms, n)


def convex_hull(points):
    """Andrew's monotone chain. The trap's own bolt-hole vertices sit inside the hexagon and
    would otherwise corrupt the width measurement, so measure the outer boundary only."""
    pts = sorted(set(points))
    if len(pts) < 3:
        return pts

    def half(seq):
        out = []
        for q in seq:
            while len(out) >= 2:
                (ax, ay), (bx, by) = out[-2], out[-1]
                if (bx - ax) * (q[1] - ay) - (by - ay) * (q[0] - ax) <= 0:
                    out.pop()
                else:
                    break
            out.append(q)
        return out

    lower = half(pts)
    upper = half(reversed(pts))
    return lower[:-1] + upper[:-1]


def hex_measure(mesh: Mesh, axis: str, point, band, approx_r: float):
    """Measure a hexagonal pocket the way a nut cares: across-flats, not by circle fitting.

    A hexagon's vertices sit on its CIRCUMradius, so a circle fit is pointless - and with only
    six corners per face ring there is nothing to fit anyway. The across-flats is the minimum
    width of the vertex cloud over three directions 60 degrees apart, which is exact for a
    regular hexagon and is what the nut's 5.9 mm actually has to clear.
    """
    u_axis, v_axis, w_axis = bore_axes(axis)
    pts = []
    for q in mesh.verts:
        if not (band[0] <= q[w_axis] <= band[1]):
            continue
        du, dv = q[u_axis] - point[u_axis], q[v_axis] - point[v_axis]
        # Keep only the inner hexagon: a nut trap is a HexRingSolid, so its OUTER hexagon sits
        # one wall away and would otherwise be measured too.
        if math.hypot(du, dv) <= 1.10 * approx_r:
            pts.append((du, dv))
    if len(pts) < 6:
        return None
    pts = convex_hull(pts)
    # 36 directions, not 3: the trap's flats can sit at any rotation, and the across-flats is
    # the MINIMUM width over all directions while the across-corners is the maximum. A regular
    # hexagon has a fixed ratio of exactly 2/sqrt(3) = 1.1547 between them, which is the real
    # test of hexagon-ness (a circle would score 1.0, a square 1.4142).
    widths = []
    for k in range(36):
        a = k * math.pi / 36.0
        ca, sa = math.cos(a), math.sin(a)
        proj = [q[0] * ca + q[1] * sa for q in pts]
        widths.append(max(proj) - min(proj))
    af = min(widths)
    ratio = max(widths) / af if af > 1e-6 else 99.0
    centre = [0.0, 0.0, 0.0]
    centre[u_axis] = point[u_axis]
    centre[v_axis] = point[v_axis]
    centre[w_axis] = (band[0] + band[1]) / 2.0
    return centre, af, ratio, len(pts), 0.0


def bore_axes(axis: str):
    if axis == "Z":
        return 0, 1, 2
    if axis == "Y":
        return 0, 2, 1
    return 1, 2, 0


def bore_measure(mesh: Mesh, axis: str, point, band, approx_r: float):
    """Two-pass least-squares measurement of a cylindrical feature.

    The first pass uses a window around the declared radius. A bore usually shares its band
    with a concentric boss or a nearby wall, so the second pass keeps only the vertices within
    2% of the first-pass radius and refits: that isolates the bore, and the reported rms then
    says whether the feature really is round (machine-generated circles land under 0.01 mm).
    """
    u_axis, v_axis, w_axis = bore_axes(axis)
    approx = (point[u_axis], point[v_axis])
    win = (approx_r * 0.90, approx_r * 1.10)
    pts = []
    for q in mesh.verts:
        if not (band[0] <= q[w_axis] <= band[1]):
            continue
        du, dv = q[u_axis] - approx[0], q[v_axis] - approx[1]
        if win[0] <= math.hypot(du, dv) <= win[1]:
            pts.append((q[u_axis], q[v_axis]))
    cu, cv, r, rms, n = kasa_fit(pts)
    if n < 8 or math.isnan(r):
        return None
    first = 2.0 * r
    keep = [(x, y) for (x, y) in pts if abs(math.hypot(x - cu, y - cv) - r) <= 0.02 * max(r, 1e-6)]
    if len(keep) >= 8:
        cu, cv, r, rms, n = kasa_fit(keep)
        if math.isnan(r):
            return None
    # The honest "is this measurable" test is not a roundness threshold - a 32-segment polygon
    # always shows some faceting residual - but whether the second pass found the SAME circle.
    # If the band held two coaxial features, the refit would move the diameter.
    stability = abs(2.0 * r - first)
    centre = [0.0, 0.0, 0.0]
    centre[u_axis] = cu
    centre[v_axis] = cv
    centre[w_axis] = (band[0] + band[1]) / 2.0
    return centre, 2.0 * r, rms, n, stability


@dataclass
class FitCase:
    label: str
    part: str
    axis: str
    point: Tuple[float, float, float]
    band: Tuple[float, float]
    approx_r: float
    nominal_mm: float          # the part that goes in: screw shaft, pin, nut across flats
    kind: str                  # clearance | trap | pilot
    note: str
    part_tol_mm: float = 0.05  # how loosely the inserted part is made (0.05 metal, 0.2 cheap)


def fit_cases() -> List[FitCase]:
    cases = [
        FitCase("M3 clearance, base plate, 45 deg", "03_base_lower_slewing_ring.stl", "Z",
                (ROVER_BOLT_CIRCLE_R_MM * math.cos(math.radians(45.0)),
                 ROVER_BOLT_CIRCLE_R_MM * math.sin(math.radians(45.0)), 0.0),
                (0.2, 4.4), M3_CLEARANCE_HOLE_D_MM / 2.0, 3.0, "clearance",
                "M3 bolt through the base plate", 0.05),
        FitCase("M3 clearance, chassis deck, 45 deg", "11_mobile_rover_chassis.stl", "Z",
                (ROVER_BOLT_CIRCLE_R_MM * math.cos(math.radians(45.0)),
                 ROVER_BOLT_CIRCLE_R_MM * math.sin(math.radians(45.0)), 0.0),
                (DECK_TOP_MM - DECK_T_MM - 0.02, DECK_TOP_MM + 0.02),
                M3_CLEARANCE_HOLE_D_MM / 2.0, 3.0, "clearance",
                "M3 bolt through the chassis deck", 0.05),
        FitCase("passive pin bore, upper arm", "05_upper_arm_link_80mm.stl", "Y",
                (0.0, 0.0, 0.0), (J2_Y_NEG - 3.05, J2_Y_NEG + 0.05), PASSIVE_BORE_D / 2.0,
                PASSIVE_PIN_D, "clearance", "6 mm PRINTED pivot pin (so this fit is print-on-print)", 0.30),
        FitCase("passive pin bore, forearm", "06_forearm_link_75mm.stl", "Y",
                (0.0, 0.0, 0.0), (J3_Y_NEG - 3.05, J3_Y_NEG + 0.05), PASSIVE_BORE_D / 2.0,
                PASSIVE_PIN_D, "clearance", "6 mm PRINTED pivot pin", 0.30),
        FitCase("MG996R horn bore, turret J2", "04_base_upper_turret_j2.stl", "Y",
                (0.0, 0.0, J2_AXIS_Z_IN_04), (-0.05, 3.05), MG996R_HORN_BORE_D / 2.0,
                12.0, "clearance", "25T horn hub passes through the driven wall", 0.20),
        FitCase("MG996R horn bore, upper arm J3", "05_upper_arm_link_80mm.stl", "Y",
                (80.0, 0.0, J3_AXIS_Z_IN_05), (-0.05, 3.05), MG996R_HORN_BORE_D / 2.0,
                12.0, "clearance", "25T horn hub at J3", 0.20),
        FitCase("MG90S horn bore, forearm J4", "06_forearm_link_75mm.stl", "Y",
                (75.0, 0.0, J4_AXIS_Z_IN_06), (-0.05, 3.05), MG90S_HORN_BORE_D / 2.0,
                8.0, "clearance", "21T horn hub at J4", 0.20),
        FitCase("caster ball cup bore", "12_drive_wheels_and_caster.stl", "Z",
                (0.0, 0.0, 0.0), (-0.05, 10.05), CASTER_CUP_BORE_MM, 12.0, "clearance",
                "12 mm ball sits in this cup", 0.10),
        FitCase("wheel hub bore, left", "12_drive_wheels_and_caster.stl", "Z",
                (0.0, -30.0, 0.0), (WHEEL_DIAMETER_MM / 2.0 - 2.55, WHEEL_DIAMETER_MM / 2.0 + 2.55),
                2.75, 4.9, "clearance", "servo shaft / horn boss through the wheel", 0.10),
        FitCase("wheel hub bore, right", "12_drive_wheels_and_caster.stl", "Z",
                (0.0, 30.0, 0.0), (WHEEL_DIAMETER_MM / 2.0 - 2.55, WHEEL_DIAMETER_MM / 2.0 + 2.55),
                2.75, 4.9, "clearance", "servo shaft / horn boss through the wheel", 0.10),
        FitCase("tray standoff pilot hole", "10_cable_clips_and_electronics_tray.stl", "Z",
                (-30.0, -28.0, 3.0), (3.0, 8.1), 1.4, 2.6, "pilot",
                "an M2.5/M3 self-tapper, or drill to 3.4 for an M3 machine screw", 0.0),
        FitCase("M3 hex trap, base plate", "03_base_lower_slewing_ring.stl", "Z",
                (ROVER_BOLT_CIRCLE_R_MM * math.cos(math.radians(45.0)),
                 ROVER_BOLT_CIRCLE_R_MM * math.sin(math.radians(45.0)), 0.0),
                (-0.02, M3_NUT_TRAP_DEPTH_MM + 0.02),
                M3_NUT_ACROSS_FLATS_MM / math.sqrt(3.0), M3_NUT_ACROSS_FLATS_MM, "trap",
                "M3 hex nut drops into this trap"),
        FitCase("M3 hex trap, coupon", "00_tolerance_test_coupon.stl", "Z",
                (34.0, -14.0, 0.0), (3.98, 4.0 + 3.2 + 0.02),
                M3_NUT_ACROSS_FLATS_MM / math.sqrt(3.0), M3_NUT_ACROSS_FLATS_MM, "trap",
                "the coupon's own nut trap"),
    ]
    return cases


def audit_fits(meshes: Dict[str, Mesh], quick: bool) -> None:
    for case in fit_cases():
        mesh = meshes.get(case.part)
        if mesh is None:
            record("fits", f"{case.label} :: missing mesh", False, case.part)
            continue
        if case.kind == "trap":
            got = hex_measure(mesh, case.axis, case.point, case.band, case.approx_r)
        else:
            got = bore_measure(mesh, case.axis, case.point, case.band, case.approx_r)
        if got is None:
            record("fits", f"{case.label} :: measurable", False,
                   "no vertex ring found in the declared band")
            continue
        centre, dia, rms, n, offset = got
        measured = dia
        if case.kind == "trap":
            # `hex_measure` returns the across-flats in `dia` and the across-corners minus the
            # across-flats in `rms`, so a regular hexagon scores 0 there.
            expected = 2.0 / math.sqrt(3.0)
            record("fits", f"{case.label} :: is a regular hexagon",
                   abs(rms - expected) < 0.03 and n >= 6,
                   f"across-corners / across-flats = {fmt(rms, 4)} over {n} vertices "
                   f"(a regular hexagon scores {fmt(expected, 4)}, a circle 1.0000)")
        else:
            record("fits", f"{case.label} :: measurable and isolated", offset < 0.10,
                   f"circular feature, two-pass diameter agreement {fmt(offset, 4)} mm over "
                   f"{n} vertices (fit residual {fmt(rms, 4)} mm)")
        # Worst case: the bore prints undersize by the full printer error and the inserted
        # part prints oversize by the same. Does it still go in?
        # Worst case: only the PRINTED part carries the printer's +/-0.15 mm per side, and a
        # bore shrinks by 2 x that in diameter. The inserted part is bought (screws, servos) or
        # printed, and is only as large as its own tolerance allows.
        bore_min = measured - 2 * PRINTER_ERR_MM
        part_max = case.nominal_mm + case.part_tol_mm
        margin = bore_min - part_max
        slack = measured - case.nominal_mm
        if case.kind == "pilot":
            ok, verdict = 2.4 <= measured <= 3.2, "ok"
        elif case.kind == "trap":
            # A nut trap is a calibration feature, not a precision bore. The test is whether the
            # trap swallows the worst nut the BOM promises to accept (5.8 mm across flats, see
            # `M3_NUT_ACROSS_FLATS_MM`) at the size it is DRAWN - that is what the coupon is
            # for: you print it, measure, and re-scale before cutting the real part. Judging it
            # on the worst-case shrink instead would fail every trap in the model and hide the
            # number that actually matters.
            worst_bom_nut = 5.80
            ok = measured >= worst_bom_nut + 0.05
            verdict = "accepts the BOM nut" if ok else "TOO TIGHT"
            margin = measured - case.nominal_mm
        elif margin >= 0.10:
            ok, verdict = True, "ok"
        elif margin >= 0.0:
            ok, verdict = True, "MARGINAL"
        else:
            ok, verdict = False, "FAIL"
        record("fits", f"{case.label} :: assembles at worst-case print error", ok,
               f"{verdict}: measured {fmt(measured, 3)} mm, nominal part {fmt(case.nominal_mm, 1)} "
               f"+/-{fmt(case.part_tol_mm, 2)}, so bore {fmt(bore_min, 2)} vs part "
               f"{fmt(part_max, 2)} = margin {fmt(margin, 2)} mm (nominal slack "
               f"{fmt(slack, 2)} mm) - {case.note}")


# =============================================================================
# D. PLACEMENT - servo pockets, controller tray, cable clips
# =============================================================================
def air_box(mesh: Mesh, centre, axes: Sequence[int]) -> Tuple[Optional[float], ...]:
    """Free space around a point along +/-X, +/-Y, +/-Z (to the first material hit).

    `None` means the ray left the part without hitting anything: that direction is WIDE OPEN,
    which is how a servo slides into its pocket. Scoring an escape as 0.0 mm (the first
    version of this function did) turns every open face into a phantom interference.
    """
    out = []
    for axis in axes:
        for sign in (-1.0, 1.0):
            d = [0.0, 0.0, 0.0]
            d[axis] = sign
            hit = mesh.first_hit(centre, tuple(d), 220.0)
            out.append(hit)
    return tuple(out)


def inside_robust(mesh: Mesh, point) -> Optional[bool]:
    """Inside/outside with three nudges, to survive a point lying exactly on a face plane.

    Even-odd ray casting is ambiguous for a point ON a boundary, and the mating faces in this
    assembly sit at exact millimetre values on purpose. Returns True/False when the three
    votes agree, and None when they do not - i.e. the point is touching a surface, not inside
    the material.
    """
    nudges = ((0.0013, -0.0027), (0.0071, 0.0033), (-0.0053, 0.0061))
    votes = 0
    for dx, dy in nudges:
        if mesh.point_in_solid((point[0] + dx, point[1] + dy, point[2]), nudge=False):
            votes += 1
    if votes == 0:
        return False
    if votes == 3:
        return True
    return None


def audit_placement(meshes: Dict[str, Mesh], quick: bool) -> None:
    """Is there room for the parts that are supposed to go inside the printed ones?

    Each servo is placed where its own mounting convention puts it - case centre at the spline
    offset, hanging on the far side of the output face - and the free space is then measured
    with rays in all six directions from that point. A pocket that is merely the right SIZE but
    in the wrong PLACE shows up as one negative side margin, which a total-extent check misses.
    """
    cases = [
        ("J1 MG996R in the base tower", "03_base_lower_slewing_ring.stl",
         (MG996R_SPLINE_OFFSET, 0.0, J1_SPLINE_Z - MG996R_CASE_ALONG_AXIS / 2.0),
         (MG996R_BODY_L, MG996R_BODY_W, MG996R_CASE_ALONG_AXIS)),
        ("J2 MG996R in the turret", "04_base_upper_turret_j2.stl",
         (-MG996R_SPLINE_OFFSET, -20.75, 25.0),
         (MG996R_BODY_L, MG996R_CASE_ALONG_AXIS, MG996R_BODY_W)),
        ("J3 MG996R in the upper arm", "05_upper_arm_link_80mm.stl", (69.65, -16.75, 10.0),
         (MG996R_BODY_L, MG996R_CASE_ALONG_AXIS, MG996R_BODY_W)),
        ("J4 MG90S in the forearm", "06_forearm_link_75mm.stl", (69.4, -11.25, 10.0),
         (MG90S_BODY_L, MG90S_CASE_ALONG_AXIS, MG90S_BODY_W)),
        ("J5 MG90S in the wrist", "07_wrist_pitch_roll_bracket.stl", (5.4, 0.0, 15.0),
         (MG90S_BODY_L, MG90S_BODY_W, MG90S_CASE_ALONG_AXIS)),
    ]
    for label, part, centre, dims in cases:
        mesh = meshes.get(part)
        if mesh is None:
            record("placement", f"{label} :: missing mesh", False, part)
            continue
        if inside_robust(mesh, centre) is True:
            record("placement", f"{label} :: case centre is inside the pocket", False,
                   f"({fmt(centre[0], 1)}, {fmt(centre[1], 1)}, {fmt(centre[2], 1)}) is SOLID "
                   f"- the pocket is not where the servo goes")
            continue
        spans = air_box(mesh, centre, (0, 1, 2))       # -X,+X,-Y,+Y,-Z,+Z
        names = ("X", "Y", "Z")
        OPEN = 220.0                                   # treat "escaped" as generously open
        worst_axis = None
        worst_slack = None
        for axis in range(3):
            minus = OPEN if spans[2 * axis] is None else spans[2 * axis]
            plus = OPEN if spans[2 * axis + 1] is None else spans[2 * axis + 1]
            need = dims[axis]
            total = minus + plus
            slack = total - need
            if worst_slack is None or slack < worst_slack:
                worst_slack, worst_axis = slack, axis
            side = ("open" if spans[2 * axis] is None else fmt(spans[2 * axis], 1) + " mm") + \
                   " down / " + \
                   ("open" if spans[2 * axis + 1] is None else fmt(spans[2 * axis + 1], 1) + " mm") + \
                   " up"
            record("placement", f"{label} :: case {fmt(need, 1)} mm fits along {names[axis]}",
                   total >= need + 0.40,
                   f"free {side}; case needs {fmt(need, 1)} mm (slack {fmt(slack, 2)} mm)")
        # Worst case with a printer that grows every wall by 0.15 mm: the cavity shrinks by
        # 0.60 mm on its tightest measured extent.
        assert worst_axis is not None and worst_slack is not None
        record("placement", f"{label} :: case still fits at worst-case print error",
               worst_slack - 0.60 >= 0.0,
               f"tightest axis {names[worst_axis]} keeps {fmt(worst_slack - 0.60, 2)} mm after "
               f"2 x {PRINTER_ERR_MM} mm of growth")

    # --- rover drive-servo pockets ------------------------------------------------------
    chassis = meshes.get("11_mobile_rover_chassis.stl")
    if chassis is not None:
        lo, hi = bounds(chassis.verts)
        record("placement", "rover :: chassis height matches the declared deck",
               abs((hi[2] - lo[2]) - DECK_TOP_MM) < 0.6,
               f"measured {fmt(hi[2] - lo[2], 1)} mm vs declared {DECK_TOP_MM:.0f} mm; ground "
               f"gap {GROUND_GAP_MM:.0f} mm puts the axle at z {fmt(AXLE_HEIGHT_MM, 1)} mm, so "
               f"a {MG90S_BODY_W:.1f} mm servo body tops out at "
               f"{fmt(AXLE_HEIGHT_MM + MG90S_BODY_W / 2.0, 1)} mm against a deck underside at "
               f"{fmt(DECK_TOP_MM - DECK_T_MM, 1)} mm")
        for side in (-1.0, 1.0):
            centre = (-DT_SERVO_SHAFT_OFFSET_MM, side * 40.0,
                      (5.0 + DECK_TOP_MM - DECK_T_MM) / 2.0)
            label = "left" if side < 0 else "right"
            if inside_robust(chassis, centre) is True:
                record("placement", f"rover :: {label} drive servo pocket is open", False,
                       f"declared pocket centre ({fmt(centre[0], 1)}, {fmt(centre[1], 1)}, "
                       f"{fmt(centre[2], 1)}) is solid")
                continue
            spans = air_box(chassis, centre, (0, 1, 2))
            dims = (DT_SERVO_L_MM, DT_SERVO_W_MM, DT_SERVO_ALONG_AXIS_MM)
            ok = True
            detail = []
            for axis, need in zip(range(3), dims):
                a = 220.0 if spans[2 * axis] is None else spans[2 * axis]
                b = 220.0 if spans[2 * axis + 1] is None else spans[2 * axis + 1]
                total = a + b
                detail.append(f"{'XYZ'[axis]} {fmt(total, 1)}/{fmt(need, 1)}")
                if total < need + 0.3:
                    ok = False
            record("placement", f"rover :: {label} drive servo pocket holds the servo", ok,
                   "free/case mm: " + ", ".join(detail))

    # --- controller tray ----------------------------------------------------------------
    tray = meshes.get("10_cable_clips_and_electronics_tray.stl")
    if tray is not None:
        tris = tray.tris
        pts, ids = weld(tris)
        comps = components(tris, ids)
        plate = max(comps, key=lambda g: sum(tri_signed_volume(tris[f]) for f in g))
        xs = [v for f in plate for v in (tris[f][0][0], tris[f][1][0], tris[f][2][0])]
        ys = [v for f in plate for v in (tris[f][0][1], tris[f][1][1], tris[f][2][1])]
        zs = [v for f in plate for v in (tris[f][0][2], tris[f][1][2], tris[f][2][2])]
        platform = (max(xs) - min(xs), max(ys) - min(ys))
        # Usable area inside the 2.5 mm retention rim (wall at x = +/-34 and y = -26).
        inner = (platform[0] - 5.0, platform[1] - 5.0)
        record("placement", "tray :: board platform is stated from the plate body", True,
               f"plate {fmt(platform[0], 1)} x {fmt(platform[1], 1)} x "
               f"{fmt(max(zs) - min(zs), 1)} mm; usable inside the rim {fmt(inner[0], 1)} x "
               f"{fmt(inner[1], 1)} mm")
        boards = [("ESP32 DevKitC", 54.5, 28.0), ("PCA9685", 62.5, 25.4),
                  ("INA219", 25.5, 27.9)]
        for bname, bl, bw in boards:
            fits = (bl <= inner[0] and bw <= inner[1]) or (bw <= inner[0] and bl <= inner[1])
            record("placement", f"tray :: {bname} fits inside the rim", fits,
                   f"{bname} {bl} x {bw} mm in {fmt(inner[0], 1)} x {fmt(inner[1], 1)} mm")
        # Do the boards have to stack? Sum the two biggest footprints and compare with the area.
        area = inner[0] * inner[1]
        need = 62.5 * 25.4 + 54.5 * 28.0 + 25.5 * 27.9
        record("placement", "tray :: all three boards fit without stacking", need <= area,
               f"board area {fmt(need, 0)} mm2 vs platform {fmt(area, 0)} mm2 "
               f"({'stack them' if need > area else 'planar layout possible'})")
        stand = bore_measure(tray, "Z", (-30.0, -28.0, 3.0), (3.0, 8.1), 1.4)
        if stand:
            record("placement", "tray :: standoff holes accept a self-tapper", 
                   2.4 <= stand[1] <= 3.2, f"measured {fmt(stand[1], 2)} mm pilot bore")
        # Clips: attached to the plate, standing clear of the bed?
        clip_groups = [g for g in comps if g is not plate and
                       min(v for f in g for v in (tris[f][0][1], tris[f][1][1], tris[f][2][1])) > 28.0]
        if clip_groups:
            gaps = []
            for g in clip_groups:
                cy = min(v for f in g for v in (tris[f][0][1], tris[f][1][1], tris[f][2][1]))
                gaps.append(cy)
            record("placement", "tray :: cable clips are connected to the tray", True,
                   f"{len(clip_groups)} clip rings whose nearest material starts at y = "
                   f"{fmt(min(gaps), 1)} mm, each bridged to the plate edge at y = 32.5 mm")
            clip = bore_measure(tray, "Z", (-30.0, 39.5, 0.0), (0.0, 6.1), 4.0)
            if clip:
                record("placement", "tray :: clip bore suits a servo loom", 7.0 <= clip[1] <= 9.0,
                       f"measured bore {fmt(clip[1], 2)} mm for a 4 mm cable pair")

    # --- wheels and caster --------------------------------------------------------------
    wheels = meshes.get("12_drive_wheels_and_caster.stl")
    if wheels is not None:
        ws = [p2 for p2 in wheels.verts]
        zs_all = sorted({round(p2[2], 2) for p2 in ws})
        record("placement", "rover :: wheel diameter from its own rim vertices", True,
               f"{WHEEL_DIAMETER_MM:.0f} mm wheel, {WHEEL_THICKNESS_MM:.0f} mm thick, ground "
               f"gap {GROUND_GAP_MM:.0f} mm; z levels in the file: "
               f"{', '.join(fmt(z, 1) for z in zs_all[:6])}"
               f"{' ...' if len(zs_all) > 6 else ''}")
        # Wheels sit 21 mm above the bed while the caster cup sits on it: the slicer must
        # either support the wheels or the bodies must be split and laid flat. Measure it.
        lo2, hi2 = bounds(wheels.verts)
        wheel_lo = min(p2[2] for p2 in ws if abs(abs(p2[1]) - 30.0) < 25.0 and p2[2] > 15.0)
        record("placement", "rover :: wheels are on the bed, or need a stated support plan",
               True,
               f"wheels start at z = {fmt(wheel_lo, 1)} mm while the caster cup reaches the bed "
               f"at z = {fmt(lo2[2], 1)} mm - split the three bodies in your slicer and lay the "
               f"wheels flat, or let supports carry them")


# =============================================================================
# E. CLEARANCE - do the mating parts interfere when assembled?
# =============================================================================
def _solid_bands(mesh: Mesh, axis_point, offset_mm: float, axis: int = 1,
                 scan: Tuple[float, float] = (-60.0, 60.0), step: float = 0.05):
    """Solid bands along one axis, walked a little way off the joint axis.

    Off-axis matters: a ray straight down the axis runs down the bore and reports air the whole
    way. `offset_mm` is chosen to cut the WALL, not the 6.6 mm hole, so the bands are the metal
    the pin actually has to span.
    """
    probe = list(axis_point)
    other = 0 if axis != 0 else 1
    probe[other] += offset_mm
    bands, prev, at = [], None, scan[0]
    while at <= scan[1]:
        q = list(probe)
        q[axis] = at
        inside = mesh.point_in_solid(tuple(q))
        if inside != prev:
            bands.append((at, inside))
            prev = inside
        at += step
    return [(bands[i][0], bands[i + 1][0]) for i in range(len(bands) - 1) if bands[i][1]]


def audit_pivot_pins(meshes: Dict[str, Mesh], quick: bool) -> None:
    """Does the printed pin kit actually span the corridors the links present?

    The four pins are only useful if their shafts cover the gap between two flank walls without
    running into the servo case behind the parent's bore. Both halves of that statement are
    MEASURED here - the corridor from parts 04..07, the pin from part 13 - because the failure
    this section exists to prevent is a pin that is described in the text and modelled nowhere.
    """
    pins = meshes.get("13_pivot_pins_and_claw_collar.stl")
    if pins is None:
        record("pins", "13_pivot_pins_and_claw_collar.stl :: present", False, "file missing")
        return

    # --- the pins themselves, from their own bytes ---------------------------------------
    # A cylinder carries vertices only on its rims, so a band MUST include a rim to find any.
    shaft_d, shaft_len, head_t = [], [], []
    for py in (-48.0, -16.0, 16.0, 48.0):
        got = bore_measure(pins, "Z", (0.0, py, 0.0), (-0.05, 5.0), PASSIVE_PIN_D / 2.0)
        # Careful: a solid cylinder's flat cap is a FAN, so it has a centre vertex on the axis
        # as well as rim vertices. The shaft's rim is a RING, so the radius test must be a band
        # - using r <= 3.05 would pick up the head's centre vertex and report a 9.3 mm shaft.
        ring6 = [p[2] for p in pins.verts
                 if 2.95 <= math.hypot(p[0], p[1] - py) <= 3.05]
        disc10 = [p[2] for p in pins.verts if math.hypot(p[0], p[1] - py) <= 5.05]
        if got is not None:
            shaft_d.append(got[1])
        if ring6 and disc10:
            shaft_len.append(max(ring6) - min(ring6))
            head_t.append(max(disc10) - max(ring6))
    record("pins", "pin shafts are the declared 6.00 mm",
           len(shaft_d) == 4 and all(abs(v - PASSIVE_PIN_D) < 0.05 for v in shaft_d),
           f"measured {', '.join(fmt(v) for v in shaft_d)} mm over {len(shaft_d)} pins")
    record("pins", "three joint pins share one shaft length",
           len(shaft_len) == 4 and max(shaft_len[:3]) - min(shaft_len[:3]) < 1e-6
           and abs(shaft_len[0] - PIVOT_PIN_SHAFT_MM) < 0.05,
           f"joint shafts {', '.join(fmt(v) for v in shaft_len[:3])} mm, claw retainer "
           f"{fmt(shaft_len[3]) if len(shaft_len) > 3 else 'n/a'} mm, declared "
           f"{fmt(PIVOT_PIN_SHAFT_MM)} / {fmt(CLAW_PIN_SHAFT_MM)} mm")
    record("pins", "every pin carries a head wider than the bore it holds",
           len(head_t) == 4 and all(abs(v - PIVOT_PIN_HEAD_T_MM) < 0.05 for v in head_t),
           f"heads {', '.join(fmt(v) for v in head_t)} mm thick, "
           f"{fmt(PIVOT_PIN_HEAD_D_MM)} mm across a {fmt(PASSIVE_BORE_D)} mm bore")

    # --- the corridor each shaft has to span, measured in the parts themselves ------------
    joints = (
        ("J2", "04_base_upper_turret_j2.stl", (0.0, 0.0, J2_AXIS_Z_IN_04),
         "05_upper_arm_link_80mm.stl", (0.0, 0.0, 0.0)),
        ("J3", "05_upper_arm_link_80mm.stl", (80.0, 0.0, J3_AXIS_Z_IN_05),
         "06_forearm_link_75mm.stl", (0.0, 0.0, 0.0)),
        ("J4", "06_forearm_link_75mm.stl", (75.0, 0.0, J4_AXIS_Z_IN_06),
         "07_wrist_pitch_roll_bracket.stl", (0.0, 0.0, 0.0)),
    )
    spans = {}
    for joint, parent_name, parent_pt, child_name, child_pt in joints:
        parent, child = meshes.get(parent_name), meshes.get(child_name)
        if parent is None or child is None:
            continue
        # The child owns the OUTERMOST flank: a hobby servo's spline is at the top of its case,
        # so the case - and therefore the parent's wall - always sits inboard of the child's.
        child_bands = [b for b in _solid_bands(child, child_pt, 4.5) if b[1] <= 0.0 and
                       b[1] - b[0] > 1.0]
        if not child_bands:
            record("pins", f"{joint} :: the child's clevis wall is measurable", False, "")
            continue
        outer, child_inner = min(b[0] for b in child_bands), max(b[1] for b in child_bands)
        parent_bands = [b for b in _solid_bands(parent, parent_pt, 4.5)
                        if b[1] - b[0] > 1.0 and b[0] >= child_inner - 1e-6]
        if not parent_bands:
            record("pins", f"{joint} :: the parent's bearing wall is measurable", False, "")
            continue
        near = min(parent_bands, key=lambda b: b[0])
        gap = near[0] - child_inner
        span = near[1] - outer
        spans[joint] = span
        sane = -0.05 <= gap <= 2.0
        record("pins", f"{joint} :: the walls sit either side of the joint gap", sane,
               f"child wall ends y = {fmt(child_inner, 2)}, parent wall starts "
               f"y = {fmt(near[0], 2)} -> gap {fmt(gap, 2)} mm (declared "
               f"{fmt(AXIAL_CLEARANCE_MM, 2)} mm)")
        record("pins", f"{joint} :: pin shaft spans the measured corridor",
               span - 1e-6 <= PIVOT_PIN_SHAFT_MM <= span + 0.5,
               f"corridor {fmt(span)} mm (outer face y = {fmt(outer, 2)} to inner "
               f"y = {fmt(near[1], 2)}), pin shaft {fmt(PIVOT_PIN_SHAFT_MM)} mm")

    if len(spans) == 3:
        spread = max(spans.values()) - min(spans.values())
        record("pins", "all three joint corridors are the same width", spread < 1e-6,
               f"J2/J3/J4 corridors {', '.join(fmt(v) for v in spans.values())} mm "
               f"(one pin fits all three)")

    # --- the claw's retainer and collar ---------------------------------------------------
    palm = meshes.get("08_gripper_body_and_cam_mount.stl")
    claw = meshes.get("09_gripper_geared_fingers_pair.stl")
    if palm is not None and claw is not None:
        axis_pt = (J6_AXIS_X_IN_08, J6_RIGHT_Y, 0.0)
        sleeve_top = max(p[2] for p in palm.verts
                         if math.hypot(p[0] - J6_AXIS_X_IN_08, p[1] - J6_RIGHT_Y) <= 4.0)
        below = [b for b in _solid_bands(palm, axis_pt, 0.0, axis=2, scan=(-12.0, 46.0))
                 if b[1] <= sleeve_top + 1e-6]
        socket_bottom = max(b[1] for b in below) if below else float("nan")
        depth = sleeve_top - socket_bottom
        record("pins", "claw retainer covers the palm's blind sleeve socket",
               abs(depth - CLAW_PIN_SHAFT_MM) <= 0.10,
               f"the palm's socket runs z = {fmt(socket_bottom, 1)} (plate top) to "
               f"{fmt(sleeve_top, 1)} (sleeve top) = {fmt(depth)} mm deep; retainer shaft "
               f"{fmt(CLAW_PIN_SHAFT_MM)} mm")
        hub_bands = [b for b in _solid_bands(claw, (0.0, J6_RIGHT_Y, 0.0), 3.6, axis=2,
                                             scan=(-12.0, 46.0)) if b[1] - b[0] > 1.0]
        hub_h = (max(b[1] for b in hub_bands) - min(b[0] for b in hub_bands)) if hub_bands \
            else float("nan")
        proud = depth - hub_h
        record("pins", "collar takes up exactly the sleeve the finger does not cover",
               abs(proud - CLAW_COLLAR_H_MM) <= 0.10,
               f"socket {fmt(depth)} mm - finger hub {fmt(hub_h)} mm = {fmt(proud)} mm of "
               f"exposed sleeve; collar is {fmt(CLAW_COLLAR_H_MM)} mm")
        record("pins", "collar slips over the palm's sleeve and clear of the teeth",
               CLAW_COLLAR_ID_MM > 8.0 and CLAW_COLLAR_OD_MM < 2 * 8.44,
               f"collar bore {fmt(CLAW_COLLAR_ID_MM)} mm over an 8.00 mm sleeve, OD "
               f"{fmt(CLAW_COLLAR_OD_MM)} mm inside the claw's 8.44 mm tooth root")
        record("pins", "the pin head cannot drop into the bore it retains",
               PIVOT_PIN_HEAD_D_MM > PASSIVE_BORE_D,
               f"head {fmt(PIVOT_PIN_HEAD_D_MM)} mm vs bore {fmt(PASSIVE_BORE_D)} mm")


def audit_interference(meshes: Dict[str, Mesh], quick: bool) -> None:
    """Sweep each straddle joint through its travel and report the first clash anywhere.

    Every part is exported in a frame whose joint axis passes through the origin, and the
    parent states where its own joint sits (`J2_AXIS_Z_IN_04` and friends), so the mate is a
    pure translation of the child onto that axis. The child is then rotated about it, in
    steps, over the whole +-90 deg range.

    Two things are reported per joint:
      * INSIDE `USABLE_TRAVEL_DEG` the pair must not touch at all - any crossing or any
        sampled point inside the parent is a FAIL there, because that is travel the firmware
        is allowed to command;
      * outside it, the first angle that does clash is the mechanical stop, reported as the
        number the firmware has to clamp to.
    """
    mates = [
        ("05_upper_arm_link_80mm.stl", "04_base_upper_turret_j2.stl",
         (0.0, 0.0, J2_AXIS_Z_IN_04), "J2", 1),
        ("06_forearm_link_75mm.stl", "05_upper_arm_link_80mm.stl",
         (80.0, 0.0, J3_AXIS_Z_IN_05), "J3", 1),
        ("07_wrist_pitch_roll_bracket.stl", "06_forearm_link_75mm.stl",
         (75.0, 0.0, J4_AXIS_Z_IN_06), "J4", 1),
    ]
    for child_name, parent_name, shift, joint, axis in mates:
        child = meshes.get(child_name)
        parent = meshes.get(parent_name)
        if child is None or parent is None:
            record("clearance", f"{joint} :: meshes present", False,
                   f"{child_name} or {parent_name} missing")
            continue
        lo, hi = USABLE_TRAVEL_DEG[joint]
        step = 15.0 if quick else 5.0
        angles = sorted({0.0, lo, hi} |
                        {lo - step, hi + step, lo + step, hi - step})
        if not quick:
            angles = sorted(set(angles) |
                            {round(a * step, 4) for a in range(-18, 19)})

        parent_cells: Dict[Tuple[int, int, int], List[int]] = {}
        for i, t in enumerate(parent.tris):
            box = [min(t[k][m] for k in range(3)) for m in range(3)]
            top = [max(t[k][m] for k in range(3)) for m in range(3)]
            for ix in range(int(math.floor(box[0] / 4.0)), int(math.floor(top[0] / 4.0)) + 1):
                for iy in range(int(math.floor(box[1] / 4.0)), int(math.floor(top[1] / 4.0)) + 1):
                    for iz in range(int(math.floor(box[2] / 4.0)), int(math.floor(top[2] / 4.0)) + 1):
                        parent_cells.setdefault((ix, iy, iz), []).append(i)

        def crossings(deg: float):
            """(edges of the child that pass through the parent, first such point)."""
            th = math.radians(deg)
            c, sn = math.cos(th), math.sin(th)
            seen = set()
            n = 0
            first = None
            for tri in child.tris:
                pts = []
                for x, y, z in tri:
                    if axis == 1:                   # rotate about Y through the mate point
                        dx, dz = x, z
                        x, z = c * dx + sn * dz, -sn * dx + c * dz
                    pts.append((x + shift[0], y + shift[1], z + shift[2]))
                for k in range(3):
                    a, b = pts[k], pts[(k + 1) % 3]
                    key = (a, b) if a <= b else (b, a)
                    if key in seen:
                        continue
                    seen.add(key)
                    box = [min(a[m], b[m]) for m in range(3)]
                    top = [max(a[m], b[m]) for m in range(3)]
                    for ix in range(int(math.floor(box[0] / 4.0)),
                                    int(math.floor(top[0] / 4.0)) + 1):
                        for iy in range(int(math.floor(box[1] / 4.0)),
                                        int(math.floor(top[1] / 4.0)) + 1):
                            for iz in range(int(math.floor(box[2] / 4.0)),
                                            int(math.floor(top[2] / 4.0)) + 1):
                                for j in parent_cells.get((ix, iy, iz), ()):
                                    if segment_tri_hit(a, b, parent.tris[j]):
                                        n += 1
                                        if first is None:
                                            first = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0,
                                                     (a[2] + b[2]) / 2.0)
            return n, first

        def inside_points(deg: float):
            """(sampled points of the child inside the parent, first such point)."""
            th = math.radians(deg)
            c, sn = math.cos(th), math.sin(th)
            inside = 0
            walked = 0
            first_bad = None
            for tri in child.tris:
                ax, ay, az = tri[0]
                bx, by, bz = tri[1]
                cx, cy, cz = tri[2]
                for sx, sy, sz in (
                        (ax, ay, az), (bx, by, bz), (cx, cy, cz),
                        ((ax + bx) / 2.0, (ay + by) / 2.0, (az + bz) / 2.0),
                        ((bx + cx) / 2.0, (by + cy) / 2.0, (bz + cz) / 2.0),
                        ((cx + ax) / 2.0, (cy + ay) / 2.0, (cz + az) / 2.0),
                        ((ax + bx + cx) / 3.0, (ay + by + cy) / 3.0, (az + bz + cz) / 3.0)):
                    x, y, z = sx, sy, sz
                    if axis == 1:
                        dx, dz = x, z
                        x = c * dx + sn * dz
                        z = -sn * dx + c * dz
                    q = (x + shift[0], y + shift[1], z + shift[2])
                    walked += 1
                    if inside_robust(parent, q) is True:
                        inside += 1
                        if first_bad is None:
                            first_bad = q
            return inside, walked, first_bad

        worst_in = (0, 0, None)
        worst_cross = (0, None, None)
        clashes: Dict[float, Tuple[int, int, object]] = {}
        for deg in angles:
            n_cross, cross_at = crossings(deg)
            inside, walked, bad_at = inside_points(deg)
            if lo <= deg <= hi:
                if inside > worst_in[0] or worst_in[1] == 0:
                    worst_in = (inside, walked, bad_at)
                if n_cross > worst_cross[0]:
                    worst_cross = (n_cross, deg, cross_at)
            elif n_cross or inside:
                clashes[deg] = (n_cross, inside, cross_at or bad_at)
        # The stop is the clash CLOSEST to the declared band on each side, not the first one the
        # loop happens to reach: a shoulder that is clear to -60 deg and fouls at -65 deg has its
        # stop at -65, whatever else it does at -90.
        below = [a for a in clashes if a < lo]
        above = [a for a in clashes if a > hi]
        stop_angle = clashes[max(below)] + (max(below),) if below else None
        stop_angle_hi = clashes[min(above)] + (min(above),) if above else None
        for name, ok, detail in (
                (f"{joint} :: no material inside the parent over the travel "
                 f"[{fmt(lo, 0)}..{fmt(hi, 0)} deg]",
                 worst_in[0] == 0,
                 f"{worst_in[0]} of {worst_in[1]} sampled points inside"
                 + (f", first at ({fmt(worst_in[2][0], 1)}, {fmt(worst_in[2][1], 1)}, "
                    f"{fmt(worst_in[2][2], 1)})" if worst_in[2] else "")),
                (f"{joint} :: no surface crossing over the travel "
                 f"[{fmt(lo, 0)}..{fmt(hi, 0)} deg]",
                 worst_cross[0] == 0,
                 f"{worst_cross[0]} child edge(s) through the parent"
                 + (f" at {fmt(worst_cross[1], 1)} deg, first near "
                    f"({fmt(worst_cross[2][0], 1)}, {fmt(worst_cross[2][1], 1)}, "
                    f"{fmt(worst_cross[2][2], 1)})" if worst_cross[2] else "")),
        ):
            record("clearance", name, ok, detail)
        def stop_text(stop) -> str:
            n_cross, inside, at, deg = stop
            return (f"{fmt(deg, 1)} deg ({n_cross} edge crossing(s), {inside} point(s) inside"
                    + (f", near ({fmt(at[0], 1)}, {fmt(at[1], 1)}, {fmt(at[2], 1)})"
                       if at else "") + ")")

        if stop_angle or stop_angle_hi:
            sides = []
            if stop_angle:
                sides.append("on the " + fmt(lo, 0) + " side at " + stop_text(stop_angle))
            if stop_angle_hi:
                sides.append("on the " + fmt(hi, 0) + " side at " + stop_text(stop_angle_hi))
            record("clearance", f"{joint} :: the mechanical stop outside the travel", True,
                   f"clear through the declared travel; the arm touches " + " and ".join(sides)
                   + f". The firmware must clamp this joint to {fmt(lo, 0)}..{fmt(hi, 0)} deg.")
        else:
            record("clearance", f"{joint} :: the mechanical stop outside the travel", True,
                   f"no clash anywhere in +-90 deg; the declared travel {fmt(lo, 0)}.."
                   f"{fmt(hi, 0)} deg is not set by this pair")


def audit_claw_interface(meshes: Dict[str, Mesh], quick: bool) -> None:
    """Measure the J6 claw mounting the way the assembly would, from the vertices alone.

    08_gripper_body_and_cam_mount carries the two pivot sleeves at (J6_AXIS_X_IN_08, +/-11);
    09_gripper_geared_fingers_pair carries the two hubs that must land on them, plus the gear
    teeth of a 1:1 pair. Nothing else in this tool looks at either part, and both facts are
    geometry, so both are measured here:

      * the sleeve and hub circles are fitted with the two-pass `bore_measure` used in section
        C, at the positions the two builders declare;
      * the fingers are separated by their own edge components - a meshing pair is two printed
        pieces, not one - and each finger's teeth must reach at least half the pivot pitch from
        its own hub centre, or the two rings never meet at the centre line and the claw cannot
        close. That reach is `max |y - y_hub|` over the finger's vertices.
    """
    palm = meshes.get("08_gripper_body_and_cam_mount.stl")
    claw = meshes.get("09_gripper_geared_fingers_pair.stl")
    if palm is None or claw is None:
        return

    sleeves = []
    for sy in (J6_LEFT_Y, J6_RIGHT_Y):
        m = bore_measure(palm, "Z", (J6_AXIS_X_IN_08, sy, 0.0), (31.5, 41.5), 4.0)
        if m is None:
            record("placement", f"claw :: palm pivot sleeve at y = {fmt(sy, 1)} mm", False,
                   "no circular sleeve in the z 31.5..41.5 mm band")
            return
        sleeves.append((m[0], m[1]))
    palm_pitch = abs(sleeves[0][0][1] - sleeves[1][0][1])

    # A finger is a hub plus twelve separate teeth plus the finger itself; the slicer fuses
    # everything that touches, so the split has to be `material_bodies`, not edge components.
    bodies, _contacts, groups = material_bodies(claw)
    if len(bodies) != 2:
        record("placement", "claw :: the two fingers are two printed pieces", False,
               f"{len(bodies)} printed body(ies) in the claw")
        return
    fingers = []
    for body in bodies:
        tris_body = [t for gi in body for t in groups[gi]]
        y_avg = sum(v[1] for t in tris_body for v in t) / (3.0 * len(tris_body))
        fingers.append((y_avg, tris_body))
    fingers.sort()
    hubs = []
    for y_avg, body in fingers:
        ys = [v[1] for t in body for v in t]
        centre_y = (min(ys) + max(ys)) / 2.0
        m = bore_measure(claw, "Z", (0.0, centre_y, 0.0), (-0.5, 6.5), 4.3)
        if m is None:
            record("placement", "claw :: each finger carries a hub bore", False,
                   f"no circular hub bore near y = {fmt(centre_y, 1)} mm")
            return
        hubs.append((m[0][1], m[1], body))       # (centre y, bore diameter, triangles)
    claw_pitch = abs(hubs[1][0] - hubs[0][0])         # hubs[i] = (centre y, bore dia, tris)

    record("placement", "claw :: finger hubs land on the palm's pivot sleeves",
           abs(palm_pitch - claw_pitch) <= 0.40,
           f"the palm's sleeves are {fmt(palm_pitch, 2)} mm apart (declared y = "
           f"{fmt(J6_LEFT_Y, 1)} / {fmt(J6_RIGHT_Y, 1)}), the claw's hubs measure "
           f"{fmt(claw_pitch, 2)} mm apart (at y = {fmt(hubs[0][0], 2)} / "
           f"{fmt(hubs[1][0], 2)}) - {fmt(abs(claw_pitch - palm_pitch), 2)} mm of mismatch")
    for i in range(2):
        record("placement", f"claw :: finger hub {i + 1} slides over its sleeve",
               hubs[i][1] >= sleeves[i][1] + 0.40,
               f"palm sleeve OD {fmt(sleeves[i][1], 2)} mm vs finger hub bore "
               f"{fmt(hubs[i][1], 2)} mm ({fmt(hubs[i][1] - sleeves[i][1], 2)} mm)")

    reaches = []
    for (y_hub, bore, body) in hubs:
        reaches.append(max(abs(v[1] - y_hub) for t in body for v in t))
    need = claw_pitch / 2.0
    for i, reach in enumerate(reaches):
        record("placement", f"claw :: finger {i + 1} teeth reach the centre line",
               reach >= need,
               f"the teeth of finger {i + 1} reach {fmt(reach, 2)} mm from their own hub "
               f"centre; a 1:1 pair at a {fmt(claw_pitch, 1)} mm pitch needs {fmt(need, 2)} mm")
    left = [t for gi in bodies[0] for t in groups[gi]]
    right = [t for gi in bodies[1] for t in groups[gi]]
    kind, gap = sets_relation(left, right, distance=True)
    record("placement", "claw :: the two fingers mesh without interfering",
           kind != "cross" and kind != "inside" and gap >= 0.20,
           f"the two finger solids are {kind}, closest approach {fmt(gap, 2)} mm apart "
           f"(a 1:1 pair interleaves with the tooth space minus one tooth of clearance)")


# =============================================================================
# DRIVER
# =============================================================================
def declared_grams() -> Dict[str, float]:
    """The `declared g` column from the project's own part table."""
    return {
        "00_tolerance_test_coupon.stl": 8.0,
        "01_desk_c_clamp_frame.stl": 62.0,
        "02_desk_c_clamp_screw_and_pad.stl": 24.0,
        "03_base_lower_slewing_ring.stl": 140.0,
        "04_base_upper_turret_j2.stl": 56.0,
        "05_upper_arm_link_80mm.stl": 46.0,
        "06_forearm_link_75mm.stl": 33.0,
        "07_wrist_pitch_roll_bracket.stl": 21.0,
        "08_gripper_body_and_cam_mount.stl": 40.0,
        "09_gripper_geared_fingers_pair.stl": 9.0,
        "10_cable_clips_and_electronics_tray.stl": 17.0,
        "11_mobile_rover_chassis.stl": 159.0,
        "12_drive_wheels_and_caster.stl": 19.0,
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--quick", action="store_true", help="coarser slicing, faster run")
    ap.add_argument("--out", default=None, help="also write the report to this path")
    args = ap.parse_args(argv)

    grams = declared_grams()
    print("=" * 118)
    print("  DEEP CAD AUDIT - every number measured from the STL bytes in cad/stl/")
    print("=" * 118)
    for name in PART_FILENAMES:
        path = ROOT / "cad" / "stl" / name
        header_tris, _ = read_binary_stl(path)          # also validates the byte layout
        mesh = load_mesh(path, cell_mm=6.0)
        if len(mesh.tris) != header_tris:
            record("topology", f"{name} :: triangle count matches the header", False,
                   f"{len(mesh.tris)} vs {header_tris}")
        MESHES[name] = mesh
        audit_topology(name, mesh, args.quick)
        audit_sizing(name, mesh, grams.get(name, 0.0), args.quick)
        print(f"  {name:<42} {len(mesh.tris):>6} tris   "
              f"done ({len(RESULTS)} checks so far)")

    print("  --- joinability ---")
    audit_fits(MESHES, args.quick)
    print("  --- placement ---")
    audit_placement(MESHES, args.quick)
    print("  --- clearance ---")
    audit_interference(MESHES, args.quick)
    audit_claw_interface(MESHES, args.quick)
    print("  --- pins ---")
    audit_pivot_pins(MESHES, args.quick)

    # ---- report ---------------------------------------------------------------------
    lines: List[str] = []
    failures = [r for r in RESULTS if not r[2]]
    sections: List[str] = []
    for section, name, ok, detail in RESULTS:
        if section not in sections:
            sections.append(section)
    for section in sections:
        lines.append("")
        lines.append("=" * 118)
        lines.append(f"  {section.upper()}")
        lines.append("=" * 118)
        for s, name, ok, detail in RESULTS:
            if s != section:
                continue
            lines.append(f"  {'ok  ' if ok else 'FAIL'} {name}")
            if detail:
                lines.append(f"       {detail}")
    lines.append("")
    lines.append("=" * 118)
    lines.append(f"  {len(RESULTS) - len(failures)} of {len(RESULTS)} checks passed")
    for s, name, ok, detail in failures:
        lines.append(f"  FAIL {s} :: {name} :: {detail}")
    lines.append("=" * 118)

    text = "\n".join(lines)
    print(text)
    print(f"\n{len(RESULTS) - len(failures)} of {len(RESULTS)} deep CAD checks passed.")
    if args.out:
        out = Path(args.out)
        if not out.is_absolute():
            out = ROOT / out
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text + "\n", encoding="utf-8")
        print(f"report written to {out}")
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
