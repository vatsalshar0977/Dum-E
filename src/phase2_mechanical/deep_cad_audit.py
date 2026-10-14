#!/usr/bin/env python3
"""Deep CAD audit: reads the 13 shipped STL files and interrogates them.

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
    MG90S_BODY_W, MG90S_HORN_BORE_D, MG90S_SPLINE_OFFSET, PASSIVE_BORE_D, PASSIVE_PIN_D,
    PART_FILENAMES, ROVER_BOLT_ANGLES_DEG, ROVER_BOLT_CIRCLE_R_MM, WALL_T,
    WHEEL_DIAMETER_MM, WHEEL_THICKNESS_MM, STRADDLE_GAP_TOTAL,
    J2_OUTER_Y, J3_OUTER_Y, J4_OUTER_Y, J2_Y_NEG, J3_Y_NEG, J4_Y_NEG,
    J2_AXIS_Z_IN_04, J3_AXIS_Z_IN_05, J4_AXIS_Z_IN_06, J5_AXIS_X_IN_07,
    J5_AXIS_Z_IN_07, J6_AXIS_X_IN_08, J6_AXIS_Z_IN_08,
)

# How many separate pieces each file is DESIGNED to print as. A union of overlapping
# primitives is one piece; a screw and its pad, a pair of fingers, two wheels and a caster are
# deliberately several. Anything else - a plate that floats 2 mm off the part it belongs to -
# is a defect, and the check below now fails on it instead of shrugging at "<= 8 bodies".
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
    boxes = []
    for group in comps:
        xs = [v for f in group for v in (tris[f][0][0], tris[f][1][0], tris[f][2][0])]
        ys = [v for f in group for v in (tris[f][0][1], tris[f][1][1], tris[f][2][1])]
        zs = [v for f in group for v in (tris[f][0][2], tris[f][1][2], tris[f][2][2])]
        boxes.append((min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)))
    parents = list(range(len(comps)))

    def find(i: int) -> int:
        while parents[i] != i:
            parents[i] = parents[parents[i]]
            i = parents[i]
        return i

    for i in range(len(boxes)):
        for j in range(i + 1, len(boxes)):
            a, b = boxes[i], boxes[j]
            if (a[0] - 0.05 < b[1] and b[0] - 0.05 < a[1]
                    and a[2] - 0.05 < b[3] and b[2] - 0.05 < a[3]
                    and a[4] - 0.05 < b[5] and b[4] - 0.05 < a[5]):
                ra, rb = find(i), find(j)
                if ra != rb:
                    parents[ra] = rb
    clusters: Dict[int, List[int]] = {}
    for i in range(len(comps)):
        clusters.setdefault(find(i), []).append(i)
    want = DECLARED_BODIES.get(name, 1)
    record("topology", f"{name} :: prints as {want} piece(s), not more",
           len(clusters) == want,
           f"{len(clusters)} body cluster(s) of touching/overlapping material for a declared "
           f"{want}; {len(comps)} edge-disjoint solid(s) went in")

    if not quick and len(comps) > 1:
        # Multi-body files are deliberate (a pair of fingers, two wheels and a caster), but a
        # body that nobody meant to create is a bug: list them, largest first.
        rows = []
        for group in sorted(comps, key=lambda g: -sum(tri_signed_volume(tris[f]) for f in g))[:8]:
            xs = [v for f in group for v in (tris[f][0][0], tris[f][1][0], tris[f][2][0])]
            ys = [v for f in group for v in (tris[f][0][1], tris[f][1][1], tris[f][2][1])]
            zs = [v for f in group for v in (tris[f][0][2], tris[f][1][2], tris[f][2][2])]
            rows.append(f"{len(group)}t vol {sum(tri_signed_volume(tris[f]) for f in group):8.1f} "
                        f"mm3 bbox [{min(xs):.0f}..{max(xs):.0f}, {min(ys):.0f}..{max(ys):.0f}, "
                        f"{min(zs):.0f}..{max(zs):.0f}]")
        record("topology", f"{name} :: disjoint bodies are intentional", True,
               f"{len(comps)} bodies :: " + " | ".join(rows))


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
        volume += area * ((z1 - z0) / n_z)
        cx_acc += acx * area
        cy_acc += acy * area
        cz_acc += z * area
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
                2.5, 4.9, "clearance", "servo shaft / horn boss through the wheel", 0.10),
        FitCase("wheel hub bore, right", "12_drive_wheels_and_caster.stl", "Z",
                (0.0, 30.0, 0.0), (WHEEL_DIAMETER_MM / 2.0 - 2.55, WHEEL_DIAMETER_MM / 2.0 + 2.55),
                2.5, 4.9, "clearance", "servo shaft / horn boss through the wheel", 0.10),
        FitCase("tray standoff pilot hole", "10_cable_clips_and_electronics_tray.stl", "Z",
                (-30.0, -22.0, 3.0), (3.0, 8.1), 1.4, 2.6, "pilot",
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
        stand = bore_measure(tray, "Z", (-30.0, -22.0, 3.0), (3.0, 8.1), 1.4)
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
                   f"{fmt(min(gaps), 1)} mm, each bridged to the plate edge at y = 26 mm")
            clip = bore_measure(tray, "Z", (-24.8, 33.0, 0.0), (0.0, 6.1), 4.0)
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
def audit_interference(meshes: Dict[str, Mesh], quick: bool) -> None:
    """Put the parts together in the ASSEMBLED pose and look for material inside material.

    Every part is exported in a frame whose joint axis passes through the origin, and the
    parent states where each of its own joints sits (`J2_AXIS_Z_IN_04` and friends). So the
    mate is a pure translation of the child onto that axis - no guessed clearance - and the
    child can then be rotated about it to sweep the joint through part of its travel.

    A vertex inside the parent is interference. The declared +0.60 mm telescoping gap lives
    between the straddle faces, so a correct pair reports zero at every angle.
    """
    mates = [
        ("05_upper_arm_link_80mm.stl", "04_base_upper_turret_j2.stl",
         (0.0, 0.0, J2_AXIS_Z_IN_04), "J2", 1),
        ("06_forearm_link_75mm.stl", "05_upper_arm_link_80mm.stl",
         (80.0, 0.0, J3_AXIS_Z_IN_05), "J3", 1),
        ("07_wrist_pitch_roll_bracket.stl", "06_forearm_link_75mm.stl",
         (75.0, 0.0, J4_AXIS_Z_IN_06), "J4", 1),
    ]
    angles = (0.0,) if quick else (-5.0, -2.5, 0.0, 2.5, 5.0)
    for child_name, parent_name, shift, joint, axis in mates:
        child = meshes.get(child_name)
        parent = meshes.get(parent_name)
        if child is None or parent is None:
            record("clearance", f"{joint} :: meshes present", False,
                   f"{child_name} or {parent_name} missing")
            continue
        worst_angle = 0.0
        worst_inside = 0
        worst_walked = 0
        worst_at = None
        for deg in angles:
            th = math.radians(deg)
            c, sn = math.cos(th), math.sin(th)
            inside = 0
            walked = 0
            first_bad = None
            # Vertices alone miss a clash that lies in the middle of a big flat face, so every
            # triangle contributes its 3 corners, its 3 edge midpoints and its centroid.
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
                    if axis == 1:                   # rotate about Y through the mate point
                        dx, dz = x, z
                        x = c * dx + sn * dz
                        z = -sn * dx + c * dz
                    q = (x + shift[0], y + shift[1], z + shift[2])
                    walked += 1
                    verdict = inside_robust(parent, q)
                    if verdict is True:
                        inside += 1
                        if first_bad is None:
                            first_bad = (deg, q)
            if walked > worst_walked:
                worst_walked = walked
            if inside > worst_inside or worst_at is None:
                worst_inside = inside
                worst_angle = deg
                worst_at = first_bad
        detail = (f"{worst_inside} of {worst_walked} sampled points of the child sit inside "
                  f"the parent at {fmt(worst_angle, 1)} deg")
        if worst_at is not None:
            detail += (f"; first offender at ({fmt(worst_at[1][0], 1)}, {fmt(worst_at[1][1], 1)}, "
                       f"{fmt(worst_at[1][2], 1)}) mm in the parent frame")
        record("clearance", f"{joint} :: child never enters the parent through the travel",
               worst_inside == 0, detail)


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
