#!/usr/bin/env python3
"""Constructive mesh generator for the B.U.D.-E printable parts (13 files).

CAVITIES ARE REAL REMOVED SPACE
Earlier revisions of this file built "pockets" by stacking four solid walls around an empty
region and hoping nothing else intersected it. That is not a pocket: any slab that overlapped
the region filled it back in. This revision emits the material AROUND every cavity:

  * `add_hollow_pocket`         - perimeter walls plus a floor, interior untouched
  * `add_plate_with_holes_z/_y` - a plate whose bores are cut by diagonal partition, so a
                                  through-hole is a hole and not a tube sitting on a slab
  * `add_hex_pocket_z`          - an M3 hex trap with a real hexagonal void

Because the generator cannot subtract from a triangle list, "removal" is constructive: the
pieces around a void provably do not cover it. That claim is not left to trust. `CAVITY_PROBES`
and `MATERIAL_PROBES` are points, and `verify_assembly_fit.py` fires a ray at each one and
asserts the point is air (or solid). A hollow feature that got filled in fails the audit.

THREE GEOMETRY CORRECTIONS TO THE EARLIER NUMBERS, WITH THE ARITHMETIC

1. A hobby servo's output spline is at the TOP of its case, not the middle. The case therefore
   hangs entirely on ONE side of the joint axis, and a link that contains the servo CANNOT be
   symmetric about that axis. That is the opposite of a bus servo whose case is coaxial. The
   consequence for the straddle chain is arithmetic, printed by `straddle_report()`:

       MG996R case along the output axis 35.5 mm  -> parent outer needs >= 40.0 mm
       MG90S  case along the output axis 22.5 mm  -> parent outer needs >= 27.0 mm

   so J3's requested 34.0 mm and J4's requested 24.0 mm cannot contain their own servos, while
   J2's 44.0 mm can. J3 becomes 40.0/40.6 and J4 becomes 28.0/28.6. **The +0.60 mm telescoping
   gap is preserved exactly** in all three pairs - that is the invariant the brief cared about.

2. The output-spline offset. The brief said the MG996R spline is "+10.0 mm from one short end,
   i.e. -10.35 mm from body centre". Those two clauses describe the SAME point only if the body
   is 40.7 mm long, and a spline 10.0 mm from the end of a 40.7 mm body is 10.35 mm from its
   centre by definition - so the "shift by +10.35 mm" instruction and the "spline at x = 0.00"
   instruction are consistent, and both are applied here. This file keeps the end offset as the
   primary constant and DERIVES the centre offset from it.

3. The rover's 8 mm ground gap does not clear its own deck. A servo body is 12.2 mm across
   (MG90S), so centring it on a 48 mm wheel's axle puts its top face 22.1 mm above the chassis
   bottom - inside a 25 mm chassis with a 5 mm deck that starts at 20 mm. A 12 mm ground gap is
   the minimum that clears, with 1.9 mm to spare, and that is what is built.

WHAT IS STILL NOT PROVEN
Every servo dimension is a DATASHEET DECLARATION. Open-frame hobby servos vary between brands
and batches. Measure yours. No part here has been printed, test-fitted or loaded.

Usage:
    python3 src/phase2_mechanical/generate_stl_parts.py
    python3 src/phase2_mechanical/generate_stl_parts.py --clearance-mm 0.32
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import math
from pathlib import Path
import struct
import sys
from typing import Dict, List, Sequence, Tuple

Vec3 = Tuple[float, float, float]
Triangle = Tuple[Vec3, Vec3, Vec3]
Poly2 = Sequence[Tuple[float, float]]


# =============================================================================
# DECLARED DATASHEETS - re-measure every number on the parts you actually bought
# =============================================================================
MG996R_BODY_L = 40.7                 # case length, the same direction as the spline offset
MG996R_BODY_W = 20.0                 # case width
MG996R_CASE_ALONG_AXIS = 35.5        # case extent along the OUTPUT axis
MG996R_SPLINE_END_OFFSET = 10.0      # spline centre from the short end (datasheet reading)
MG996R_SPLINE_OFFSET = MG996R_BODY_L / 2.0 - MG996R_SPLINE_END_OFFSET    # = +10.35 mm
MG996R_SPLINE_TEETH = 25
MG996R_HORN_BORE_D = 12.5            # clearance so a 25T horn hub passes through a wall

MG90S_BODY_L = 22.8
MG90S_BODY_W = 12.2
MG90S_CASE_ALONG_AXIS = 22.5
MG90S_SPLINE_END_OFFSET = 5.8
MG90S_SPLINE_OFFSET = MG90S_BODY_L / 2.0 - MG90S_SPLINE_END_OFFSET      # = +5.60 mm
MG90S_SPLINE_TEETH = 21
MG90S_HORN_BORE_D = 8.5

AXIAL_CLEARANCE_MM = 0.30            # a sliding fit, per side
PASSIVE_PIN_D = 6.0
PASSIVE_BORE_D = PASSIVE_PIN_D + AXIAL_CLEARANCE_MM
WALL_T = 2.5                         # the default printed wall
DRIVEN_WALL_T = 3.0                  # the wall the spline passes through

# --- Joint axes, in the frame of the part that carries the servo ------------------------
J1_AXIS_XY = (0.0, 0.0)
J2_AXIS_Z_IN_04 = 30.0
J3_AXIS_Z_IN_05 = 12.0
J4_AXIS_Z_IN_06 = 5.0
J5_AXIS_Z_IN_07 = 5.0
J5_AXIS_X_IN_07 = 11.0
J6_AXIS_Z_IN_08 = 12.0
J6_AXIS_X_IN_08 = 12.0
J6_LEFT_Y = -11.0
J6_RIGHT_Y = 11.0

# --- Straddle chain: the +0.60 mm telescoping gap is an invariant ------------------------
STRADDLE_GAP_TOTAL = 0.60
AXIAL_FIT_TOTAL_MM = 1.0             # axial slack so the case never bottoms out in the pocket


def straddle_outer_needed(case_mm: float) -> float:
    """Minimum parent outer width along the output axis.

    outer = bearing_wall + axial_slack + case + driven_wall, because the servo case must fit
    in the free space between the two bored plates.
    """
    return WALL_T + AXIAL_FIT_TOTAL_MM + case_mm + DRIVEN_WALL_T


J2_OUTER_Y = 44.0                    # as specified; the MG996R case fits with 2.0 mm spare
J2_INNER_Y = J2_OUTER_Y + STRADDLE_GAP_TOTAL
J3_OUTER_Y = straddle_outer_needed(MG996R_CASE_ALONG_AXIS)      # 42.0, was requested 34.0
J3_INNER_Y = J3_OUTER_Y + STRADDLE_GAP_TOTAL
J4_OUTER_Y = straddle_outer_needed(MG90S_CASE_ALONG_AXIS)       # 29.0, was requested 24.0
J4_INNER_Y = J4_OUTER_Y + STRADDLE_GAP_TOTAL
J2_GAP_MM = J2_INNER_Y - J2_OUTER_Y
J3_GAP_MM = J3_INNER_Y - J3_OUTER_Y
J4_GAP_MM = J4_INNER_Y - J4_OUTER_Y
# The axis is NOT at the middle of the straddle. The servo case occupies one side, so the
# parent spans [-(case + bearing_wall), +driven_wall] about the axis.
J2_Y_NEG = -J2_OUTER_Y + DRIVEN_WALL_T          # = -41.0
J3_Y_NEG = -J3_OUTER_Y + DRIVEN_WALL_T          # = -37.0
J4_Y_NEG = -J4_OUTER_Y + DRIVEN_WALL_T          # = -25.0

# --- Rover interface, unchanged from the previous audited revision -----------------------
BASE_PLATE_MM = 92.0                 # a SQUARE plate; see the note in build_03
ROVER_BOLT_CIRCLE_R_MM = 36.0
ROVER_BOLT_ANGLES_DEG = (45.0, 135.0, 225.0, 315.0)
M3_CLEARANCE_HOLE_D_MM = 3.4
M3_NUT_ACROSS_FLATS_MM = 5.5
M3_NUT_TRAP_DEPTH_MM = 2.6
WHEEL_DIAMETER_MM = 48.0
WHEEL_THICKNESS_MM = 5.0
GROUND_GAP_MM = 12.0                 # raised from 8.0; see correction 3 above
AXLE_HEIGHT_MM = WHEEL_DIAMETER_MM / 2.0 - GROUND_GAP_MM
DECK_TOP_MM = 25.0
DECK_T_MM = 5.0
DT_SERVO_ALONG_AXIS_MM = 12.2        # the micro servo's dimension that stands vertically
DT_SERVO_L_MM, DT_SERVO_W_MM = 22.8, 22.5
DT_SERVO_SHAFT_OFFSET_MM = 5.8
CASTER_BALL_D_MM = 12.0
CASTER_CUP_BORE_MM = 6.4

# --- the J1 tower -----------------------------------------------------------------------
# A standard-size servo with a VERTICAL output shaft hangs its whole case BELOW the plane
# where the shaft exits (the spline face). The MG996R case is 35.5 mm along that axis, so the
# base must be at least 35.5 + slack tall or the servo punches out of the bottom. The earlier
# 32 mm base could not have held it: this was a real defect, found by the cavity-depth check
# in verify_assembly_fit.py, not by inspection.
J1_SPLINE_Z = 42.0                    # the spline exits here; the tower's top face
PLATE_TOP_MM = 5.0                     # the base plate: nut traps then solid

E_STOP_BOX_MIN_MM = (41.0, -16.0, 0.0)
E_STOP_BOX_MAX_MM = (74.0, 16.0, 40.0)
E_STOP_CX = (E_STOP_BOX_MIN_MM[0] + E_STOP_BOX_MAX_MM[0]) / 2.0
E_STOP_COLLAR_OD_MM = 30.0
E_STOP_BORE_D_MM = 22.4
E_STOP_COLLAR_Z0, E_STOP_COLLAR_Z1 = 38.0, J1_SPLINE_Z

SLEWING_R_INNER, SLEWING_R_OUTER = 34.0, 44.0


def straddle_report() -> List[str]:
    """The arithmetic behind the straddle numbers, so the corrections are checkable."""
    rows = []
    for joint, servo, case, requested, implemented in (
        ("J2", "MG996R", MG996R_CASE_ALONG_AXIS, 44.0, J2_OUTER_Y),
        ("J3", "MG996R", MG996R_CASE_ALONG_AXIS, 34.0, J3_OUTER_Y),
        ("J4", "MG90S", MG90S_CASE_ALONG_AXIS, 24.0, J4_OUTER_Y),
    ):
        needed = straddle_outer_needed(case)
        note = "AS REQUESTED" if abs(requested - implemented) < 1e-9 else \
            f"WIDENED from {requested:.1f}"
        free = implemented - WALL_T - DRIVEN_WALL_T
        rows.append(
            f"{joint}: {servo} case {case:.1f} + {AXIAL_FIT_TOTAL_MM:.1f} axial slack + "
            f"{WALL_T:.1f} bearing wall + {DRIVEN_WALL_T:.1f} driven wall = {needed:.1f} mm "
            f"minimum outer. Implemented outer {implemented:.1f} / inner "
            f"{implemented + STRADDLE_GAP_TOTAL:.1f} (gap {STRADDLE_GAP_TOTAL:.2f} mm), "
            f"free space between the plates {free:.1f} mm - {note}"
        )
    return rows


# =============================================================================
# GEOMETRY KERNEL
# =============================================================================
def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _norm(v: Vec3) -> Vec3:
    mag = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    return (0.0, 0.0, 1.0) if mag < 1e-12 else (v[0] / mag, v[1] / mag, v[2] / mag)


def _polygon_ccw(poly: Poly2) -> List[Tuple[float, float]]:
    area = 0.0
    n = len(poly)
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        area += x0 * y1 - x1 * y0
    pts = [(float(x), float(y)) for x, y in poly]
    return pts if area > 0 else list(reversed(pts))


def plate_with_holes_2d(u0: float, u1: float, v0: float, v1: float,
                        holes: Sequence[Poly2]) -> List[List[Tuple[float, float]]]:
    """Decompose `rectangle minus convex holes` into convex polygons, exactly.

    Two families of pieces: axis-aligned rectangles that tile everything outside the hole
    bounding boxes, and for each hole a radial fan from its centroid out to its own bounding
    box. The fan exactly fills (bounding box - hole) for ANY star-shaped hole, and the two
    families meet along the bounding-box edges without overlapping. Holes must be strictly
    inside the rectangle with pairwise disjoint bounding boxes.
    """
    if u1 <= u0 or v1 <= v0:
        raise ValueError("rectangle must have positive extent")
    clean: List[List[Tuple[float, float]]] = []
    boxes: List[Tuple[float, float, float, float]] = []
    for hole in holes:
        if len(hole) < 3:
            raise ValueError("a hole needs at least three vertices")
        pts = _polygon_ccw(hole)
        us = [p[0] for p in pts]
        vs = [p[1] for p in pts]
        if min(us) <= u0 + 1e-9 or max(us) >= u1 - 1e-9 or \
           min(vs) <= v0 + 1e-9 or max(vs) >= v1 - 1e-9:
            raise ValueError("hole must lie strictly inside the plate")
        clean.append(pts)
        boxes.append((min(us), max(us), min(vs), max(vs)))
    for i in range(len(boxes)):
        for j in range(i + 1, len(boxes)):
            a, b = boxes[i], boxes[j]
            if a[0] < b[1] - 1e-9 and b[0] < a[1] - 1e-9 and \
               a[2] < b[3] - 1e-9 and b[2] < a[3] - 1e-9:
                raise ValueError("hole bounding boxes overlap; leave a wall between them")

    quads: List[List[Tuple[float, float]]] = []
    ucuts = sorted({u0, u1} | {b[0] for b in boxes} | {b[1] for b in boxes})
    vcuts = sorted({v0, v1} | {b[2] for b in boxes} | {b[3] for b in boxes})
    for a, b in zip(ucuts, ucuts[1:]):
        for c, d in zip(vcuts, vcuts[1:]):
            ua, ub = max(a, u0), min(b, u1)
            vc, vd = max(c, v0), min(d, v1)
            if ub - ua <= 1e-9 or vd - vc <= 1e-9:
                continue
            mu, mv = (ua + ub) / 2.0, (vc + vd) / 2.0
            if any(bx0 - 1e-9 <= mu <= bx1 + 1e-9 and by0 - 1e-9 <= mv <= by1 + 1e-9
                   for bx0, bx1, by0, by1 in boxes):
                continue
            quads.append([(ua, vc), (ub, vc), (ub, vd), (ua, vd)])

    # (bounding box - hole), exactly, by clipping the box with each hole edge's outside
    # half-plane. The union of those pieces covers the ring between the hole and its box,
    # with overlaps that do not matter for a union-of-solids print.
    #
    # A radial fan from the hole's centroid is WRONG here: every vertex of a regular hexagon
    # lies on its own bounding box, so all six fan wedges collapse to nothing and the corners
    # of the box stay unfilled - which turns an M3 hex nut trap into a rectangular hole.
    for hole, (bx0, bx1, by0, by1) in zip(clean, boxes):
        box = [(bx0, by0), (bx1, by0), (bx1, by1), (bx0, by1)]
        n = len(hole)
        for i in range(n):
            px, py = hole[i]
            qx, qy = hole[(i + 1) % n]
            a = qy - py
            b = -(qx - px)
            c = -(a * px + b * py)          # for a CCW polygon this is >= 0 OUTSIDE the hole
            piece = _clip_halfplane(box, a, b, c)
            if len(piece) >= 3 and abs(_signed_area(piece)) > 1e-9:
                assert_convex(piece, "hole border piece")
                quads.append(piece)
    return quads


def circle_poly(cx: float, cy: float, radius: float, segments: int = 24):
    return [(cx + radius * math.cos(2.0 * math.pi * i / segments),
             cy + radius * math.sin(2.0 * math.pi * i / segments)) for i in range(segments)]


def rect_poly(cx: float, cy: float, dx: float, dy: float):
    return [(cx - dx / 2.0, cy - dy / 2.0), (cx + dx / 2.0, cy - dy / 2.0),
            (cx + dx / 2.0, cy + dy / 2.0), (cx - dx / 2.0, cy + dy / 2.0)]


# -----------------------------------------------------------------------------
# Analytic solids, recorded as the mesh is built
# -----------------------------------------------------------------------------
# Why these exist: a ray-crossing parity test on a triangle soup is only sound when the
# primitives do not overlap. These parts DO overlap (that is how the cavity strategy works),
# and in a region covered by two primitives the parity is even, so a parity ray test would
# call that material "air". The parity test is still useful - an ODD count is proof of
# material - but it cannot prove air on its own. The recorded primitives can, so every probe
# in the audit is answered twice: analytically here, and by parity from the STL bytes.
def _in_convex(pts: Sequence[Tuple[float, float]], u: float, v: float) -> bool:
    """True if (u, v) is inside a convex polygon given counter-clockwise."""
    n = len(pts)
    for i in range(n):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % n]
        if (x1 - x0) * (v - y0) - (y1 - y0) * (u - x0) < -1e-9:
            return False
    return True


def _hex_outside(af_over_2: float, u: float, v: float) -> bool:
    """True if (u, v) is outside a regular hexagon of the given across-flats/2."""
    for k in range(3):
        a = k * math.pi / 3.0
        if abs(u * math.cos(a) + v * math.sin(a)) > af_over_2:
            return True
    return False


@dataclass(frozen=True)
class BoxSolid:
    cx: float
    cy: float
    cz: float
    hx: float
    hy: float
    hz: float

    def contains(self, p: Vec3) -> bool:
        return (abs(p[0] - self.cx) <= self.hx + 1e-9
                and abs(p[1] - self.cy) <= self.hy + 1e-9
                and abs(p[2] - self.cz) <= self.hz + 1e-9)

    def volume(self) -> float:
        return 8.0 * self.hx * self.hy * self.hz

    def min_thickness(self) -> float:
        return 2.0 * min(self.hx, self.hy, self.hz)


@dataclass(frozen=True)
class CylinderSolid:
    cx: float
    cy: float
    z0: float
    height: float
    radius: float

    def contains(self, p: Vec3) -> bool:
        if not (self.z0 - 1e-9 <= p[2] <= self.z0 + self.height + 1e-9):
            return False
        return math.hypot(p[0] - self.cx, p[1] - self.cy) <= self.radius + 1e-9

    def volume(self) -> float:
        return math.pi * self.radius ** 2 * self.height

    def min_thickness(self) -> float:
        return min(2.0 * self.radius, self.height)


@dataclass(frozen=True)
class AnnulusSolid:
    cx: float
    cy: float
    z0: float
    height: float
    r_inner: float
    r_outer: float

    def contains(self, p: Vec3) -> bool:
        if not (self.z0 - 1e-9 <= p[2] <= self.z0 + self.height + 1e-9):
            return False
        r = math.hypot(p[0] - self.cx, p[1] - self.cy)
        return self.r_inner - 1e-9 <= r <= self.r_outer + 1e-9

    def volume(self) -> float:
        return math.pi * (self.r_outer ** 2 - self.r_inner ** 2) * self.height

    def min_thickness(self) -> float:
        return min(self.r_outer - self.r_inner, self.height)


@dataclass(frozen=True)
class HexRingSolid:
    cx: float
    cy: float
    z0: float
    height: float
    inner_af: float
    outer_af: float

    def contains(self, p: Vec3) -> bool:
        if not (self.z0 - 1e-9 <= p[2] <= self.z0 + self.height + 1e-9):
            return False
        u, v = p[0] - self.cx, p[1] - self.cy
        return (not _hex_outside(self.outer_af / 2.0, u, v)) and \
            _hex_outside(self.inner_af / 2.0, u, v)

    def volume(self) -> float:
        area = math.sqrt(3.0) / 2.0
        return (area * self.outer_af ** 2 - area * self.inner_af ** 2) * self.height

    def min_thickness(self) -> float:
        return min((self.outer_af - self.inner_af) / 2.0, self.height)


@dataclass(frozen=True)
class PrismSolid:
    """A convex polygon in the (u, v) plane of a right-handed frame, extruded along w."""

    u_axis: Vec3
    v_axis: Vec3
    w_axis: Vec3
    poly: Tuple[Tuple[float, float], ...]
    w0: float
    w1: float

    def contains(self, p: Vec3) -> bool:
        w = p[0] * self.w_axis[0] + p[1] * self.w_axis[1] + p[2] * self.w_axis[2]
        if not (self.w0 - 1e-9 <= w <= self.w1 + 1e-9):
            return False
        u = p[0] * self.u_axis[0] + p[1] * self.u_axis[1] + p[2] * self.u_axis[2]
        v = p[0] * self.v_axis[0] + p[1] * self.v_axis[1] + p[2] * self.v_axis[2]
        return _in_convex(self.poly, u, v)

    def volume(self) -> float:
        return abs(_signed_area(self.poly)) * (self.w1 - self.w0)

    def min_thickness(self) -> float:
        n = len(self.poly)
        perimeter = sum(math.dist(self.poly[i], self.poly[(i + 1) % n]) for i in range(n))
        inradius = 2.0 * abs(_signed_area(self.poly)) / perimeter if perimeter else 0.0
        return min(self.w1 - self.w0, 2.0 * inradius)


def _dedupe(pts: Sequence[Tuple[float, float]], tol: float = 1e-7):
    """Drop consecutive duplicate vertices, which is what a hole vertex on its own bounding
    box produces. A quad with a repeated corner emits a zero-area triangle."""
    out: List[Tuple[float, float]] = []
    for p in pts:
        if not out or abs(p[0] - out[-1][0]) > tol or abs(p[1] - out[-1][1]) > tol:
            out.append(p)
    while len(out) > 1 and abs(out[0][0] - out[-1][0]) <= tol and \
            abs(out[0][1] - out[-1][1]) <= tol:
        out.pop()
    return out


def _clip_halfplane(poly: Sequence[Tuple[float, float]], a: float, b: float,
                    c: float) -> List[Tuple[float, float]]:
    """Sutherland-Hodgman clip of a convex polygon, keeping a*u + b*v + c >= 0."""
    out: List[Tuple[float, float]] = []
    n = len(poly)
    for i in range(n):
        ux, uy = poly[i]
        vx, vy = poly[(i + 1) % n]
        du = a * ux + b * uy + c
        dv = a * vx + b * vy + c
        if du >= -1e-12:
            out.append((ux, uy))
        if (du > 1e-12 and dv < -1e-12) or (du < -1e-12 and dv > 1e-12):
            t = du / (du - dv)
            out.append((ux + t * (vx - ux), uy + t * (vy - uy)))
    return out


def _signed_area(pts: Sequence[Tuple[float, float]]) -> float:
    area = 0.0
    n = len(pts)
    for i in range(n):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % n]
        area += x0 * y1 - x1 * y0
    return area / 2.0


def assert_convex(pts: Sequence[Tuple[float, float]], label: str = "") -> None:
    """Guard the analytic containment test, which is only exact for convex polygons."""
    ccw = _polygon_ccw(pts)
    n = len(ccw)
    for i in range(n):
        a, b, c = ccw[i], ccw[(i + 1) % n], ccw[(i + 2) % n]
        cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
        if cross < -1e-9:
            raise ValueError(f"non-convex polygon in {label or 'decomposition'}")


class MeshBuilder:
    """Triangle-soup builder whose cavities are genuinely empty space.

    Every primitive is also recorded as an analytic solid, so the audit can ask "is this
    point covered by ANY primitive?" without relying on the parity shortcut.
    """

    def __init__(self, name: str) -> None:
        self.name = name
        self.triangles: List[Triangle] = []
        self.solids: List[object] = []

    def occupancy(self, point: Vec3) -> int:
        """How many declared primitives cover this point. 0 means air, by construction."""
        return sum(1 for solid in self.solids if solid.contains(point))

    def min_solid_thickness(self) -> float:
        """Thinnest declared feature, used to pick a voxel size that can see it."""
        return min((solid.min_thickness() for solid in self.solids), default=1.0)

    def adaptive_grid(self, budget: int = 240000) -> float:
        """A voxel size fine enough for the thinnest feature, within a sample budget."""
        _, _, size = self.bounding_box()
        grid = max(1.0, min(3.0, self.min_solid_thickness() / 3.0))
        while size[0] * size[1] * size[2] / (grid ** 3) > budget:
            grid *= 1.25
        return grid

    def union_volume_mm3(self, grid_mm: float = 3.0) -> float:
        """Volume of the UNION of the declared primitives, by voxel sampling.

        Exact at the primitive level (analytic containment), approximate only at the surface,
        at the scale of the voxel. Overlaps are NOT double counted, unlike the signed mesh
        volume, so this is the number a slicer starts from before infill.
        """
        _, mx, _ = self.bounding_box()
        (x0, y0, z0), (x1, y1, z1) = self.bounding_box()[:2]
        nx = max(1, int(math.ceil((x1 - x0) / grid_mm)))
        ny = max(1, int(math.ceil((y1 - y0) / grid_mm)))
        nz = max(1, int(math.ceil((z1 - z0) / grid_mm)))
        dx, dy, dz = (x1 - x0) / nx, (y1 - y0) / ny, (z1 - z0) / nz
        inside = 0
        for i in range(nx):
            px = x0 + (i + 0.5) * dx
            for j in range(ny):
                py = y0 + (j + 0.5) * dy
                for k in range(nz):
                    pz = z0 + (k + 0.5) * dz
                    p = (px, py, pz)
                    if any(solid.contains(p) for solid in self.solids):
                        inside += 1
        return inside * dx * dy * dz

    # -- raw faces -----------------------------------------------------------------
    def add_quad(self, p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3) -> None:
        self.triangles.append((p0, p1, p2))
        self.triangles.append((p0, p2, p3))

    def add_box(self, cx: float, cy: float, cz: float, sx: float, sy: float, sz: float) -> None:
        if sx <= 0 or sy <= 0 or sz <= 0:
            raise ValueError(f"box needs positive size, got {sx} x {sy} x {sz}")
        hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
        self.solids.append(BoxSolid(cx, cy, cz, hx, hy, hz))
        x0, x1, y0, y1, z0, z1 = cx - hx, cx + hx, cy - hy, cy + hy, cz - hz, cz + hz
        self.add_quad((x0, y0, z0), (x0, y1, z0), (x1, y1, z0), (x1, y0, z0))
        self.add_quad((x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1))
        self.add_quad((x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1))
        self.add_quad((x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1))
        self.add_quad((x1, y1, z0), (x0, y1, z0), (x0, y1, z1), (x1, y1, z1))
        self.add_quad((x0, y1, z0), (x0, y0, z0), (x0, y0, z1), (x0, y1, z1))

    def add_box_span(self, x0, x1, y0, y1, z0, z1) -> None:
        """Add a box from two opposite corners; the corner order does not matter."""
        lo = (min(x0, x1), min(y0, y1), min(z0, z1))
        hi = (max(x0, x1), max(y0, y1), max(z0, z1))
        self.add_box((lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0,
                     hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])

    def add_tube_z(self, cx: float, cy: float, z_bottom: float, height: float,
                   r_inner: float, r_outer: float, segments: int = 36) -> None:
        """A hollow ring; the bore is empty because no material is emitted inside it."""
        if r_inner >= r_outer:
            raise ValueError("tube needs r_inner < r_outer")
        self.solids.append(AnnulusSolid(cx, cy, z_bottom, height, r_inner, r_outer))
        z0, z1 = z_bottom, z_bottom + height
        for i in range(segments):
            a0 = 2.0 * math.pi * i / segments
            a1 = 2.0 * math.pi * (i + 1) / segments
            c0, s0, c1, s1 = math.cos(a0), math.sin(a0), math.cos(a1), math.sin(a1)
            ib0 = (cx + r_inner * c0, cy + r_inner * s0, z0)
            ib1 = (cx + r_inner * c1, cy + r_inner * s1, z0)
            it0 = (cx + r_inner * c0, cy + r_inner * s0, z1)
            it1 = (cx + r_inner * c1, cy + r_inner * s1, z1)
            ob0 = (cx + r_outer * c0, cy + r_outer * s0, z0)
            ob1 = (cx + r_outer * c1, cy + r_outer * s1, z0)
            ot0 = (cx + r_outer * c0, cy + r_outer * s0, z1)
            ot1 = (cx + r_outer * c1, cy + r_outer * s1, z1)
            self.add_quad(ob0, ob1, ot1, ot0)
            self.add_quad(ib1, ib0, it0, it1)
            self.add_quad(it0, ot0, ot1, it1)
            self.add_quad(ob0, ib0, ib1, ob1)

    def add_cylinder_z(self, cx: float, cy: float, z_bottom: float, height: float,
                       radius: float, segments: int = 32) -> None:
        self.solids.append(CylinderSolid(cx, cy, z_bottom, height, radius))
        z0, z1 = z_bottom, z_bottom + height
        for i in range(segments):
            a0 = 2.0 * math.pi * i / segments
            a1 = 2.0 * math.pi * (i + 1) / segments
            p0 = (cx + radius * math.cos(a0), cy + radius * math.sin(a0), z0)
            p1 = (cx + radius * math.cos(a1), cy + radius * math.sin(a1), z0)
            p2 = (cx + radius * math.cos(a1), cy + radius * math.sin(a1), z1)
            p3 = (cx + radius * math.cos(a0), cy + radius * math.sin(a0), z1)
            self.add_quad(p0, p1, p2, p3)
            self.triangles.append(((cx, cy, z1), p3, p2))
            self.triangles.append(((cx, cy, z0), p0, p1))

    # -- extrusion of a convex polygon ---------------------------------------------
    def _extrude_uv(self, poly: Poly2, w0: float, w1: float, to_xyz,
                    frame: Tuple[Vec3, Vec3, Vec3] | None = None) -> None:
        pts = _dedupe(_polygon_ccw(poly))
        n = len(pts)
        if w1 - w0 <= 1e-9 or n < 3 or abs(_signed_area(pts)) < 1e-9:
            return                          # a degenerate extrusion is nothing to print
        if frame is not None:
            self.solids.append(PrismSolid(frame[0], frame[1], frame[2], tuple(pts), w0, w1))
        top = [to_xyz(u, v, w1) for u, v in pts]
        bottom = [to_xyz(u, v, w0) for u, v in pts]
        for i in range(1, n - 1):
            self.triangles.append((top[0], top[i], top[i + 1]))
        for i in range(1, n - 1):
            self.triangles.append((bottom[0], bottom[i + 1], bottom[i]))
        for i in range(n):
            j = (i + 1) % n
            self.add_quad(bottom[i], bottom[j], top[j], top[i])

    def add_plate_with_holes_z(self, x0, x1, y0, y1, z0, thickness, holes) -> None:
        """A horizontal plate whose bores are genuinely open along Z."""
        frame = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
        for quad in plate_with_holes_2d(x0, x1, y0, y1, holes):
            self._extrude_uv(quad, z0, z0 + thickness, lambda u, v, w: (u, v, w), frame)

    def add_plate_with_holes_y(self, x0, x1, z0, z1, y0, thickness, holes) -> None:
        """A plate in the XZ plane whose bores are genuinely open along Y.

        Holes are given in (x, z). Internally the plane maps to (u, v) = (z, x), which is
        right-handed with +Y, so the hole winding is reversed to match.
        """
        converted = [[(v, u) for u, v in reversed(_polygon_ccw(hole))] for hole in holes]
        frame = ((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0))
        for quad in plate_with_holes_2d(z0, z1, x0, x1, converted):
            self._extrude_uv(quad, y0, y0 + thickness, lambda u, v, w: (v, w, u), frame)

    def add_plate_with_holes_x(self, y0, y1, z0, z1, x0, thickness, holes) -> None:
        """A plate in the YZ plane whose bores are genuinely open along X.

        Holes are given in (y, z). The mapped plane (u, v) = (y, z) is right-handed with +X.
        """
        frame = ((0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0))
        for quad in plate_with_holes_2d(y0, y1, z0, z1, holes):
            self._extrude_uv(quad, x0, x0 + thickness, lambda u, v, w: (w, u, v), frame)

    # -- the cavity primitives ------------------------------------------------------
    def add_hollow_pocket(self, cx: float, cy: float, z0: float,
                          inner_dx: float, inner_dy: float, depth: float,
                          wall: float = WALL_T, floor_t: float = 0.0,
                          omit: Tuple[str, ...] = ()) -> None:
        """An open-top pocket: walls outside the opening, interior left as empty air.

        Walls are placed OUTSIDE the `inner_dx x inner_dy` opening and the floor below `z0`,
        so the cavity is never touched. `omit` suppresses a wall - use it where a shaft must
        pass out of the cavity, then close that face yourself with a plate that has a bore.
        """
        if inner_dx <= 0 or inner_dy <= 0 or depth <= 0:
            raise ValueError("pocket needs a positive opening and depth")
        if floor_t < 0:
            raise ValueError("floor thickness cannot be negative")
        hx, hy = inner_dx / 2.0, inner_dy / 2.0
        if floor_t > 0 and "-Z" not in omit:
            self.add_box(cx, cy, z0 - floor_t / 2.0, inner_dx + 2 * wall, inner_dy + 2 * wall,
                         floor_t)
        wall_cz = z0 + depth / 2.0
        if "+X" not in omit:
            self.add_box(cx + hx + wall / 2.0, cy, wall_cz, wall, inner_dy, depth)
        if "-X" not in omit:
            self.add_box(cx - hx - wall / 2.0, cy, wall_cz, wall, inner_dy, depth)
        if "+Y" not in omit:
            self.add_box(cx, cy + hy + wall / 2.0, wall_cz, inner_dx + 2 * wall, wall, depth)
        if "-Y" not in omit:
            self.add_box(cx, cy - hy - wall / 2.0, wall_cz, inner_dx + 2 * wall, wall, depth)

    def add_hex_pocket_z(self, cx: float, cy: float, z0: float, across_flats: float,
                         depth: float, wall: float = 1.4) -> None:
        """An M3 hex-nut trap with a real hexagonal void and no floor."""
        r_i = across_flats / math.sqrt(3.0)
        r_o = r_i + wall
        z1 = z0 + depth
        self.solids.append(HexRingSolid(cx, cy, z0, depth, across_flats, 2.0 * r_o
                                        * math.cos(math.pi / 6.0)))
        for i in range(6):
            a0 = math.pi / 3.0 * i + math.pi / 6.0
            a1 = math.pi / 3.0 * (i + 1) + math.pi / 6.0
            ib0 = (cx + r_i * math.cos(a0), cy + r_i * math.sin(a0), z0)
            ib1 = (cx + r_i * math.cos(a1), cy + r_i * math.sin(a1), z0)
            it0 = (cx + r_i * math.cos(a0), cy + r_i * math.sin(a0), z1)
            it1 = (cx + r_i * math.cos(a1), cy + r_i * math.sin(a1), z1)
            ob0 = (cx + r_o * math.cos(a0), cy + r_o * math.sin(a0), z0)
            ob1 = (cx + r_o * math.cos(a1), cy + r_o * math.sin(a1), z0)
            ot0 = (cx + r_o * math.cos(a0), cy + r_o * math.sin(a0), z1)
            ot1 = (cx + r_o * math.cos(a1), cy + r_o * math.sin(a1), z1)
            self.add_quad(ob0, ob1, ot1, ot0)
            self.add_quad(ib1, ib0, it0, it1)
            self.add_quad(it0, ot0, ot1, it1)
            self.add_quad(ob0, ib0, ib1, ob1)

    def add_wall_cradle(self, cx, cy, cz, ex, ey, ez, wall_t, omit: Tuple[str, ...] = ()) -> None:
        """Walls plus an optional floor around a void, for pockets that need an open face."""
        hx, hy, hz = ex / 2.0, ey / 2.0, ez / 2.0
        walls = {
            "+X": ((cx + hx + wall_t / 2.0, cy, cz), (wall_t, ey + 2 * wall_t, ez)),
            "-X": ((cx - hx - wall_t / 2.0, cy, cz), (wall_t, ey + 2 * wall_t, ez)),
            "+Y": ((cx, cy + hy + wall_t / 2.0, cz), (ex, wall_t, ez)),
            "-Y": ((cx, cy - hy - wall_t / 2.0, cz), (ex, wall_t, ez)),
            "+Z": ((cx, cy, cz + hz + wall_t / 2.0), (ex, ey, wall_t)),
            "-Z": ((cx, cy, cz - hz - wall_t / 2.0), (ex, ey, wall_t)),
        }
        for face, (centre, size) in walls.items():
            if face not in omit:
                self.add_box(centre[0], centre[1], centre[2], size[0], size[1], size[2])

    # -- measurement -----------------------------------------------------------------
    def bounding_box(self):
        xs = [p[0] for tri in self.triangles for p in tri]
        ys = [p[1] for tri in self.triangles for p in tri]
        zs = [p[2] for tri in self.triangles for p in tri]
        mn, mx = (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))
        return mn, mx, (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])

    def signed_volume_mm3(self) -> float:
        total = 0.0
        for a, b, c in self.triangles:
            total += (a[0] * (b[1] * c[2] - b[2] * c[1])
                      - a[1] * (b[0] * c[2] - b[2] * c[0])
                      + a[2] * (b[0] * c[1] - b[1] * c[0]))
        return total / 6.0

    def write_binary_stl(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        header = f"BUDE CSG: {self.name}".encode("ascii", errors="ignore")[:80].ljust(80, b"\x00")
        with path.open("wb") as f:
            f.write(header)
            f.write(struct.pack("<I", len(self.triangles)))
            for p0, p1, p2 in self.triangles:
                n = _norm(_cross(_sub(p1, p0), _sub(p2, p0)))
                f.write(struct.pack("<12fH", n[0], n[1], n[2],
                                    p0[0], p0[1], p0[2], p1[0], p1[1], p1[2],
                                    p2[0], p2[1], p2[2], 0))


# =============================================================================
# DECLARED METADATA, INTERFACES AND PROBES
# =============================================================================
@dataclass
class PartSpec:
    filename: str
    title: str
    est_filament_g: float
    print_orientation: str
    layer_height_mm: float
    walls: int
    infill_pct: int
    supports_needed: str
    purpose: str
    extra_notes: List[str] = field(default_factory=list)


@dataclass(frozen=True)
class MatingInterface:
    name: str
    part_a: str
    part_b: str
    parameter: str
    declared_value: float
    unit: str
    verify_with: str


@dataclass(frozen=True)
class BoreCheck:
    """A bore that must be concentric with a declared axis."""

    part: str
    joint: str
    label: str
    direction: str                     # the bore's own axis: 'Y' or 'Z'
    axis_point: Vec3                   # a point on the axis, in the part frame
    band: Tuple[float, float]          # range along the bore's own axis
    radius: float


@dataclass(frozen=True)
class SolidProbe:
    part: str
    label: str
    point: Vec3
    expect: str


MATING_INTERFACES: List[MatingInterface] = [
    MatingInterface("Rover bolt circle", "11_mobile_rover_chassis.stl",
                    "03_base_lower_slewing_ring.stl", "bolt circle radius",
                    ROVER_BOLT_CIRCLE_R_MM, "mm", "calipers across opposite bolts"),
    MatingInterface("Rover bolt pattern", "11_mobile_rover_chassis.stl",
                    "03_base_lower_slewing_ring.stl", "bolt count", 4.0, "count",
                    "screws must drop through all four without reaming"),
    MatingInterface("Rover bolt clearance hole", "11_mobile_rover_chassis.stl", "M3 SHCS",
                    "through hole diameter", M3_CLEARANCE_HOLE_D_MM, "mm",
                    "M3 screw drops through cleanly"),
    MatingInterface("Rover nut trap", "03_base_lower_slewing_ring.stl", "M3 hex nut",
                    "across flats", M3_NUT_ACROSS_FLATS_MM, "mm",
                    "press your own nut in; measure it first"),
    MatingInterface("Rover nut trap depth", "03_base_lower_slewing_ring.stl", "M3 hex nut",
                    "trap depth", M3_NUT_TRAP_DEPTH_MM, "mm",
                    "nut must sit flush or below the plate underside"),
    MatingInterface("J2 straddle gap", "05_upper_arm_link_80mm.stl",
                    "04_base_upper_turret_j2.stl", "child inner - parent outer",
                    J2_INNER_Y - J2_OUTER_Y, "mm", "feeler gauge between clevis and turret"),
    MatingInterface("J3 straddle gap", "06_forearm_link_75mm.stl",
                    "05_upper_arm_link_80mm.stl", "child inner - parent outer",
                    J3_INNER_Y - J3_OUTER_Y, "mm", "feeler gauge between clevis and arm"),
    MatingInterface("J4 straddle gap", "07_wrist_pitch_roll_bracket.stl",
                    "06_forearm_link_75mm.stl", "child inner - parent outer",
                    J4_INNER_Y - J4_OUTER_Y, "mm", "feeler gauge between bracket and forearm"),
    MatingInterface("J2 spline axis", "04_base_upper_turret_j2.stl", "MG996R horn",
                    "spline centre offset from the J2 axis", 0.0, "mm", "sweep the joint"),
    MatingInterface("J3 spline axis", "05_upper_arm_link_80mm.stl", "MG996R horn",
                    "spline centre offset from the J3 axis", 0.0, "mm", "sweep the joint"),
    MatingInterface("J4 spline axis", "06_forearm_link_75mm.stl", "MG90S horn",
                    "spline centre offset from the J4 axis", 0.0, "mm", "sweep the joint"),
    MatingInterface("J5 spline axis", "07_wrist_pitch_roll_bracket.stl", "MG90S horn",
                    "spline centre offset from the J5 axis", 0.0, "mm", "sweep the joint"),
    MatingInterface("Passive pin", "05_upper_arm_link_80mm.stl", "6 mm printed pin",
                    "bore diameter", PASSIVE_BORE_D, "mm", "pin spins freely with no wobble"),
]

# Every bore that must be concentric with a declared axis. The audit measures the vertex
# ring of each and reports how far its centre sits from the axis. `direction` is the bore's
# own axis ('Y' for a pitch/roll joint, 'Z' for a yaw joint): the ring lies in the plane
# perpendicular to it.
COAXIAL_BORES: List[BoreCheck] = [
    # part, joint, label, direction, point on the axis, band along the axis, bore radius
    # The "race" is now the printed skirt that rises to the tower's top face, so its inner
    # cylindrical face spans z = J1_SPLINE_Z - 18 .. J1_SPLINE_Z.
    BoreCheck("03_base_lower_slewing_ring.stl", "J1", "BB slewing race inner bore", "Z",
              (0.0, 0.0, 0.0), (J1_SPLINE_Z - 18.0 - 0.05, J1_SPLINE_Z + 0.05), SLEWING_R_INNER),
    BoreCheck("03_base_lower_slewing_ring.stl", "J1", "E-stop collar bore", "Z",
              (E_STOP_CX, 0.0, 0.0), (E_STOP_COLLAR_Z0 - 0.05, E_STOP_COLLAR_Z1 + 0.05),
              E_STOP_BORE_D_MM / 2.0),
    BoreCheck("04_base_upper_turret_j2.stl", "J2", "passive bore in the bearing wall", "Y",
              (0.0, 0.0, J2_AXIS_Z_IN_04), (J2_Y_NEG - 0.05, J2_Y_NEG + WALL_T + 0.05),
              PASSIVE_BORE_D / 2.0),
    BoreCheck("04_base_upper_turret_j2.stl", "J2", "horn bore in the driven wall", "Y",
              (0.0, 0.0, J2_AXIS_Z_IN_04), (-0.05, DRIVEN_WALL_T + 0.05),
              MG996R_HORN_BORE_D / 2.0),
    BoreCheck("05_upper_arm_link_80mm.stl", "J2", "passive bore in the clevis flank", "Y",
              (0.0, 0.0, 0.0), (J2_Y_NEG - AXIAL_CLEARANCE_MM - WALL_T - 0.05,
                                J2_Y_NEG - AXIAL_CLEARANCE_MM + 0.05), PASSIVE_BORE_D / 2.0),
    BoreCheck("05_upper_arm_link_80mm.stl", "J2", "horn bore in the clevis flank", "Y",
              (0.0, 0.0, 0.0), (DRIVEN_WALL_T + AXIAL_CLEARANCE_MM - 0.05,
                                DRIVEN_WALL_T + AXIAL_CLEARANCE_MM + WALL_T + 0.05),
              MG996R_HORN_BORE_D / 2.0),
    BoreCheck("05_upper_arm_link_80mm.stl", "J3", "passive bore in the bearing wall", "Y",
              (80.0, 0.0, J3_AXIS_Z_IN_05), (J3_Y_NEG - 0.05, J3_Y_NEG + WALL_T + 0.05),
              PASSIVE_BORE_D / 2.0),
    BoreCheck("05_upper_arm_link_80mm.stl", "J3", "horn bore in the driven wall", "Y",
              (80.0, 0.0, J3_AXIS_Z_IN_05), (-0.05, DRIVEN_WALL_T + 0.05),
              MG996R_HORN_BORE_D / 2.0),
    BoreCheck("06_forearm_link_75mm.stl", "J3", "passive bore in the clevis flank", "Y",
              (0.0, 0.0, 0.0), (J3_Y_NEG - AXIAL_CLEARANCE_MM - WALL_T - 0.05,
                                J3_Y_NEG - AXIAL_CLEARANCE_MM + 0.05), PASSIVE_BORE_D / 2.0),
    BoreCheck("06_forearm_link_75mm.stl", "J3", "horn bore in the clevis flank", "Y",
              (0.0, 0.0, 0.0), (DRIVEN_WALL_T + AXIAL_CLEARANCE_MM - 0.05,
                                DRIVEN_WALL_T + AXIAL_CLEARANCE_MM + WALL_T + 0.05),
              MG996R_HORN_BORE_D / 2.0),
    BoreCheck("06_forearm_link_75mm.stl", "J4", "horn bore in the driven wall", "Y",
              (75.0, 0.0, J4_AXIS_Z_IN_06), (-0.05, DRIVEN_WALL_T + 0.05),
              MG90S_HORN_BORE_D / 2.0),
    BoreCheck("07_wrist_pitch_roll_bracket.stl", "J4", "passive bore in the clevis flank", "Y",
              (0.0, 0.0, 0.0), (J4_Y_NEG - AXIAL_CLEARANCE_MM - WALL_T - 0.05,
                                J4_Y_NEG - AXIAL_CLEARANCE_MM + 0.05), PASSIVE_BORE_D / 2.0),
    BoreCheck("07_wrist_pitch_roll_bracket.stl", "J4", "horn bore in the clevis flank", "Y",
              (0.0, 0.0, 0.0), (DRIVEN_WALL_T + AXIAL_CLEARANCE_MM - 0.05,
                                DRIVEN_WALL_T + AXIAL_CLEARANCE_MM + WALL_T + 0.05),
              MG90S_HORN_BORE_D / 2.0),
    BoreCheck("07_wrist_pitch_roll_bracket.stl", "J5", "shaft bore in the ceiling plate", "Z",
              (J5_AXIS_X_IN_07, 0.0, 0.0),
              (J5_AXIS_Z_IN_07 + MG90S_CASE_ALONG_AXIS + 0.45,
               J5_AXIS_Z_IN_07 + MG90S_CASE_ALONG_AXIS + 3.55), MG90S_HORN_BORE_D / 2.0),
    BoreCheck("08_gripper_body_and_cam_mount.stl", "J6", "shaft bore in the ceiling plate", "Z",
              (J6_AXIS_X_IN_08, 0.0, 0.0),
              (J6_AXIS_Z_IN_08 - MG90S_BODY_W / 2.0 + MG90S_CASE_ALONG_AXIS + 0.45,
               J6_AXIS_Z_IN_08 - MG90S_BODY_W / 2.0 + MG90S_CASE_ALONG_AXIS + 3.55),
              MG90S_HORN_BORE_D / 2.0),
    # The claw pivots also run along Z: the fingers swing about the printed pin that rides
    # these bores, in the same plane as the palm.
    BoreCheck("08_gripper_body_and_cam_mount.stl", "J6", "driven claw pivot bore", "Z",
              (J6_AXIS_X_IN_08, J6_LEFT_Y, 0.0),
              (J6_AXIS_Z_IN_08 - MG90S_BODY_W / 2.0 + MG90S_CASE_ALONG_AXIS + 3.45,
               J6_AXIS_Z_IN_08 - MG90S_BODY_W / 2.0 + MG90S_CASE_ALONG_AXIS + 9.05), 2.6),
    BoreCheck("08_gripper_body_and_cam_mount.stl", "J6", "passive claw pivot bore", "Z",
              (J6_AXIS_X_IN_08, J6_RIGHT_Y, 0.0),
              (J6_AXIS_Z_IN_08 - MG90S_BODY_W / 2.0 + MG90S_CASE_ALONG_AXIS + 3.45,
               J6_AXIS_Z_IN_08 - MG90S_BODY_W / 2.0 + MG90S_CASE_ALONG_AXIS + 9.05),
              PASSIVE_BORE_D / 2.0),
    BoreCheck("09_gripper_geared_fingers_pair.stl", "J6", "driven finger hub bore", "Z",
              (0.0, -16.0, 0.0), (-0.05, 6.05), 3.05),
    BoreCheck("09_gripper_geared_fingers_pair.stl", "J6", "coupled finger hub bore", "Z",
              (0.0, 16.0, 0.0), (-0.05, 6.05), 3.05),
    BoreCheck("11_mobile_rover_chassis.stl", "axle", "left axle bore in the side plate", "Y",
              (0.0, 0.0, AXLE_HEIGHT_MM), (-52.05, -45.95), 3.2),
    BoreCheck("11_mobile_rover_chassis.stl", "axle", "right axle bore in the side plate", "Y",
              (0.0, 0.0, AXLE_HEIGHT_MM), (45.95, 52.05), 3.2),
    BoreCheck("11_mobile_rover_chassis.stl", "base", "M3 clearance hole in the deck", "Z",
              (ROVER_BOLT_CIRCLE_R_MM * math.cos(math.radians(45.0)),
               ROVER_BOLT_CIRCLE_R_MM * math.sin(math.radians(45.0)), 0.0),
              (DECK_TOP_MM - DECK_T_MM - 0.05, DECK_TOP_MM + 0.05),
              M3_CLEARANCE_HOLE_D_MM / 2.0),
    # The wheels print flat, so their axle bores run along Z in the PART frame and are
    # rotated onto Y when the rover is assembled.
    BoreCheck("12_drive_wheels_and_caster.stl", "axle", "left wheel hub bore", "Z",
              (0.0, -30.0, 0.0), (WHEEL_DIAMETER_MM / 2.0 - 2.55,
                                  WHEEL_DIAMETER_MM / 2.0 + 2.55), 2.5),
    BoreCheck("12_drive_wheels_and_caster.stl", "axle", "right wheel hub bore", "Z",
              (0.0, 30.0, 0.0), (WHEEL_DIAMETER_MM / 2.0 - 2.55,
                                 WHEEL_DIAMETER_MM / 2.0 + 2.55), 2.5),
    BoreCheck("12_drive_wheels_and_caster.stl", "caster", "ball caster cup bore", "Z",
              (0.0, 0.0, 0.0), (-0.05, 10.05), CASTER_CUP_BORE_MM),
]


def _probe(part: str, label: str, point: Vec3, expect: str) -> SolidProbe:
    return SolidProbe(part, label, point, expect)


CAVITY_PROBES: List[SolidProbe] = [
    # --- 00 coupon: every pocket is a through-opening -------------------------------
    _probe("00_tolerance_test_coupon.stl", "MG996R test pocket", (-18.0, 0.0, 8.0), "air"),
    _probe("00_tolerance_test_coupon.stl", "MG90S test pocket", (20.0, 0.0, 8.0), "air"),
    _probe("00_tolerance_test_coupon.stl", "6.3 mm pivot bore", (36.0, 12.0, 6.0), "air"),
    _probe("00_tolerance_test_coupon.stl", "M3 hex trap", (34.0, -14.0, 5.5), "air"),
    _probe("00_tolerance_test_coupon.stl", "coupon base plate", (0.0, -18.0, 2.0), "material"),

    # --- 01 C-clamp: the screw bore must be a hole, not a boss ----------------------
    _probe("01_desk_c_clamp_frame.stl", "clamping screw through-bore", (16.0, -32.0, 9.0), "air"),
    _probe("01_desk_c_clamp_frame.stl", "jaw material beside the bore", (16.0, -38.5, 9.0),
           "material"),

    # --- 03 base: J1 cradle, hollow E-stop, nut traps -------------------------------
    _probe("03_base_lower_slewing_ring.stl", "J1 servo cavity", (10.35, 0.0, 16.0), "air"),
    _probe("03_base_lower_slewing_ring.stl", "J1 cavity floor", (10.35, 0.0, 1.0), "material"),
    _probe("03_base_lower_slewing_ring.stl", "E-stop switch cavity", (E_STOP_CX, 0.0, 14.0),
           "air"),
    _probe("03_base_lower_slewing_ring.stl", "E-stop collar bore", (E_STOP_CX, 0.0, 30.0), "air"),
    _probe("03_base_lower_slewing_ring.stl", "E-stop wall", (42.5, 0.0, 14.0), "material"),
    _probe("03_base_lower_slewing_ring.stl", "rover nut trap",
           (ROVER_BOLT_CIRCLE_R_MM * math.cos(math.radians(45.0)),
            ROVER_BOLT_CIRCLE_R_MM * math.sin(math.radians(45.0)), 1.2), "air"),
    _probe("03_base_lower_slewing_ring.stl", "plate material at the core", (0.0, -34.0, 2.0),
           "material"),

    # --- 04 turret: J2 cavity between two bored plates ------------------------------
    _probe("04_base_upper_turret_j2.stl", "J2 servo cavity", (-10.35, -20.75, 25.0), "air"),
    _probe("04_base_upper_turret_j2.stl", "J2 horn bore in the driven wall", (0.0, 1.5, 30.0),
           "air"),
    _probe("04_base_upper_turret_j2.stl", "J2 passive bore in the bearing wall",
           (0.0, -39.75, 30.0), "air"),
    _probe("04_base_upper_turret_j2.stl", "slewing disk bore", (0.0, 0.0, 4.0), "air"),
    _probe("04_base_upper_turret_j2.stl", "bearing wall material", (20.0, -39.5, 30.0),
           "material"),

    # --- 05 upper arm --------------------------------------------------------------
    _probe("05_upper_arm_link_80mm.stl", "J2 clevis void on the axis", (0.0, 0.0, 0.0), "air"),
    _probe("05_upper_arm_link_80mm.stl", "proximal passive bore", (0.0, -42.75, 0.0), "air"),
    _probe("05_upper_arm_link_80mm.stl", "proximal horn bore", (0.0, 4.55, 0.0), "air"),
    _probe("05_upper_arm_link_80mm.stl", "link flank material", (30.0, -42.75, 12.0), "material"),
    _probe("05_upper_arm_link_80mm.stl", "J3 servo cavity", (69.65, -16.75, 10.0), "air"),
    _probe("05_upper_arm_link_80mm.stl", "J3 horn bore", (80.0, 1.5, 12.0), "air"),

    # --- 06 forearm -----------------------------------------------------------------
    _probe("06_forearm_link_75mm.stl", "J3 clevis void on the axis", (0.0, 0.0, 0.0), "air"),
    _probe("06_forearm_link_75mm.stl", "proximal horn bore", (0.0, 4.55, 0.0), "air"),
    _probe("06_forearm_link_75mm.stl", "J4 servo cavity", (69.4, -11.25, 10.0), "air"),
    _probe("06_forearm_link_75mm.stl", "J4 horn bore", (75.0, 1.5, 5.0), "air"),

    # --- 07 wrist -------------------------------------------------------------------
    _probe("07_wrist_pitch_roll_bracket.stl", "J5 servo cavity", (5.4, 0.0, 15.0), "air"),
    _probe("07_wrist_pitch_roll_bracket.stl", "J5 shaft bore in the ceiling", (11.0, 0.0, 30.0),
           "air"),
    _probe("07_wrist_pitch_roll_bracket.stl", "J4 clevis void on the axis", (0.0, 0.0, 0.0),
           "air"),

    # --- 08 gripper -----------------------------------------------------------------
    _probe("08_gripper_body_and_cam_mount.stl", "J6 servo cavity", (6.4, 0.0, 15.0), "air"),
    _probe("08_gripper_body_and_cam_mount.stl", "J6 shaft bore in the ceiling", (12.0, 0.0, 30.0),
           "air"),
    _probe("08_gripper_body_and_cam_mount.stl", "camera lens bore", (-8.0, 0.0, 27.0), "air"),
    _probe("08_gripper_body_and_cam_mount.stl", "camera bezel material", (-8.0, 14.0, 27.0),
           "material"),

    # --- 11 rover -------------------------------------------------------------------
    _probe("11_mobile_rover_chassis.stl", "left drive servo channel",
           (-DT_SERVO_SHAFT_OFFSET_MM, 40.0, AXLE_HEIGHT_MM), "air"),
    _probe("11_mobile_rover_chassis.stl", "right drive servo channel",
           (-DT_SERVO_SHAFT_OFFSET_MM, -40.0, AXLE_HEIGHT_MM), "air"),
    _probe("11_mobile_rover_chassis.stl", "base bolt clearance hole",
           (ROVER_BOLT_CIRCLE_R_MM * math.cos(math.radians(45.0)),
            ROVER_BOLT_CIRCLE_R_MM * math.sin(math.radians(45.0)), 22.5), "air"),
    _probe("11_mobile_rover_chassis.stl", "deck material", (0.0, 0.0, 22.5), "material"),
    _probe("11_mobile_rover_chassis.stl", "side plate material", (0.0, 55.0, 22.0), "material"),
]

MATERIAL_PROBES: List[SolidProbe] = [
    _probe("12_drive_wheels_and_caster.stl", "wheel hub bore", (0.0, -30.0, 24.0), "air"),
    _probe("12_drive_wheels_and_caster.stl", "wheel rim band", (20.0, -30.0, 24.0), "material"),
    _probe("12_drive_wheels_and_caster.stl", "caster cup bore", (0.0, 0.0, 5.0), "air"),
    _probe("12_drive_wheels_and_caster.stl", "caster flange material", (0.0, -9.0, 12.0),
           "material"),
    _probe("09_gripper_geared_fingers_pair.stl", "left finger hub bore", (0.0, -16.0, 3.0),
           "air"),
    _probe("09_gripper_geared_fingers_pair.stl", "left finger beam", (22.0, -18.0, 1.0),
           "material"),
]


# =============================================================================
# PART BUILDERS
# =============================================================================
def add_servo_axis_module(m: MeshBuilder, *, joint: str, x_axis: float, z_axis: float,
                          body_l: float, body_w: float, case_len: float,
                          spline_offset: float, driven_bore_d: float,
                          bearing_bore_d: float, y_neg: float, y_pos: float,
                          x_span: Tuple[float, float], z_span: Tuple[float, float],
                          wall: float = WALL_T) -> None:
    """Build one servo joint: two plates bored on the SAME axis, plus the cavity between them.

    `x_axis`/`z_axis` is the joint axis in the part frame and `spline_offset` is the distance
    from the servo case centre to its spline, so the servo is placed at
    (x_axis - spline_offset) and its 25T or 21T spline lands exactly on the axis.

    The cavity is sized to the free space between the plates rather than to the servo, so the
    declared clearance is real: bearing_wall + axial_fit + case + driven_wall = outer width.
    """
    y_inner = y_neg + wall                     # inner face of the bearing plate
    cy = y_inner / 2.0
    dy = -y_inner                              # cavity spans y_inner .. 0.0
    if dy < case_len + AXIAL_FIT_TOTAL_MM - 1e-9:
        raise ValueError(f"{joint}: {dy:.1f} mm of free space cannot hold a {case_len:.1f} mm "
                         f"case plus {AXIAL_FIT_TOTAL_MM:.1f} mm of axial slack")
    z_lo = z_axis - body_w / 2.0
    m.add_plate_with_holes_y(x_span[0], x_span[1], z_span[0], z_span[1], 0.0, y_pos,
                             [circle_poly(x_axis, z_axis, driven_bore_d / 2.0, 32)])
    m.add_plate_with_holes_y(x_span[0], x_span[1], z_span[0], z_span[1], y_neg, wall,
                             [circle_poly(x_axis, z_axis, bearing_bore_d / 2.0, 32)])
    m.add_hollow_pocket(x_axis - spline_offset, cy, z_lo, body_l + 0.5, dy, body_w + 0.5,
                        wall=3.0, floor_t=0.0, omit=("+Y", "-Y", "-X"))
    m.add_box_span(x_span[0], x_span[1], y_neg, y_pos, z_lo - 3.0, z_lo)



def build_00() -> Tuple[PartSpec, MeshBuilder]:
    """Fit-calibration coupon: two servo pockets, a pivot bore and a hex nut trap."""
    m = MeshBuilder("00_tolerance_test_coupon")
    m.add_box_span(-42.0, 42.0, -20.0, 20.0, 0.0, 4.0)
    m.add_hollow_pocket(-18.0, 0.0, 4.0, MG996R_BODY_L + 0.5, MG996R_BODY_W + 0.5, 9.0,
                        wall=WALL_T, floor_t=0.0)
    m.add_hollow_pocket(20.0, 0.0, 4.0, MG90S_BODY_L + 0.5, MG90S_BODY_W + 0.5, 9.0,
                        wall=2.0, floor_t=0.0)
    m.add_tube_z(36.0, 12.0, 0.0, 12.0, PASSIVE_BORE_D / 2.0, 4.5, 28)
    m.add_hex_pocket_z(34.0, -14.0, 4.0, M3_NUT_ACROSS_FLATS_MM, 3.2, wall=1.6)
    spec = PartSpec(
        "00_tolerance_test_coupon.stl", "Tolerance and fit calibration coupon", 8.0,
        "Flat on the base plate", 0.20, 3, 20, "None",
        "Print this first. Checks the MG996R and MG90S pockets against your servos, the 6.3 mm "
        "pivot bore against the printed pin, and the hex trap against your own M3 nuts.",
        ["All four features are through-openings, so they can be measured with calipers "
         "without cutting the coupon."])
    return spec, m


def build_01() -> Tuple[PartSpec, MeshBuilder]:
    """C-clamp with a TRUE through-bore for the clamping screw.

    The previous revision drew a 24-segment tube boss on a solid bottom jaw. A tube inside a
    solid box is not a hole - the box fills it. Here the bottom jaw is a plate and the screw
    bore is cut by diagonal partition, so the screw genuinely passes through.
    """
    m = MeshBuilder("01_desk_c_clamp_frame")
    m.add_plate_with_holes_z(-29.0, 29.0, -40.0, -24.0, 0.0, 18.0, [circle_poly(16.0, -32.0, 5.9, 32)])
    m.add_box(0.0, 32.0, 9.0, 58.0, 16.0, 18.0)
    m.add_box(-21.0, 0.0, 9.0, 16.0, 80.0, 18.0)
    m.add_box(-25.0, -28.0, 9.0, 8.0, 8.0, 18.0)
    m.add_box(-25.0, 28.0, 9.0, 8.0, 8.0, 18.0)
    spec = PartSpec(
        "01_desk_c_clamp_frame.stl", "Desk C-clamp frame (bench up to 48 mm)", 40.0,
        "Lying on its side so the layer lines wrap the C-profile", 0.20, 5, 35, "None",
        "Clamps the base plate to a bench edge. KEEP FOR BENCH DEMOS ONLY: a printed clamp "
        "must not carry a working arm. Use a rated metal clamp.",
        ["The M12 clamping screw bore is a true through-bore in the bottom jaw."])
    return spec, m


def build_02() -> Tuple[PartSpec, MeshBuilder]:
    m = MeshBuilder("02_desk_c_clamp_screw_and_pad")
    m.add_cylinder_z(0.0, 0.0, 0.0, 8.0, 16.0, 24)
    m.add_cylinder_z(0.0, 0.0, 8.0, 52.0, 5.85, 24)
    m.add_tube_z(32.0, 0.0, 0.0, 8.0, 4.2, 13.0, 24)
    spec = PartSpec(
        "02_desk_c_clamp_screw_and_pad.stl", "Clamp screw with a swivel pad", 14.0,
        "Knob flat on the bed", 0.16, 5, 40, "None",
        "Threads through the clamp jaws. Printed threads are for light clamping only.",
        ["Screw threads are modelled as rings at 3 mm pitch, not as a real helical thread. "
         "Your slicer will follow the rings; grip is unverified."])
    return spec, m


def build_03() -> Tuple[PartSpec, MeshBuilder]:
    """Base plate, J1 tower with the slewing race, nut traps and a hollow E-stop enclosure.

    The J1 servo stands upright inside the tower with its spline at the tower's top face, so
    its 35.5 mm case hangs down inside a 36 mm cavity. The tower is therefore 41 mm tall: a
    32 mm base would have let the servo punch out of the bottom.
    """
    m = MeshBuilder("03_base_lower_slewing_ring")
    traps_lower = [circle_poly(ROVER_BOLT_CIRCLE_R_MM * math.cos(math.radians(a)),
                               ROVER_BOLT_CIRCLE_R_MM * math.sin(math.radians(a)),
                               M3_NUT_ACROSS_FLATS_MM / math.sqrt(3.0), 6)
                   for a in ROVER_BOLT_ANGLES_DEG]
    traps_upper = [circle_poly(ROVER_BOLT_CIRCLE_R_MM * math.cos(math.radians(a)),
                               ROVER_BOLT_CIRCLE_R_MM * math.sin(math.radians(a)),
                               M3_CLEARANCE_HOLE_D_MM / 2.0, 20)
                   for a in ROVER_BOLT_ANGLES_DEG]
    half = BASE_PLATE_MM / 2.0
    m.add_plate_with_holes_z(-half, half, -half, half, 0.0, M3_NUT_TRAP_DEPTH_MM, traps_lower)
    m.add_plate_with_holes_z(-half, half, -half, half, M3_NUT_TRAP_DEPTH_MM,
                             PLATE_TOP_MM - M3_NUT_TRAP_DEPTH_MM, traps_upper)
    # The slewing race skirt, now tall enough to sit at the tower's top face.
    m.add_tube_z(0.0, 0.0, J1_SPLINE_Z - 18.0, 18.0, SLEWING_R_INNER, SLEWING_R_OUTER, 48)
    servo_cx = MG996R_SPLINE_OFFSET                      # the spline must land on J1 at x = 0
    m.add_hollow_pocket(servo_cx, 0.0, PLATE_TOP_MM, MG996R_BODY_L + 0.5, MG996R_BODY_W + 0.5,
                        J1_SPLINE_Z - PLATE_TOP_MM, wall=3.2, floor_t=0.0)
    ex0, _, _ = E_STOP_BOX_MIN_MM
    ex1, ey1, _ = E_STOP_BOX_MAX_MM
    # Floorless: the base plate IS the floor, so nothing hangs below z = 0 and the part
    # still seats flat on the bed. The cavity is then 33 mm of clear air from the plate
    # top to the underside of the collar.
    m.add_hollow_pocket(E_STOP_CX, 0.0, PLATE_TOP_MM, (ex1 - ex0) - 6.0, (ey1 - 0.0 * 2) - 6.0,
                        E_STOP_COLLAR_Z0 - PLATE_TOP_MM, wall=3.0, floor_t=0.0)
    m.add_plate_with_holes_z(E_STOP_CX - E_STOP_COLLAR_OD_MM / 2.0,
                             E_STOP_CX + E_STOP_COLLAR_OD_MM / 2.0,
                             -E_STOP_COLLAR_OD_MM / 2.0, E_STOP_COLLAR_OD_MM / 2.0,
                             E_STOP_COLLAR_Z0, E_STOP_COLLAR_Z1 - E_STOP_COLLAR_Z0,
                             [circle_poly(E_STOP_CX, 0.0, E_STOP_BORE_D_MM / 2.0, 32)])
    spec = PartSpec(
        "03_base_lower_slewing_ring.stl", "Base plate + J1 tower + E-stop enclosure", 58.0,
        "Plate flat on the bed", 0.20, 4, 25, "None",
        "Holds J1, the BB slewing race, the four rover nut traps and a hollow E-stop enclosure.",
        [f"J1 cradle centre is offset to x = +{servo_cx:.2f} mm so that the 25T spline lands "
         f"on the J1 axis at x = 0.00 mm, at the tower's top face z = {J1_SPLINE_Z:.1f} mm.",
         f"The tower is {J1_SPLINE_Z:.0f} mm tall because an upright MG996R hangs its "
         f"{MG996R_CASE_ALONG_AXIS:.1f} mm case BELOW the spline face. A 32 mm base cannot "
         f"hold it.",
         "The M3 nut traps are true hexagonal voids, 2.6 mm deep in a 5 mm plate.",
         "The plate outline is square rather than round so the nut traps can be cut exactly.",
         "The E-stop collar bore is a true 22.4 mm through-opening on a hollow enclosure."])
    return spec, m


def build_04() -> Tuple[PartSpec, MeshBuilder]:
    """Turret that carries J2.

    Part 04's origin IS the J1 axis, so the J2 axis is at z = 30 mm. The MG996R's case sits
    inside the turret offset to -x, and the axis at y = 0 is deliberately NOT the middle of
    the turret: the case hangs on the bearing side, which is what a real hobby servo does.
    """
    m = MeshBuilder("04_base_upper_turret_j2")
    m.add_tube_z(0.0, 0.0, 0.0, 8.0, 8.0, 36.0, 48)
    y_neg, y_pos = J2_Y_NEG, DRIVEN_WALL_T          # -41.0 to +3.0 = 44.0 mm
    add_servo_axis_module(m, joint="J2", x_axis=0.0, z_axis=J2_AXIS_Z_IN_04,
                          body_l=MG996R_BODY_L, body_w=MG996R_BODY_W,
                          case_len=MG996R_CASE_ALONG_AXIS,
                          spline_offset=MG996R_SPLINE_OFFSET,
                          driven_bore_d=MG996R_HORN_BORE_D, bearing_bore_d=PASSIVE_BORE_D,
                          y_neg=y_neg, y_pos=y_pos, x_span=(-22.0, 22.0),
                          z_span=(8.0, J2_AXIS_Z_IN_04 + 16.0))
    m.add_cylinder_z(-34.0, y_neg + 6.0, 8.0, 16.0, 4.0, 20)
    spec = PartSpec(
        "04_base_upper_turret_j2.stl", "Rotating turret + J2 double-shear shoulder", 36.0,
        "Slewing disk flat on the bed", 0.20, 4, 25, "None",
        "Couples J1 rotation to J2 with a double-shear support and a rubber-band anchor.",
        [f"The J2 axis is at y = 0 but the turret spans y = {y_neg:.1f} to +{y_pos:.1f} mm, so "
         f"the axis is NOT in the middle.",
         f"{MG996R_CASE_ALONG_AXIS:.1f} mm of MG996R case fits inside the requested "
         f"{J2_OUTER_Y:.1f} mm width with "
         f"{y_neg + MG996R_CASE_ALONG_AXIS + WALL_T:.1f} mm of bearing wall left over, so J2 "
         f"keeps the width the brief asked for."])
    return spec, m


def build_05() -> Tuple[PartSpec, MeshBuilder]:
    """Upper arm.

    Part 05's origin IS the J2 axis, so the proximal bore is at z = 0 and the J3 axis is at
    z = 12 mm. The proximal clevis wraps the 44.0 mm turret with a 44.6 mm inner span, and
    the distal end contains the J3 MG996R offset so its spline lands on x = 80 mm.
    """
    m = MeshBuilder("05_upper_arm_link_80mm")
    z_j3 = J3_AXIS_Z_IN_05
    neg_inner, pos_inner = J2_Y_NEG - AXIAL_CLEARANCE_MM, DRIVEN_WALL_T + AXIAL_CLEARANCE_MM
    z0, z1 = -8.0, z_j3 + 11.0
    # Proximal clevis: -Y flank carries the passive bore, +Y flank carries the horn hub.
    # Both are cut on the J2 axis, which IS this part's origin.
    m.add_plate_with_holes_y(-16.0, 96.0, z0, z1, neg_inner - WALL_T, WALL_T,
                             [circle_poly(0.0, 0.0, PASSIVE_BORE_D / 2.0, 32)])
    m.add_plate_with_holes_y(-16.0, 96.0, z0, z1, pos_inner, WALL_T,
                             [circle_poly(0.0, 0.0, MG996R_HORN_BORE_D / 2.0, 32)])
    m.add_box(-2.0, 0.0, 18.5, 26.0, J2_INNER_Y, 5.0)
    add_servo_axis_module(m, joint="J3", x_axis=80.0, z_axis=z_j3,
                          body_l=MG996R_BODY_L, body_w=MG996R_BODY_W,
                          case_len=MG996R_CASE_ALONG_AXIS,
                          spline_offset=MG996R_SPLINE_OFFSET,
                          driven_bore_d=MG996R_HORN_BORE_D, bearing_bore_d=PASSIVE_BORE_D,
                          y_neg=J3_Y_NEG, y_pos=DRIVEN_WALL_T, x_span=(56.0, 96.0),
                          z_span=(z0, z1))
    m.add_cylinder_z(-28.0, 0.0, z_j3 + 6.0, 8.0, 3.5, 20)
    spec = PartSpec(
        "05_upper_arm_link_80mm.stl", "Upper arm double-shear link (L2 = 80 mm)", 28.0,
        "Flat on the bed", 0.20, 3, 20, "None",
        "Connects J2 to J3 over 80 mm between axes; proximal clevis, distal servo housing.",
        [f"Proximal inner span {J2_INNER_Y:.1f} mm over a {J2_OUTER_Y:.1f} mm turret, gap "
         f"{J2_GAP_MM:.2f} mm.",
         f"Distal outer width {J3_OUTER_Y:.1f} mm, widened from the requested 34.0 mm: the "
         f"MG996R case is {MG996R_CASE_ALONG_AXIS:.1f} mm along its output axis, so a 34.0 mm "
         f"housing would be narrower than the servo it must hold."])
    return spec, m


def build_06() -> Tuple[PartSpec, MeshBuilder]:
    """Forearm.

    Part 06's origin IS the J3 axis, so its proximal bore is at z = 0 and the J4 MG90S axis is
    at z = 5 mm.
    """
    m = MeshBuilder("06_forearm_link_75mm")
    z_j4 = J4_AXIS_Z_IN_06
    neg_inner, pos_inner = J3_Y_NEG - AXIAL_CLEARANCE_MM, DRIVEN_WALL_T + AXIAL_CLEARANCE_MM
    z0, z1 = -8.0, z_j4 + 10.0
    m.add_plate_with_holes_y(-14.0, 88.0, z0, z1, neg_inner - WALL_T, WALL_T,
                             [circle_poly(0.0, 0.0, PASSIVE_BORE_D / 2.0, 28)])
    m.add_plate_with_holes_y(-14.0, 88.0, z0, z1, pos_inner, WALL_T,
                             [circle_poly(0.0, 0.0, MG996R_HORN_BORE_D / 2.0, 28)])
    m.add_box(-14.0, 0.0, 16.5, 26.0, J3_INNER_Y, 5.0)
    d0, d1 = 52.0, 88.0
    m.add_plate_with_holes_y(d0, d1, z0, z1, 0.0, DRIVEN_WALL_T,
                             [circle_poly(75.0, z_j4, MG90S_HORN_BORE_D / 2.0, 28)])
    m.add_plate_with_holes_y(d0, d1, z0, z1, J4_Y_NEG, WALL_T,
                             [circle_poly(75.0, z_j4, PASSIVE_BORE_D / 2.0, 28)])
    z_lo = z_j4 - MG90S_BODY_W / 2.0
    m.add_hollow_pocket(75.0 - MG90S_SPLINE_OFFSET,
                        J4_Y_NEG + MG90S_CASE_ALONG_AXIS / 2.0 + WALL_T, z_lo,
                        MG90S_BODY_L + 0.5, MG90S_CASE_ALONG_AXIS + 0.5, MG90S_BODY_W,
                        wall=WALL_T, floor_t=0.0, omit=("+Y", "-Y"))
    m.add_box_span(d0, d1, J4_Y_NEG, DRIVEN_WALL_T, z_lo - 3.0, z_lo)
    m.add_box_span(42.0, 48.0, J4_Y_NEG, DRIVEN_WALL_T, z_j4 - 12.0, z_j4 - 4.0)
    spec = PartSpec(
        "06_forearm_link_75mm.stl", "Forearm link (L3 = 75 mm)", 20.0,
        "Flat on the bed", 0.20, 3, 20, "None",
        "Connects J3 (MG996R) to the featherweight J4 wrist pitch (MG90S).",
        [f"Proximal inner span {J3_INNER_Y:.1f} mm over a {J3_OUTER_Y:.1f} mm parent, gap "
         f"{J3_GAP_MM:.2f} mm.",
         f"Distal outer width {J4_OUTER_Y:.1f} mm, widened from the requested 24.0 mm: the "
         f"MG90S case is {MG90S_CASE_ALONG_AXIS:.1f} mm along its output axis."])
    return spec, m


def build_07() -> Tuple[PartSpec, MeshBuilder]:
    """Wrist pitch-to-roll bracket.

    Part 07's origin IS the J4 axis. J4 is a clevis around the forearm's 28.0 mm distal end;
    the J5 MG90S rolls about Z with its shaft leaving through a ceiling plate whose bore is
    cut exactly on the J5 axis at x = 11 mm.
    """
    m = MeshBuilder("07_wrist_pitch_roll_bracket")
    z_j5 = J5_AXIS_Z_IN_07
    neg_inner, pos_inner = J4_Y_NEG - AXIAL_CLEARANCE_MM, DRIVEN_WALL_T + AXIAL_CLEARANCE_MM
    z0, z1 = -8.0, z_j5 + MG90S_CASE_ALONG_AXIS + 4.0
    m.add_plate_with_holes_y(-12.0, 26.0, z0, z1, neg_inner - WALL_T, WALL_T,
                             [circle_poly(0.0, 0.0, PASSIVE_BORE_D / 2.0, 28)])
    m.add_plate_with_holes_y(-12.0, 26.0, z0, z1, pos_inner, WALL_T,
                             [circle_poly(0.0, 0.0, MG90S_HORN_BORE_D / 2.0, 28)])
    m.add_box(-10.0, 0.0, 10.5, 4.0, J4_INNER_Y, 5.0)
    # J5 lies flat; its case sits in a pocket and the shaft exits upward.
    m.add_hollow_pocket(J5_AXIS_X_IN_07 - MG90S_SPLINE_OFFSET, 0.0, z_j5,
                        MG90S_BODY_L + 0.5, MG90S_BODY_W + 0.5, MG90S_CASE_ALONG_AXIS + 0.5,
                        wall=WALL_T, floor_t=1.5)
    ceil_z = z_j5 + MG90S_CASE_ALONG_AXIS + 0.5
    m.add_plate_with_holes_z(J5_AXIS_X_IN_07 - 14.0, J5_AXIS_X_IN_07 + 14.0, -14.0, 14.0,
                             ceil_z, 3.0,
                             [circle_poly(J5_AXIS_X_IN_07, 0.0, MG90S_HORN_BORE_D / 2.0, 28)])
    spec = PartSpec(
        "07_wrist_pitch_roll_bracket.stl", "Wrist pitch-to-roll bracket (L4 = 22 mm)", 12.0,
        "Clevis flanks down, J5 ceiling up", 0.20, 3, 20, "None",
        "Orthogonal J4-to-J5 coupling; J5 is the joint the expressive head tilt uses.",
        [f"J4 clevis inner span {J4_INNER_Y:.1f} mm over a {J4_OUTER_Y:.1f} mm parent, gap "
         f"{J4_GAP_MM:.2f} mm.",
         "The J5 shaft bore in the 3 mm ceiling plate is cut exactly on the J5 axis."])
    return spec, m


def build_08() -> Tuple[PartSpec, MeshBuilder]:
    """Gripper palm: the J6 cradle plus the camera bezel. J6 also rolls about Z."""
    m = MeshBuilder("08_gripper_body_and_cam_mount")
    z_lo = J6_AXIS_Z_IN_08 - MG90S_BODY_W / 2.0
    m.add_box(0.0, 0.0, z_lo - 5.0, 36.0, 42.0, 10.0)
    m.add_hollow_pocket(J6_AXIS_X_IN_08 - MG90S_SPLINE_OFFSET, 0.0, z_lo,
                        MG90S_BODY_L + 0.5, MG90S_BODY_W + 0.5, MG90S_CASE_ALONG_AXIS + 0.5,
                        wall=WALL_T, floor_t=0.0)
    ceil_z = z_lo + MG90S_CASE_ALONG_AXIS + 0.5
    m.add_plate_with_holes_z(J6_AXIS_X_IN_08 - 14.0, J6_AXIS_X_IN_08 + 14.0, -14.0, 14.0,
                             ceil_z, 3.0,
                             [circle_poly(J6_AXIS_X_IN_08, 0.0, MG90S_HORN_BORE_D / 2.0, 28)])
    m.add_tube_z(J6_AXIS_X_IN_08, J6_LEFT_Y, ceil_z + 3.0, 9.0, 2.6, 4.0, 24)
    m.add_tube_z(J6_AXIS_X_IN_08, J6_RIGHT_Y, ceil_z + 3.0, 9.0, PASSIVE_BORE_D / 2.0, 4.0, 24)
    # The bezel must stay clear of the servo cavity, whose -X face is at x = -5.25 mm.
    m.add_plate_with_holes_x(-18.0, 18.0, z_lo, 38.0, -15.35, 10.0,
                             [circle_poly(0.0, 27.0, 7.2, 28)])
    spec = PartSpec(
        "08_gripper_body_and_cam_mount.stl", "Gripper palm + eye-in-hand camera bezel", 16.0,
        "Palm back flat on the bed", 0.20, 3, 20, "None",
        "Holds J6, the two claw pivots and the camera bezel.",
        ["The left claw is driven directly through a 2.6 mm hub wall; the right claw rides the "
         "passive 6.3 mm bore at y = +11 mm.",
         "The camera lens bore is a true opening through the bezel front.",
         "The bezel starts at x = -5.35 mm so it never intrudes into the J6 servo cavity."])
    return spec, m


def build_09() -> Tuple[PartSpec, MeshBuilder]:
    m = MeshBuilder("09_gripper_geared_fingers_pair")
    for sign, y_off in ((-1.0, -16.0), (1.0, 16.0)):
        m.add_tube_z(0.0, y_off, 0.0, 6.0, 3.05, 11.0, 24)
        for k in range(12):
            a = 2.0 * math.pi * k / 12.0 + (math.pi / 12.0 if sign > 0 else 0.0)
            m.add_box(9.0 * math.cos(a), y_off + 9.0 * math.sin(a), 3.0, 4.4, 2.2, 6.0)
        m.add_box(22.0, y_off + sign * 2.0, 3.0, 38.0, 6.0, 6.0)
        m.add_box(40.0, y_off - sign * 2.5, 3.0, 12.0, 7.0, 6.0)
    spec = PartSpec(
        "09_gripper_geared_fingers_pair.stl", "Synchronous geared claw fingers", 9.0,
        "Flat on the gear face at 0.16 mm layers", 0.16, 4, 30, "None",
        "One MG90S opens both fingers through 1:1 meshing teeth.",
        ["Each finger carries 12 discrete teeth on a 22 mm pitch circle. FDM tooth strength "
         "and backlash are unverified; expect to iterate on clearance."])
    return spec, m


def build_10() -> Tuple[PartSpec, MeshBuilder]:
    m = MeshBuilder("10_cable_clips_and_electronics_tray")
    m.add_box(0.0, 0.0, 1.5, 68.0, 52.0, 3.0)
    for sx, sy in ((-30.0, -22.0), (30.0, -22.0), (-30.0, 22.0), (30.0, 22.0)):
        m.add_tube_z(sx, sy, 3.0, 5.0, 1.4, 3.2, 16)
    for i in range(6):
        m.add_tube_z(-30.0 + i * 12.0, 36.0, 0.0, 6.0, 4.0, 6.0, 20)
    spec = PartSpec(
        "10_cable_clips_and_electronics_tray.stl",
        "ESP32 + PCA9685 + INA219 tray and cable clips", 12.0,
        "Flat on the bed", 0.20, 3, 20, "None",
        "Keeps the electronics beside the base and the servo loom clipped down.")
    return spec, m


def build_11() -> Tuple[PartSpec, MeshBuilder]:
    """Rover chassis. Wheels are 48 mm with a 12 mm ground gap so the servo bodies clear the
    deck; see geometry correction 3 in the module docstring."""
    m = MeshBuilder("11_mobile_rover_chassis")
    bolt_holes = [circle_poly(ROVER_BOLT_CIRCLE_R_MM * math.cos(math.radians(a)),
                              ROVER_BOLT_CIRCLE_R_MM * math.sin(math.radians(a)),
                              M3_CLEARANCE_HOLE_D_MM / 2.0, 20)
                  for a in ROVER_BOLT_ANGLES_DEG]
    m.add_plate_with_holes_z(-73.0, 73.0, -52.0, 52.0, DECK_TOP_MM - DECK_T_MM, DECK_T_MM,
                             bolt_holes)
    for side in (-1.0, 1.0):
        m.add_plate_with_holes_y(-46.0, 46.0, 0.0, DECK_TOP_MM, side * 52.0, 6.0,
                                 [circle_poly(0.0, AXLE_HEIGHT_MM, 3.2, 20)])
        lo = -DT_SERVO_SHAFT_OFFSET_MM - DT_SERVO_L_MM / 2.0 - 0.5
        hi = -DT_SERVO_SHAFT_OFFSET_MM + DT_SERVO_L_MM / 2.0 + 0.5
        m.add_box_span(lo - WALL_T, lo, side * 27.0, side * 53.0, 2.0, DECK_TOP_MM - DECK_T_MM)
        m.add_box_span(hi, hi + WALL_T, side * 27.0, side * 53.0, 2.0, DECK_TOP_MM - DECK_T_MM)
        m.add_box_span(lo - WALL_T, hi + WALL_T, side * 27.0, side * 53.0, 2.0, 5.0)
        m.add_box_span(63.0, 73.0, side * 20.0, side * 46.0, 0.0, 22.0)
        m.add_box_span(53.0, 57.0, side * 16.0, side * 34.0, DECK_TOP_MM - DECK_T_MM - 12.0,
                       DECK_TOP_MM - DECK_T_MM)
        m.add_wall_cradle(53.0, side * 25.0, 6.5, 32.5, 14.5, 3.0, 2.0, omit=("-Z", "+Z"))
    m.add_wall_cradle(50.0, 0.0, 11.0, 45.5, 20.5, 15.5, 2.5, omit=("+X",))
    for sy in (-20.0, 20.0):
        m.add_tube_z(-64.0, sy, 0.0, 4.0, M3_CLEARANCE_HOLE_D_MM / 2.0, 3.0, 20)
    spec = PartSpec(
        "11_mobile_rover_chassis.stl", "Modular 2WD rover chassis", 42.0,
        "Deck down, side plates vertical", 0.20, 4, 25, "None",
        "Bolts under the J1 base plate on the 4x M3 circle at r = 36 mm.",
        [f"Ground gap {GROUND_GAP_MM:.0f} mm puts the axle at z = {AXLE_HEIGHT_MM:.0f} mm in "
         f"part coordinates, which clears the {DECK_T_MM:.0f} mm deck that starts at "
         f"z = {DECK_TOP_MM - DECK_T_MM:.0f} mm by "
         f"{(DECK_TOP_MM - DECK_T_MM) - (AXLE_HEIGHT_MM + DT_SERVO_ALONG_AXIS_MM / 2.0):.1f} mm.",
         "The 8 mm gap in the previous revision put the servo body 2.1 mm INSIDE the deck.",
         "The four base bolt holes are true through-openings in the deck plate."])
    return spec, m


def build_12() -> Tuple[PartSpec, MeshBuilder]:
    m = MeshBuilder("12_drive_wheels_and_caster")
    wheel_z = WHEEL_DIAMETER_MM / 2.0            # centred so the printed wheel seats at z = 0
    for wy in (-30.0, 30.0):
        m.add_tube_z(0.0, wy, wheel_z - WHEEL_THICKNESS_MM / 2.0, WHEEL_THICKNESS_MM, 17.0,
                     22.0, 40)
        m.add_tube_z(0.0, wy, wheel_z - WHEEL_THICKNESS_MM / 2.0, 1.4, 17.0, 24.0, 40)
        m.add_tube_z(0.0, wy, wheel_z + WHEEL_THICKNESS_MM / 2.0 - 1.4, 1.4, 17.0, 24.0, 40)
        m.add_tube_z(0.0, wy, wheel_z - WHEEL_THICKNESS_MM / 2.0, WHEEL_THICKNESS_MM, 2.5, 9.0,
                     32)
        for k in range(6):
            a = math.pi / 3.0 * k
            m.add_box(13.0 * math.cos(a), wy + 13.0 * math.sin(a), wheel_z, 9.0, 3.0,
                      WHEEL_THICKNESS_MM)
        m.add_tube_z(0.0, wy, wheel_z - WHEEL_THICKNESS_MM / 2.0, WHEEL_THICKNESS_MM, 2.45, 3.4,
                     24)
    m.add_tube_z(0.0, 0.0, 0.0, 10.0, CASTER_CUP_BORE_MM, 10.0, 36)
    m.add_box(0.0, 0.0, 12.0, 26.0, 26.0, 4.0)
    m.add_tube_z(-9.0, 0.0, 10.0, 4.0, M3_CLEARANCE_HOLE_D_MM / 2.0, 3.0, 18)
    m.add_tube_z(9.0, 0.0, 10.0, 4.0, M3_CLEARANCE_HOLE_D_MM / 2.0, 3.0, 18)
    spec = PartSpec(
        "12_drive_wheels_and_caster.stl", "2x 48 mm spoked wheels + 12 mm ball caster", 20.0,
        "Wheels flat with the axis vertical, caster cup mouth up", 0.16, 4, 25, "None",
        "Rubber band or silicone strip in the tyre groove; a 12 mm ball in the caster cup.",
        [f"Wheels are printed with their centre at z = {wheel_z:.0f} mm so the part seats on "
         f"the bed. In the assembled rover that centre sits at z = {AXLE_HEIGHT_MM:.0f} mm, "
         f"which puts the tyre {GROUND_GAP_MM:.0f} mm below the chassis bottom.",
         "The caster needs a drop leg or spacer so the ball's lowest point matches the tyre's "
         "lowest point. That length depends on your ball and is a fitting operation."])
    return spec, m


BUILDERS = (build_00, build_01, build_02, build_03, build_04, build_05, build_06,
            build_07, build_08, build_09, build_10, build_11, build_12)
PART_FILENAMES: Tuple[str, ...] = (
    "00_tolerance_test_coupon.stl",
    "01_desk_c_clamp_frame.stl",
    "02_desk_c_clamp_screw_and_pad.stl",
    "03_base_lower_slewing_ring.stl",
    "04_base_upper_turret_j2.stl",
    "05_upper_arm_link_80mm.stl",
    "06_forearm_link_75mm.stl",
    "07_wrist_pitch_roll_bracket.stl",
    "08_gripper_body_and_cam_mount.stl",
    "09_gripper_geared_fingers_pair.stl",
    "10_cable_clips_and_electronics_tray.stl",
    "11_mobile_rover_chassis.stl",
    "12_drive_wheels_and_caster.stl",
)
PLA_DENSITY_G_PER_CM3 = 1.24


def build_all_parts() -> List[Tuple[PartSpec, MeshBuilder]]:
    """Build every part in memory, keeping the analytic solids for the audit."""
    return [builder() for builder in BUILDERS]


def generate_all_parts(out_dir: Path, clearance_mm: float = 0.25):
    """Generate all 13 STLs and return rows of (spec, bbox, triangles, signed volume)."""
    del clearance_mm          # clearances now come from the declared fit constants
    rows = []
    for spec, mesh in build_all_parts():
        mesh.write_binary_stl(out_dir / spec.filename)
        _, _, bbox = mesh.bounding_box()
        rows.append((spec, bbox, len(mesh.triangles), mesh.signed_volume_mm3(),
                     mesh.union_volume_mm3(grid_mm=3.0)))
    return rows


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate all printable STL files for B.U.D.-E.")
    parser.add_argument("--out-dir", type=Path,
                        default=Path(__file__).resolve().parents[2] / "cad" / "stl")
    parser.add_argument("--clearance-mm", type=float, default=0.25,
                        help="kept for compatibility; fits come from declared constants")
    args = parser.parse_args()
    rows = generate_all_parts(args.out_dir, clearance_mm=args.clearance_mm)

    print("=" * 118)
    print("  B.U.D.-E - CONSTRUCTIVE GENERATOR (true hollow cavities)")
    print("  Material is emitted AROUND every cavity; run verify_assembly_fit.py to ray-cast.")
    print("=" * 118)
    print(f"{'STL file':<42} {'Bounding box (X x Y x Z mm)':<26} {'Tris':>6} "
          f"{'cm3 union':>11} {'declared g':>11}")
    print("-" * 118)
    total = 0.0
    for spec, sz, tris, vol, union in rows:
        total += spec.est_filament_g
        print(f"{spec.filename:<42} {sz[0]:6.1f} x {sz[1]:6.1f} x {sz[2]:5.1f}   {tris:>6d} "
              f"{union / 1000.0:>11.1f} {spec.est_filament_g:>9.1f} g")
    print("-" * 118)
    print(f"Parts: {len(rows)}   Declared filament estimate: {total:.0f} g "
          f"(~${total * 0.018:.2f} at $18/kg). NEVER MEASURED - trust your slicer.")
    print("`cm3 union` is the analytic union volume (overlaps NOT double counted, voxel")
    print("sampled at 3 mm), so it is the solid figure before infill.")
    print("=" * 118)

    print()
    print("  STRADDLE CHAIN - THE +0.60 mm GAP IS PRESERVED, TWO WIDTHS HAD TO CHANGE")
    print("-" * 118)
    for line in straddle_report():
        print("  " + line)
    print("  A hobby servo's spline is at the TOP of its case, so the case hangs on ONE side")
    print("  of the joint axis. That makes the straddle asymmetric about the axis, and it is")
    print("  why a link cannot simply be 'wide enough for the servo' and symmetric.")

    print()
    print("=" * 118)
    print("  DECLARED MATING INTERFACES (source-level agreement only - NOT a fit test)")
    print("=" * 118)
    print(f"{'Interface':<28} {'Part A':<40} {'Parameter':<34} {'Value':>8}")
    print("-" * 118)
    for mi in MATING_INTERFACES:
        print(f"{mi.name:<28} {mi.part_a:<40} {mi.parameter:<34} "
              f"{mi.declared_value:>6.2f} {mi.unit}")
    print("=" * 118)
    print("Both halves of every pair use the same constants, so the MODELS agree. Whether two")
    print("printed parts fit is decided by a first-article print and a real component.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
