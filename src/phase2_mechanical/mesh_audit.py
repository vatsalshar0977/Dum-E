#!/usr/bin/env python3
"""Mesh-level measurement utilities shared by the Phase 2 audits.

Everything here reads the triangle soup straight out of a binary STL, so any claim in a
report can be traced back to bytes in the file.

IMPORTANT MODEL ASSUMPTION
The parts in this project are built as unions of closed, positively-wound primitives that
may interpenetrate. For a soup like that, even-odd ray-crossing gives the correct
inside/outside verdict for the UNION - which is exactly what a slicer will print. That is
what makes "is this cavity really air?" a measurable question instead of a promise.

The ray used by `point_in_solid` runs along +Z, and the query point is nudged in X and Y by
an irrational-ish offset first. Every feature in this generator is axis-aligned with
coordinates on a 0.05 mm grid, so the nudge keeps the ray off shared edges without moving it
far enough to change the answer.
"""

from __future__ import annotations

import math
import struct
from typing import Dict, Iterable, List, Sequence, Tuple

Vec3 = Tuple[float, float, float]
Tri = Tuple[Vec3, Vec3, Vec3]

RAY_NUDGE = (0.0013, -0.0027)


def read_binary_stl(path) -> Tuple[int, List[Vec3]]:
    """Validate the binary STL layout and return (triangle_count, vertex_list)."""
    raw = path.read_bytes()
    if len(raw) < 84:
        raise ValueError(f"{path.name}: {len(raw)} bytes is too small to be a binary STL.")
    num_tris = struct.unpack("<I", raw[80:84])[0]
    expected = 84 + 50 * num_tris
    if len(raw) != expected:
        raise ValueError(
            f"{path.name}: byte length {len(raw)} != 84 + 50*{num_tris} = {expected}."
        )
    verts: List[Vec3] = []
    offset = 84
    for _ in range(num_tris):
        vals = struct.unpack("<12fH", raw[offset:offset + 50])
        offset += 50
        for v in vals[:12]:
            if not math.isfinite(v):
                raise ValueError(f"{path.name}: non-finite coordinate or normal.")
        for base in (3, 6, 9):
            verts.append((vals[base], vals[base + 1], vals[base + 2]))
    return num_tris, verts


def bounds(vertices: Sequence[Vec3]):
    xs = [v[0] for v in vertices]
    ys = [v[1] for v in vertices]
    zs = [v[2] for v in vertices]
    return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))


def degenerate_triangles(vertices: Sequence[Vec3], tol_mm2: float = 1e-6) -> int:
    """Count triangles whose area is below `tol_mm2` (a slicer repair warning sign)."""
    count = 0
    for i in range(0, len(vertices), 3):
        (ax, ay, az), (bx, by, bz), (cx, cy, cz) = vertices[i], vertices[i + 1], vertices[i + 2]
        ux, uy, uz = bx - ax, by - ay, bz - az
        vx, vy, vz = cx - ax, cy - ay, cz - az
        nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
        if 0.5 * math.sqrt(nx * nx + ny * ny + nz * nz) < tol_mm2:
            count += 1
    return count


class Mesh:
    """A triangle soup with an XY cell index for fast vertical ray casting."""

    def __init__(self, vertices: Sequence[Vec3], name: str = "", cell_mm: float = 6.0) -> None:
        if len(vertices) % 3:
            raise ValueError("vertex count must be a multiple of three")
        self.name = name
        self.verts: List[Vec3] = list(vertices)
        self.cell = cell_mm
        self.tris: List[Tri] = [
            (self.verts[i], self.verts[i + 1], self.verts[i + 2])
            for i in range(0, len(self.verts), 3)
        ]
        self._index: Dict[Tuple[int, int], List[int]] = {}
        self._global: List[int] = []
        self._build_index()

    def _build_index(self) -> None:
        limit = 400          # a triangle covering more cells than this is tested every time
        for idx, tri in enumerate(self.tris):
            xs = (tri[0][0], tri[1][0], tri[2][0])
            ys = (tri[0][1], tri[1][1], tri[2][1])
            x0, x1 = min(xs), max(xs)
            y0, y1 = min(ys), max(ys)
            i0, i1 = int(math.floor(x0 / self.cell)), int(math.floor(x1 / self.cell))
            j0, j1 = int(math.floor(y0 / self.cell)), int(math.floor(y1 / self.cell))
            if (i1 - i0 + 1) * (j1 - j0 + 1) > limit:
                self._global.append(idx)
                continue
            for i in range(i0, i1 + 1):
                for j in range(j0, j1 + 1):
                    self._index.setdefault((i, j), []).append(idx)

    def _candidates(self, x: float, y: float) -> Iterable[Tri]:
        key = (int(math.floor(x / self.cell)), int(math.floor(y / self.cell)))
        for idx in self._index.get(key, ()):
            yield self.tris[idx]
        for idx in self._global:
            yield self.tris[idx]

    def point_in_solid(self, point: Vec3, nudge: bool = True) -> bool:
        """True if the point lies inside the UNION of the primitives.

        Casts a ray along +Z and counts crossings of the horizontal faces above the point.
        Side walls project to a line segment in XY and are rejected by the strict
        point-in-triangle test, so they never contribute a spurious crossing.
        """
        px = point[0] + (RAY_NUDGE[0] if nudge else 0.0)
        py = point[1] + (RAY_NUDGE[1] if nudge else 0.0)
        pz = point[2]
        crossings = 0
        for a, b, c in self._candidates(px, py):
            # Signed area of (a,b,c) projected to XY; zero means the face is vertical.
            d = (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])
            if abs(d) < 1e-12:
                continue
            w0 = ((b[0] - px) * (c[1] - py) - (c[0] - px) * (b[1] - py)) / d
            w1 = ((c[0] - px) * (a[1] - py) - (a[0] - px) * (c[1] - py)) / d
            w2 = 1.0 - w0 - w1
            eps = 1e-9
            if w0 <= eps or w1 <= eps or w2 <= eps:
                continue                     # on or outside an edge: not a clean crossing
            z = w0 * a[2] + w1 * b[2] + w2 * c[2]
            if z > pz:
                crossings += 1
        return crossings % 2 == 1

    def first_hit(self, origin: Vec3, direction: Vec3, max_t: float = 500.0) -> float | None:
        """Distance to the nearest triangle the ray crosses, or None if it crosses nothing.

        Möller-Trumbore over the whole soup. Used to measure a cavity's inner faces by firing
        rays outward from its centre: the hit distance IS the wall position. Returns the
        smallest t along a NORMALISED `direction`, or None.
        """
        dx, dy, dz = direction
        mag = math.sqrt(dx * dx + dy * dy + dz * dz)
        if mag < 1e-12:
            raise ValueError("direction must be non-zero")
        dx, dy, dz = dx / mag, dy / mag, dz / mag
        best: float | None = None
        for a, b, c in self.tris:
            e1 = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
            e2 = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
            px = dy * e2[2] - dz * e2[1]
            py = dz * e2[0] - dx * e2[2]
            pz = dx * e2[1] - dy * e2[0]
            det = e1[0] * px + e1[1] * py + e1[2] * pz
            if abs(det) < 1e-12:
                continue
            inv = 1.0 / det
            tx = origin[0] - a[0]
            ty = origin[1] - a[1]
            tz = origin[2] - a[2]
            u = (tx * px + ty * py + tz * pz) * inv
            if u < 1e-9 or u > 1.0 - 1e-9:
                continue
            qx = ty * e1[2] - tz * e1[1]
            qy = tz * e1[0] - tx * e1[2]
            qz = tx * e1[1] - ty * e1[0]
            v = (dx * qx + dy * qy + dz * qz) * inv
            if v < 1e-9 or u + v > 1.0 - 1e-9:
                continue
            t = (e2[0] * qx + e2[1] * qy + e2[2] * qz) * inv
            if t > 1e-6 and t <= max_t and (best is None or t < best):
                best = t
        return best

    def wall_distance(self, origin: Vec3, direction: Vec3, max_t: float = 500.0):
        """Distance from an origin inside a cavity to the cavity wall along `direction`."""
        return self.first_hit(origin, direction, max_t)

    def union_volume_mm3(self, grid_mm: float = 4.0) -> float:
        """Solid volume of the UNION by voxel sampling of the point-in-solid test.

        This is the honest number: unlike summing primitive volumes it does not
        double-count overlaps, so it is what a slicer starts from (before infill).
        Accuracy is on the order of the voxel size at the surface.
        """
        if grid_mm <= 0:
            raise ValueError("grid_mm must be positive")
        (x0, y0, z0), (x1, y1, z1) = bounds(self.verts)
        nx = max(1, int(math.ceil((x1 - x0) / grid_mm)))
        ny = max(1, int(math.ceil((y1 - y0) / grid_mm)))
        nz = max(1, int(math.ceil((z1 - z0) / grid_mm)))
        voxel = ((x1 - x0) / nx) * ((y1 - y0) / ny) * ((z1 - z0) / nz)
        inside = 0
        for i in range(nx):
            x = x0 + (i + 0.5) * (x1 - x0) / nx
            for j in range(ny):
                y = y0 + (j + 0.5) * (y1 - y0) / ny
                for k in range(nz):
                    z = z0 + (k + 0.5) * (z1 - z0) / nz
                    if self.point_in_solid((x, y, z)):
                        inside += 1
        return inside * voxel

    def feature_centre(
        self,
        u_axis: int,
        v_axis: int,
        w_axis: int,
        centre_uv: Tuple[float, float],
        radius_window: Tuple[float, float],
        w_band: Tuple[float, float],
        min_vertices: int = 8,
    ) -> Tuple[float, float, int]:
        """Measure the true centre of a circular feature from its own vertices.

        Chooses vertices whose (u, v) distance from the declared centre sits inside
        `radius_window`, that live within `w_band` along the feature's axis, and returns the
        centroid of that set in (u, v). For a full ring of vertices the centroid IS the
        feature's centre, so this measures the model rather than restating it.
        """
        total_u = total_v = 0.0
        count = 0
        for p in self.verts:
            if not (w_band[0] <= p[w_axis] <= w_band[1]):
                continue
            du = p[u_axis] - centre_uv[0]
            dv = p[v_axis] - centre_uv[1]
            r = math.hypot(du, dv)
            if radius_window[0] <= r <= radius_window[1]:
                total_u += p[u_axis]
                total_v += p[v_axis]
                count += 1
        if count < min_vertices:
            return (float("nan"), float("nan"), count)
        return (total_u / count, total_v / count, count)


    def fit_circle(
        self,
        u_axis: int,
        v_axis: int,
        w_axis: int,
        approx_uv: Tuple[float, float],
        radius_window: Tuple[float, float],
        w_band: Tuple[float, float],
        min_vertices: int = 8,
    ):
        """Least-squares circle through a bore's own vertices.

        Returns (centre_u, centre_v, radius, rms_residual, count). The centroid of the vertex
        set is NOT the centre: a hole that was cut by half-plane clipping has extra vertices
        where the cut crosses the bounding box, so the angular distribution is uneven. A
        circle fit is immune to that. Points that lie exactly on a circle recover its centre
        to machine precision, which is the point of this measurement.
        """
        pts: List[Tuple[float, float]] = []
        for p in self.verts:
            if not (w_band[0] <= p[w_axis] <= w_band[1]):
                continue
            du = p[u_axis] - approx_uv[0]
            dv = p[v_axis] - approx_uv[1]
            if radius_window[0] <= math.hypot(du, dv) <= radius_window[1]:
                pts.append((p[u_axis], p[v_axis]))
        n = len(pts)
        if n < min_vertices:
            return (float("nan"), float("nan"), float("nan"), float("nan"), n)
        # Kasa fit: minimise sum((x^2 + y^2) + D x + E y + F)^2
        sx = sy = sxx = syy = sxy = sxz = syz = sz = 0.0
        for x, y in pts:
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
        rms = math.sqrt(sum((math.hypot(x - cu, y - cv) - rr) ** 2 for x, y in pts) / n)
        return (cu, cv, rr, rms, n)


def load_mesh(path, cell_mm: float = 6.0) -> Mesh:
    _, verts = read_binary_stl(path)
    return Mesh(verts, name=path.name, cell_mm=cell_mm)
