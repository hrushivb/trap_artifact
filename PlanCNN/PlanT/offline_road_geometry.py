#!/usr/bin/env python3
"""
Synthetic two-lane road: center reference + left/right lane centerlines (parallel offset).
Ego advances by arc length on one lane; route waypoints (20,2) are sampled in ego frame ahead on that lane.

Default geometry is a **straight** centerline (two parallel lane centerlines). Optional curved layout
uses ``build_centerline_straight_then_arc`` when ``TwoLaneRoad.build(..., curved=True)``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np


def _unit(v: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / max(n, eps)


def _cumlen(points: np.ndarray) -> np.ndarray:
    """Cumulative arc length along polyline (N,), first element 0."""
    d = np.linalg.norm(np.diff(points, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(d)])


def _interp_pose(points: np.ndarray, cum: np.ndarray, s: float) -> Tuple[np.ndarray, float]:
    """Position and heading (yaw, radians) at arc length s along polyline."""
    s = float(np.clip(s, 0.0, cum[-1]))
    idx = int(np.searchsorted(cum, s, side="right") - 1)
    idx = max(0, min(idx, len(points) - 2))
    s0, s1 = cum[idx], cum[idx + 1]
    t = 0.0 if s1 <= s0 else (s - s0) / (s1 - s0)
    p = (1 - t) * points[idx] + t * points[idx + 1]
    tang = points[idx + 1] - points[idx]
    yaw = float(np.arctan2(tang[1], tang[0]))
    return p.astype(np.float64), yaw


def _offset_polyline(points: np.ndarray, half_width: float, side: int) -> np.ndarray:
    """
    Offset polyline perpendicular to segments. side=+1 -> one lateral direction, -1 opposite.
    Left of forward tangent (-ty, tx) uses side sign to pick lane.
    """
    n = len(points)
    out = np.zeros_like(points, dtype=np.float64)
    for i in range(n):
        if i < n - 1:
            t = _unit(points[i + 1] - points[i])
        else:
            t = _unit(points[i] - points[i - 1])
        # left normal
        nx, ny = -t[1], t[0]
        if side < 0:
            nx, ny = -nx, -ny
        out[i] = points[i] + half_width * np.array([nx, ny], dtype=np.float64)
    return out


def build_centerline_straight_then_arc(
    straight_m: float = 45.0,
    turn_deg: float = 55.0,
    radius_m: float = 32.0,
    straight2_m: float = 80.0,
    ds: float = 0.5,
) -> np.ndarray:
    """Piecewise: +x straight, circular arc (left turn), then straight in new direction."""
    pts: List[np.ndarray] = []
    x0 = 0.0
    y0 = 0.0
    # segment 1: along +x
    n1 = max(2, int(straight_m / ds) + 1)
    for i in range(n1):
        t = i / (n1 - 1)
        pts.append(np.array([x0 + t * straight_m, y0], dtype=np.float64))
    # arc: center at (x_c, y_c), start from last point, tangent +x
    p1 = pts[-1]
    cx = p1[0]
    cy = p1[1] + radius_m  # center above road so we turn left (CCW)
    theta0 = -np.pi / 2  # from center to start point: angle down
    n_arc = max(8, int(np.radians(abs(turn_deg)) * radius_m / ds) + 1)
    for i in range(1, n_arc + 1):
        ang = theta0 + (np.radians(turn_deg) * i / n_arc)
        pts.append(np.array([cx + radius_m * np.cos(ang), cy + radius_m * np.sin(ang)], dtype=np.float64))
    # segment 3: straight continuation from arc end (tangent = CCW derivative of circle)
    p2 = pts[-1]
    theta_end = theta0 + np.radians(turn_deg)
    tang = _unit(np.array([-radius_m * np.sin(theta_end), radius_m * np.cos(theta_end)], dtype=np.float64))
    n3 = max(2, int(straight2_m / ds) + 1)
    for i in range(1, n3):
        pts.append(p2 + tang * (i * straight2_m / (n3 - 1)))
    return np.array(pts, dtype=np.float64)


def build_centerline_straight(length_m: float = 220.0, ds: float = 0.5) -> np.ndarray:
    """Straight segment along +x from (0,0) to (length_m, 0); used for two-lane straight roads."""
    length_m = float(max(length_m, ds * 2))
    n = max(2, int(length_m / ds) + 1)
    xs = np.linspace(0.0, length_m, n, dtype=np.float64)
    ys = np.zeros_like(xs, dtype=np.float64)
    return np.stack([xs, ys], axis=1)


@dataclass
class TwoLaneRoad:
    """Center reference + two lane centerlines (world xy)."""

    center: np.ndarray
    lane_width_m: float
    lane_right: np.ndarray  # offset one way
    lane_left: np.ndarray  # offset other way
    cum_center: np.ndarray

    @classmethod
    def build(
        cls,
        lane_width_m: float = 3.6,
        *,
        curved: bool = False,
        straight_length_m: float = 220.0,
        ds: float = 0.5,
        **arc_kw,
    ) -> TwoLaneRoad:
        if curved:
            arc_kw.setdefault("ds", ds)
            center = build_centerline_straight_then_arc(**arc_kw)
        else:
            center = build_centerline_straight(straight_length_m, ds=ds)
        hw = 0.5 * lane_width_m
        # right lane = center + offset to the right of forward (CW from tangent)
        lane_r = _offset_polyline(center, hw, side=-1)
        lane_l = _offset_polyline(center, hw, side=+1)
        return cls(
            center=center,
            lane_width_m=lane_width_m,
            lane_right=lane_r,
            lane_left=lane_l,
            cum_center=_cumlen(center),
        )

    def lane_polyline(self, lane_id: int) -> np.ndarray:
        """0 = right of centerline (+y offset negated in _offset), 1 = left lane."""
        return self.lane_right if int(lane_id) == 0 else self.lane_left

    def lane_cumlen(self, lane_id: int) -> np.ndarray:
        return _cumlen(self.lane_polyline(int(lane_id)))

    def pose_on_lane(self, lane_id: int, s: float) -> Tuple[np.ndarray, float]:
        poly = self.lane_polyline(lane_id)
        cum = _cumlen(poly)
        return _interp_pose(poly, cum, s)

    def advance_s(self, lane_id: int, s: float, ds: float) -> float:
        cum = self.lane_cumlen(lane_id)
        return float(min(s + ds, cum[-1] - 1e-3))


def project_point_to_lane_arc_length(road: TwoLaneRoad, lane_id: int, q_xy: np.ndarray) -> float:
    """Closest arc length ``s`` on ``lane_id`` centerline to world point ``q_xy`` (piecewise-linear)."""
    poly = road.lane_polyline(int(lane_id))
    cum = _cumlen(poly)
    q_xy = np.asarray(q_xy, dtype=np.float64).reshape(2)
    n = len(poly)
    best_s = float(cum[0])
    best_d2 = 1e30
    for i in range(n - 1):
        a = poly[i].astype(np.float64)
        b = poly[i + 1].astype(np.float64)
        ab = b - a
        denom = float(np.dot(ab, ab)) + 1e-12
        t = float(np.dot(q_xy - a, ab)) / denom
        t = max(0.0, min(1.0, t))
        proj = a + t * ab
        d2 = float(np.sum((q_xy - proj) ** 2))
        if d2 < best_d2:
            best_d2 = d2
            seg_len = float(np.linalg.norm(ab))
            best_s = float(cum[i] + t * seg_len)
    return best_s


def world_to_ego_points(world_xy: np.ndarray, ego_xy: np.ndarray, ego_yaw: float) -> np.ndarray:
    """(M,2) world -> (M,2) ego frame (x forward, y left)."""
    d = world_xy - ego_xy.reshape(1, 2)
    c, s = np.cos(ego_yaw), np.sin(ego_yaw)
    x_e = d[:, 0] * c + d[:, 1] * s
    y_e = -d[:, 0] * s + d[:, 1] * c
    return np.stack([x_e, y_e], axis=1).astype(np.float32)


def lane_markings_world_polylines(road: TwoLaneRoad) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Road edges and inter-lane line in world frame: right outer, median (between lane centers),
    left outer. Each polyline follows the same arc-length stations as ``road.center``.
    """
    hw = 0.5 * road.lane_width_m
    right_outer = _offset_polyline(road.lane_right, hw, side=-1)
    median = np.asarray(road.center, dtype=np.float64)
    left_outer = _offset_polyline(road.lane_left, hw, side=+1)
    return right_outer, median, left_outer


def lane_markings_ego_jsonable(road: TwoLaneRoad, ego_xy: np.ndarray, ego_yaw: float) -> List[Dict[str, object]]:
    """Lane / road boundaries in ego frame as JSON-friendly dicts for ``visualize_scene``."""
    ro, med, lo = lane_markings_world_polylines(road)
    rows: List[Tuple[np.ndarray, str, str, float, str]] = [
        (ro, "0.22", "-", 2.7, "road edge"),
        (med, "#a67c00", "--", 2.3, "lane divider"),
        (lo, "0.22", "-", 2.7, "_nolegend_"),
    ]
    out: List[Dict[str, object]] = []
    for poly, color, ls, lw, lab in rows:
        e = world_to_ego_points(poly, ego_xy, ego_yaw)
        if e.shape[0] >= 2:
            out.append(
                {
                    "points": np.asarray(e, dtype=np.float64).tolist(),
                    "color": color,
                    "linestyle": ls,
                    "linewidth": lw,
                    "label": lab,
                }
            )
    return out


def sample_route_ego_frame(
    road: TwoLaneRoad,
    lane_id: int,
    ego_xy: np.ndarray,
    ego_yaw: float,
    s_ego: float,
    n: int = 20,
    spacing_m: float = 2.0,
) -> np.ndarray:
    """
    Sample n points along lane starting at arc length s_ego, spacing_m apart, in ego frame.
    """
    poly = road.lane_polyline(lane_id)
    cum = road.lane_cumlen(lane_id)
    s_max = cum[-1]
    pts_world = []
    for i in range(n):
        si = s_ego + i * spacing_m
        if si > s_max:
            si = s_max
        p, _ = _interp_pose(poly, cum, si)
        pts_world.append(p)
    pw = np.array(pts_world, dtype=np.float64)
    return world_to_ego_points(pw, ego_xy, ego_yaw)


def yaw_on_polyline(poly: np.ndarray, q: np.ndarray) -> float:
    """Tangent yaw (world rad) at segment closest to q."""
    d2 = np.sum((poly - q.reshape(1, 2)) ** 2, axis=1)
    i = int(np.argmin(d2))
    i = min(max(i, 0), len(poly) - 2)
    t = poly[i + 1] - poly[i]
    return float(np.arctan2(t[1], t[0]))


def yaw_nearest_lane(road: TwoLaneRoad, q: np.ndarray) -> float:
    """Road tangent at closest point on either lane centerline."""
    best_d = 1e18
    best_y = 0.0
    for lid in (0, 1):
        poly = road.lane_polyline(lid)
        d2 = np.sum((poly - q.reshape(1, 2)) ** 2, axis=1)
        m = float(np.min(d2))
        if m < best_d:
            best_d = m
            best_y = yaw_on_polyline(poly, q)
    return best_y


def wrap_angle(a: float) -> float:
    return float(np.arctan2(np.sin(a), np.cos(a)))


def other_lane_window_ego(
    road: TwoLaneRoad,
    ego_lane: int,
    ego_xy: np.ndarray,
    ego_yaw: float,
    s_center: float,
    half_window_m: float = 35.0,
    ds: float = 1.2,
) -> np.ndarray:
    other = 1 - int(ego_lane)
    poly = road.lane_polyline(other)
    cum = _cumlen(poly)
    s_lo = max(cum[0], s_center - half_window_m)
    s_hi = min(cum[-1], s_center + half_window_m)
    ts = np.arange(s_lo, s_hi, ds, dtype=np.float64)
    if len(ts) < 2:
        ts = np.array([s_lo, min(s_hi, s_lo + 1.0)], dtype=np.float64)
    pts = []
    for s in ts:
        p, _ = _interp_pose(poly, cum, float(s))
        pts.append(p)
    pw = np.array(pts, dtype=np.float64)
    return world_to_ego_points(pw, ego_xy, ego_yaw)
