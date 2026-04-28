#!/usr/bin/env python3
"""
Visualize BEV tensor (optional) and ego-frame route / detections / predicted trajectory.

  # Output from offline_inference.py (recommended)
  python offline_inference.py --checkpoint model.ckpt --example scene.json --save-results run.json --viz-out run.png
  python visualize_scene.py --offline-results run.json -o run.png

  # Or split inputs
  python visualize_scene.py --scene-json examples/offline_sample.json --pred-json preds_only.json

Inputs use the **vehicle ego frame**: x forward (m), y left (m), origin at the ego. By default the plot uses
**world-aligned axes** at the ego (axes do not spin when ``ego_yaw_world_rad`` changes); the ego polygon rotates.
Set ``rotate_plot_with_vehicle_yaw=True`` for the legacy view where +x is always straight ahead of the car.
When ``center_ego_in_frame`` is true, axes default to ±50 m (100 m square); override with ``ego_plot_half_span_m``.

Optional ``lane_boundaries``: list of ``{"points": [[x,y],...], "color", "linestyle", "linewidth", "label"}``
for road edges and lane divider (see ``offline_road_geometry.lane_markings_ego_jsonable``).
Optional ``goal_ego_xy``: ``[x, y]`` goal point in ego frame (gold star).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np

_PLANT_DIR = Path(__file__).resolve().parent
if str(_PLANT_DIR) not in sys.path:
    sys.path.insert(0, str(_PLANT_DIR))

# Keys written by offline_inference.py alongside the input scene
_PREDICTION_KEYS = frozenset(
    {
        "pred_path",
        "pred_wps",
        "pred_speed_logits",
        "pred_speed_probs",
        "pred_speed_scalar",
    }
)

# CARLA-style half-extents [length, width, height]; matches typical offline obstacle box.
_DEFAULT_VEHICLE_EXTENT = [2.0, 1.0, 0.5]


def _ego_xy_to_world_aligned_at_ego(xy: np.ndarray, ego_yaw_world_rad: float) -> np.ndarray:
    """Rotate ego-frame points into world-aligned coordinates (origin still ego)."""
    a = np.asarray(xy, dtype=np.float64)
    if a.size == 0:
        return a
    if a.ndim == 1:
        a = a.reshape(1, -1)
    c = math.cos(float(ego_yaw_world_rad))
    s = math.sin(float(ego_yaw_world_rad))
    x_e = a[..., 0]
    y_e = a[..., 1]
    x_w = c * x_e - s * y_e
    y_w = s * x_e + c * y_e
    return np.stack([x_w, y_w], axis=-1)


def _world_aligned_scene_and_pred(
    scene: Dict[str, Any],
    pred: Optional[Dict[str, Any]],
    ego_yaw_world_rad: float,
) -> Tuple[Dict[str, Any], Optional[Dict[str, Any]]]:
    """Copy scene + pred with polylines/boxes rotated from ego frame to world-aligned frame at ego."""
    sc: Dict[str, Any] = dict(scene)
    psi = float(ego_yaw_world_rad)

    rt = np.asarray(sc.get("route", []), dtype=np.float64)
    if rt.ndim == 2 and rt.shape[0] > 0:
        sc["route"] = _ego_xy_to_world_aligned_at_ego(rt, psi).tolist()

    objs_out: list = []
    for obj in sc.get("objects", []) or []:
        o = dict(obj)
        pos = np.array(o.get("position", [0.0, 0.0, 0.0]), dtype=np.float64)
        if pos.shape[0] >= 2:
            pr = _ego_xy_to_world_aligned_at_ego(pos[:2].reshape(1, 2), psi).reshape(2)
            pos = np.concatenate([pr, pos[2:]])
        o["position"] = pos.tolist() if hasattr(pos, "tolist") else list(pos)
        if "yaw" in o:
            o["yaw"] = float(math.atan2(math.sin(float(o["yaw"]) + psi), math.cos(float(o["yaw"]) + psi)))
        objs_out.append(o)
    sc["objects"] = objs_out

    extras: list = []
    for poly in sc.get("extra_polylines", []) or []:
        pa = np.asarray(poly, dtype=np.float64)
        if pa.ndim == 2 and pa.shape[0] >= 2:
            extras.append(_ego_xy_to_world_aligned_at_ego(pa, psi).tolist())
        else:
            extras.append(poly)
    sc["extra_polylines"] = extras

    lbs: list = []
    for lb in sc.get("lane_boundaries", []) or []:
        if isinstance(lb, dict) and lb.get("points") is not None:
            d = dict(lb)
            pts = np.asarray(d["points"], dtype=np.float64)
            if pts.ndim == 2 and pts.shape[0] >= 2:
                d["points"] = _ego_xy_to_world_aligned_at_ego(pts, psi).tolist()
            lbs.append(d)
        else:
            lbs.append(lb)
    sc["lane_boundaries"] = lbs

    if sc.get("goal_ego_xy") is not None:
        g = np.asarray(sc["goal_ego_xy"], dtype=np.float64).reshape(-1)
        if g.size >= 2:
            g2 = _ego_xy_to_world_aligned_at_ego(g[:2].reshape(1, 2), psi).reshape(2)
            sc["goal_ego_xy"] = [float(g2[0]), float(g2[1])]

    sc["ego_yaw"] = psi

    pr_out: Optional[Dict[str, Any]] = None
    if pred:
        pr_out = dict(pred)
        if pr_out.get("pred_path") is not None:
            pp = np.asarray(pr_out["pred_path"], dtype=np.float64).squeeze()
            if pp.ndim == 3:
                pp = pp[0]
            if pp.ndim == 2 and pp.shape[0] > 0 and pp.shape[1] >= 2:
                pr_out["pred_path"] = _ego_xy_to_world_aligned_at_ego(pp, psi).tolist()
        if pr_out.get("pred_wps") is not None:
            pw = np.asarray(pr_out["pred_wps"], dtype=np.float64).squeeze()
            if pw.ndim == 3:
                pw = pw[0]
            if pw.ndim == 2 and pw.shape[0] > 0 and pw.shape[1] >= 2:
                pr_out["pred_wps"] = _ego_xy_to_world_aligned_at_ego(pw, psi).tolist()
    return sc, pr_out


def _ego_extent_for_scene(scene: Dict[str, Any]) -> list:
    ext = scene.get("ego_extent")
    if ext is not None and len(ext) >= 2:
        return list(ext)
    objs = scene.get("objects") or []
    if objs and isinstance(objs[0], dict) and objs[0].get("extent"):
        return list(objs[0]["extent"])
    return list(_DEFAULT_VEHICLE_EXTENT)


def _ego_vehicle_polygon_xy(scene: Dict[str, Any]) -> np.ndarray:
    """Ego at origin in ego frame; optional ``ego_yaw`` (rad); extent like obstacle ``extent``."""
    ext = _ego_extent_for_scene(scene)
    yaw = float(scene.get("ego_yaw", 0.0))
    return _box_polygon_xy(np.array([0.0, 0.0, 0.0]), yaw, ext)


def _box_polygon_xy(position: np.ndarray, yaw: float, extent: list) -> np.ndarray:
    """extent: [half_len, half_wid, half_h] in CARLA style; yaw in rad, +x forward."""
    hl, hw = float(extent[0]), float(extent[1])
    c, s = np.cos(yaw), np.sin(yaw)
    corners = np.array([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw], [hl, hw]])
    rot = np.array([[c, -s], [s, c]])
    return position[:2] + (rot @ corners.T).T


def _bev_to_rgb(bev: np.ndarray) -> np.ndarray:
    """(3, H, W) float -> (H, W, 3) uint8 for imshow."""
    x = np.asarray(bev, dtype=np.float32)
    if x.ndim != 3 or x.shape[0] != 3:
        raise ValueError(f"Expected BEV shape (3, H, W), got {x.shape}")
    x = np.transpose(x, (1, 2, 0))
    lo, hi = float(np.nanmin(x)), float(np.nanmax(x))
    if hi - lo < 1e-6:
        return np.zeros((*x.shape[:2], 3), dtype=np.uint8)
    if lo >= 0.0 and hi <= 1.0:
        return (np.clip(x, 0, 1) * 255).astype(np.uint8)
    # imagenet-normalized or arbitrary: stretch to 0..255
    x = (x - lo) / (hi - lo + 1e-8)
    return (np.clip(x, 0, 1) * 255).astype(np.uint8)


def plot_ego_scene(
    ax,
    scene: Dict[str, Any],
    pred: Optional[Dict[str, Any]],
    title: str = "Ego frame (x forward, y left)",
    *,
    center_ego_in_frame: bool = True,
    ego_plot_half_span_m: float = 50.0,
    rotate_plot_with_vehicle_yaw: bool = False,
) -> None:
    psi = float(scene.get("ego_yaw_world_rad", scene.get("ego_yaw", 0.0)))
    if not rotate_plot_with_vehicle_yaw:
        scene, pred = _world_aligned_scene_and_pred(scene, pred, psi)

    for lb in scene.get("lane_boundaries", []):
        if not isinstance(lb, dict):
            continue
        pts = np.asarray(lb.get("points", []), dtype=np.float64)
        if pts.ndim != 2 or pts.shape[0] < 2 or pts.shape[1] < 2:
            continue
        ax.plot(
            pts[:, 0],
            pts[:, 1],
            color=lb.get("color", "0.35"),
            linestyle=lb.get("linestyle", "-"),
            linewidth=float(lb.get("linewidth", 2.6)),
            label=str(lb.get("label", "_nolegend_")),
            zorder=1,
            solid_capstyle="round",
        )

    route = np.asarray(scene["route"], dtype=np.float64)
    if route.shape[0] > 0:
        ax.plot(route[:, 0], route[:, 1], "b-", lw=1.05, label="route (plan)", zorder=3, alpha=0.9)
        ax.scatter(route[:, 0], route[:, 1], c="royalblue", s=18, zorder=5, linewidths=0.5, edgecolors="midnightblue")

    ego_poly = _ego_vehicle_polygon_xy(scene)
    ax.plot(ego_poly[:, 0], ego_poly[:, 1], "k-", lw=1.2, label="ego", zorder=9)
    ax.fill(ego_poly[:, 0], ego_poly[:, 1], alpha=0.22, color="0.35", zorder=8)

    gxy = scene.get("goal_ego_xy")
    if gxy is not None and len(gxy) >= 2:
        gx, gy = float(gxy[0]), float(gxy[1])
        if math.isfinite(gx) and math.isfinite(gy):
            ax.scatter(
                [gx],
                [gy],
                c="gold",
                s=160,
                zorder=12,
                marker="*",
                edgecolors="darkgoldenrod",
                linewidths=0.9,
                label="goal",
            )

    for i, obj in enumerate(scene.get("objects", [])):
        pos = np.array(obj["position"][:2], dtype=np.float64)
        yaw = float(obj["yaw"])
        ext = obj["extent"]
        poly = _box_polygon_xy(np.array([pos[0], pos[1], 0.0]), yaw, ext)
        ax.plot(poly[:, 0], poly[:, 1], "r-", lw=1.2)
        ax.fill(poly[:, 0], poly[:, 1], alpha=0.15, color="red")
        ax.text(pos[0], pos[1], str(obj.get("class", "?"))[:4], fontsize=7, ha="center")

    for ei, poly in enumerate(scene.get("extra_polylines", [])):
        pa = np.asarray(poly, dtype=np.float64)
        if pa.ndim == 2 and pa.shape[0] >= 2 and pa.shape[1] >= 2:
            lbl = "other lane" if ei == 0 else "_nolegend_"
            ax.plot(pa[:, 0], pa[:, 1], "m--", lw=2.2, alpha=0.85, label=lbl)

    if pred:
        if "pred_path" in pred and pred["pred_path"] is not None:
            pp = np.asarray(pred["pred_path"], dtype=np.float64).squeeze()
            if pp.ndim == 2 and pp.shape[1] >= 2 and pp.shape[0] > 0:
                ax.plot(pp[:, 0], pp[:, 1], "g-", lw=3.0, label="pred path", solid_capstyle="round")
                ax.scatter(pp[:, 0], pp[:, 1], c="limegreen", s=32, zorder=4, linewidths=0.45, edgecolors="darkgreen")
        if "pred_wps" in pred and pred["pred_wps"] is not None:
            pw = np.asarray(pred["pred_wps"], dtype=np.float64).squeeze()
            if pw.ndim == 2 and pw.shape[1] >= 2 and pw.shape[0] > 0:
                ax.plot(pw[:, 0], pw[:, 1], "orange", lw=2.4, marker="o", ms=7, label="pred wps")

    if pred and pred.get("pred_speed_scalar") is not None:
        ax.text(
            0.02,
            0.98,
            f"pred speed (scalar): {float(pred['pred_speed_scalar']):.2f} m/s",
            transform=ax.transAxes,
            va="top",
            fontsize=9,
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
        )

    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    if rotate_plot_with_vehicle_yaw:
        ax.set_xlabel("x forward (m)")
        ax.set_ylabel("y left (m)")
    else:
        ax.set_xlabel("x (m, world-aligned at ego)")
        ax.set_ylabel("y (m, world-aligned at ego)")
    ax.set_title(title)
    if center_ego_in_frame:
        span = float(abs(ego_plot_half_span_m))
        if span <= 0:
            span = 50.0
        ax.set_xlim(-span, span)
        ax.set_ylim(-span, span)
    ax.legend(loc="upper right", fontsize=8)


def split_scene_and_predictions(obj: Dict[str, Any]) -> Tuple[Dict[str, Any], Optional[Dict[str, Any]]]:
    """Split a merged offline_inference result JSON into scene inputs and prediction dict."""
    pred = {k: obj[k] for k in _PREDICTION_KEYS if k in obj}
    scene = {k: v for k, v in obj.items() if k not in _PREDICTION_KEYS}
    return scene, (pred if pred else None)


def save_offline_results_figure(
    merged: Dict[str, Any],
    output_path: Path,
    title: str = "",
) -> None:
    """Save one PNG: route, objects, pred_path / pred_wps from offline_inference output."""
    scene, pred = split_scene_and_predictions(merged)
    fig, ax = plt.subplots(1, 1, figsize=(8, 8))
    plot_ego_scene(ax, scene, pred, title="Offline inference (ego frame)")
    if title:
        fig.suptitle(title, fontsize=11)
    fig.tight_layout()
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    p = argparse.ArgumentParser(description="Visualize BEV + ego trajectory / boxes")
    p.add_argument(
        "--offline-results",
        type=str,
        default="",
        help="Single JSON from offline_inference --save-results (scene + pred_path / pred_wps / ...)",
    )
    p.add_argument("--scene-json", type=str, default="", help="Scene JSON (route + objects); not needed if --offline-results set")
    p.add_argument("--bev-npy", type=str, default="", help="Optional (3,H,W) BEV tensor saved as .npy")
    p.add_argument("--pred-json", type=str, default="", help="Predictions only (pred_path, pred_wps); not needed if --offline-results set")
    p.add_argument("-o", "--output", type=str, default="", help="Save figure to PNG (e.g. scene.png)")
    p.add_argument("--show", action="store_true", help="Show interactive window")
    args = p.parse_args()

    if args.offline_results:
        merged_path = Path(args.offline_results)
        with open(merged_path, "r", encoding="utf-8") as f:
            merged = json.load(f)
        scene, pred = split_scene_and_predictions(merged)
        scene_path = merged_path
    else:
        if not args.scene_json:
            p.error("Provide --offline-results PATH or --scene-json PATH")
        scene_path = Path(args.scene_json)
        with open(scene_path, "r", encoding="utf-8") as f:
            scene = json.load(f)
        pred = None
        if args.pred_json:
            with open(args.pred_json, "r", encoding="utf-8") as f:
                pred = json.load(f)

    has_bev = bool(args.bev_npy and Path(args.bev_npy).is_file())
    ncols = 2 if has_bev else 1
    fig, axes = plt.subplots(1, ncols, figsize=(7 * ncols, 6))
    if ncols == 1:
        axes = [axes]

    if has_bev:
        bev = np.load(args.bev_npy)
        rgb = _bev_to_rgb(bev)
        axes[0].imshow(rgb, origin="lower")
        axes[0].set_title("BEV tensor (RGB channels)")
        axes[0].axis("off")
        plot_ego_scene(axes[1], scene, pred, title="Ego frame: route & objects")
    else:
        plot_ego_scene(axes[0], scene, pred)

    fig.suptitle(scene_path.name, fontsize=10)
    fig.tight_layout()

    if args.output:
        out = Path(args.output)
        out.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(out, dpi=150, bbox_inches="tight")
        print(f"Saved {out.resolve()}")

    if args.show or not args.output:
        plt.show()
    else:
        plt.close(fig)


if __name__ == "__main__":
    main()
