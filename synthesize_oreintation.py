"""
synthesize_wall_kitti_obj.py
-----------------------------
Synthesise a flat wall into KITTI *object-detection* dataset point clouds.

KITTI object dataset folder layout expected
-------------------------------------------
<dataset_root>/
    calib/          000000.txt  000001.txt  ...
    label_2/        000000.txt  000001.txt  ...   (optional – for reference)
    velodyne/       000000.bin  000001.bin  ...
    image_2/        000000.png  000001.png  ...   (left  colour camera)
    image_3/        000000.png  000001.png  ...   (right colour camera)

KITTI object calib file format (single file per frame)
-------------------------------------------------------
P0: <12 values>   – projection matrix cam0 (left  grey)
P1: <12 values>   – projection matrix cam1 (right grey)
P2: <12 values>   – projection matrix cam2 (left  colour)  ← used here
P3: <12 values>   – projection matrix cam3 (right colour)
R0_rect: <9 values>   – rectification rotation (same for all cameras)
Tr_velo_to_cam: <12 values>  – rigid transform velodyne → cam0 unrectified
Tr_imu_to_velo: <12 values>  – (not used here)

All P matrices already bake in the focal length AND the baseline offset, so:
  baseline  b = (P2[0,3] - P3[0,3]) / P2[0,0]    (always positive, ~0.54 m)
  depth     Z = f * b / disparity

Pipeline (per frame)
---------------------
  left-image pixel bbox  +  constant disparity
            │
            ▼
  Back-project → 3-D points in LEFT-RECTIFIED camera frame (cam2 rect)
            │
            ▼
  cam2_rect → cam0_unrect     pts_unrect = R0_rect⁻¹ · pts_rect
            │
            ▼
  cam0_unrect → velodyne       pts_velo = Tr_velo_to_cam⁻¹ · pts_unrect_h
            │
            ▼
  Merge with original .bin  →  save
"""

import argparse
import os
import sys
from pathlib import Path

import numpy as np

# Open3D is optional (only for --visualise)
try:
    import open3d as o3d
    HAS_O3D = True
except ImportError:
    HAS_O3D = False


# ─────────────────────────────────────────────────────────────────────────────
# 1.  KITTI object-format calibration loader
# ─────────────────────────────────────────────────────────────────────────────

class KITTIObjCalib:
    """
    Loads a single KITTI object-detection calibration file and exposes the
    matrices needed for the velodyne ↔ camera coordinate transforms.

    Attributes
    ----------
    P2          (3, 4)  – projection matrix left  colour cam (cam2)
    P3          (3, 4)  – projection matrix right colour cam (cam3)
    R0_rect     (3, 3)  – common rectification rotation
    Tr_velo2cam (3, 4)  – rigid velodyne → cam0_unrectified  [R|t]
    """

    def __init__(self, calib_path: str):
        raw = self._parse(calib_path)

        self.P0         = raw["P0"].reshape(3, 4)
        self.P1         = raw["P1"].reshape(3, 4)
        self.P2         = raw["P2"].reshape(3, 4)   # left colour camera
        self.P3         = raw["P3"].reshape(3, 4)   # right colour camera
        self.R0_rect    = raw["R0_rect"].reshape(3, 3)
        self.Tr_velo2cam = raw["Tr_velo_to_cam"].reshape(3, 4)  # [R|t]

        # Derived helpers
        self.fx    = self.P2[0, 0]
        self.fy    = self.P2[1, 1]
        self.cx    = self.P2[0, 2]
        self.cy    = self.P2[1, 2]

        # Baseline between cam2 and cam3 (metres, always positive ~0.54 m)
        # P2[0,3] = -fx * 0   = 0      (cam2 is the reference)
        # P3[0,3] = -fx * b            (cam3 is displaced by b to the right)
        self.baseline = (self.P2[0, 3] - self.P3[0, 3]) / self.fx

        # 4×4 homogeneous velodyne → cam0_unrect
        self._Tr_velo2cam_h = self._to_4x4(self.Tr_velo2cam)
        # 4×4 cam0_unrect → velodyne
        self._Tr_cam2velo_h = np.linalg.inv(self._Tr_velo2cam_h)

    # ------------------------------------------------------------------
    # public API
    # ------------------------------------------------------------------

    def depth_from_disparity(self, disparity: float) -> float:
        """Z = f * b / d"""
        if disparity <= 0:
            raise ValueError(f"Disparity must be > 0, got {disparity}")
        return self.fx * self.baseline / disparity

    def pixels_to_rect(
        self,
        us: np.ndarray,   # (N,) float  u pixel coords in left rectified image
        vs: np.ndarray,   # (N,) float  v pixel coords
        Z:  float,        # depth in metres (constant for flat wall)
    ) -> np.ndarray:      # (N, 3) float  in cam2_rect frame
        """
        Back-project pixel coordinates to 3-D points in the LEFT RECTIFIED
        camera (cam2_rect) frame.

            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy
        """
        X = (us - self.cx) * Z / self.fx
        Y = (vs - self.cy) * Z / self.fy
        return np.stack([X, Y, np.full_like(X, Z)], axis=-1)

    def rect_to_velo(self, pts_rect: np.ndarray) -> np.ndarray:
        """
        (N, 3) cam2_rect → (N, 3) velodyne

        cam2_rect  →  cam0_unrect  →  velodyne
        """
        # Step 1: cam2_rect → cam0_unrect
        #   pts_unrect = R0_rect⁻¹ · pts_rect
        R0_inv = self.R0_rect.T           # orthogonal matrix
        pts_unrect = (R0_inv @ pts_rect.T).T           # (N, 3)

        # Step 2: cam0_unrect → velodyne  (homogeneous)
        ones = np.ones((len(pts_unrect), 1), dtype=np.float64)
        pts_h = np.hstack([pts_unrect, ones])          # (N, 4)
        pts_velo = (self._Tr_cam2velo_h @ pts_h.T).T  # (N, 4)
        return pts_velo[:, :3]

    def velo_to_rect(self, pts_velo: np.ndarray) -> np.ndarray:
        """
        (N, 3) velodyne → (N, 3) cam2_rect   [inverse of rect_to_velo]
        Useful for projecting existing LiDAR into image for occlusion checks.
        """
        ones = np.ones((len(pts_velo), 1), dtype=np.float64)
        pts_h = np.hstack([pts_velo, ones])                    # (N, 4)
        pts_cam0 = (self._Tr_velo2cam_h @ pts_h.T).T[:, :3]   # (N, 3)
        pts_rect = (self.R0_rect @ pts_cam0.T).T               # (N, 3)
        return pts_rect

    def project_rect_to_image(self, pts_rect: np.ndarray) -> np.ndarray:
        """
        (N, 3) cam2_rect → (N, 2) pixel coords [u, v] in left image.
        Points behind the camera (Z ≤ 0) are included without filtering.
        """
        ones = np.ones((len(pts_rect), 1))
        pts_h = np.hstack([pts_rect, ones]).T       # (4, N)
        uvw = self.P2 @ pts_h                       # (3, N)
        uv  = (uvw[:2] / uvw[2]).T                  # (N, 2)
        return uv

    # ------------------------------------------------------------------
    # private helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _parse(path: str) -> dict:
        data = {}
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or ":" not in line:
                    continue
                key, vals = line.split(":", 1)
                data[key.strip()] = np.array(
                    [float(v) for v in vals.split()], dtype=np.float64
                )
        return data

    @staticmethod
    def _to_4x4(mat34: np.ndarray) -> np.ndarray:
        """Convert a (3, 4) rigid transform to (4, 4) homogeneous."""
        h = np.eye(4, dtype=np.float64)
        h[:3, :] = mat34
        return h

    def __repr__(self):
        return (
            f"KITTIObjCalib("
            f"fx={self.fx:.2f}, fy={self.fy:.2f}, "
            f"cx={self.cx:.2f}, cy={self.cy:.2f}, "
            f"baseline={self.baseline:.4f} m)"
        )


# ─────────────────────────────────────────────────────────────────────────────
# 2.  Wall synthesis
# ─────────────────────────────────────────────────────────────────────────────

def generate_wall_points(
    calib:      KITTIObjCalib,
    u_min:      float,
    u_max:      float,
    v_min:      float,
    v_max:      float,
    disparity:  float,
    density:    float = 0.5,       # points per pixel edge length
    reflectance: float = 0.5,
    add_noise_m: float = 0.0,      # optional Gaussian depth noise (metres)
) -> np.ndarray:
    """
    Generate (N, 4) wall points [X, Y, Z, intensity] in VELODYNE frame.

    Parameters
    ----------
    calib       : loaded KITTIObjCalib for this frame
    u_min/max   : left/right pixel column bounds (in left rectified image)
    v_min/max   : top/bottom pixel row bounds
    disparity   : constant disparity (pixels) of the flat wall
    density     : number of sample points per pixel width/height
                  (0.25 → 1 pt per 4 px, 1.0 → 1 pt per px, 2.0 → denser)
    reflectance : fake LiDAR intensity value [0, 1]
    add_noise_m : std-dev of Gaussian noise added to Z (0 = perfect flat wall)
    """
    # ── pixel grid ──────────────────────────────────────────────────────
    n_u = max(2, int((u_max - u_min) * density))
    n_v = max(2, int((v_max - v_min) * density))
    us_lin = np.linspace(u_min, u_max, n_u)
    vs_lin = np.linspace(v_min, v_max, n_v)
    uu, vv = np.meshgrid(us_lin, vs_lin)
    us = uu.ravel()
    vs = vv.ravel()

    # ── depth from disparity ─────────────────────────────────────────────
    Z = calib.depth_from_disparity(disparity)

    # ── back-project to rectified cam frame ──────────────────────────────
    pts_rect = calib.pixels_to_rect(us, vs, Z)   # (N, 3)

    # ── optional surface noise ────────────────────────────────────────────
    if add_noise_m > 0.0:
        noise = np.random.normal(0.0, add_noise_m, size=len(pts_rect))
        pts_rect[:, 2] += noise

    # ── transform to velodyne frame ───────────────────────────────────────
    pts_velo = calib.rect_to_velo(pts_rect)       # (N, 3)

    # ── append intensity column ───────────────────────────────────────────
    intensity = np.full((len(pts_velo), 1), reflectance, dtype=np.float32)
    cloud = np.hstack([pts_velo.astype(np.float32), intensity])
    return cloud


def remove_occluded_original_points(
    original:   np.ndarray,   # (N, 4) existing LiDAR cloud
    calib:      KITTIObjCalib,
    u_min:      float,
    u_max:      float,
    v_min:      float,
    v_max:      float,
    wall_depth: float,         # Z of the wall in rectified cam frame
) -> np.ndarray:
    """
    Optionally remove original LiDAR points that would be occluded by the
    synthesised wall (i.e., points that project inside the wall bbox AND
    are further away than the wall).

    Returns the filtered original cloud (N', 4).
    """
    pts_rect = calib.velo_to_rect(original[:, :3])        # (N, 3)
    uv       = calib.project_rect_to_image(pts_rect)       # (N, 2)

    in_bbox   = (
        (uv[:, 0] >= u_min) & (uv[:, 0] <= u_max) &
        (uv[:, 1] >= v_min) & (uv[:, 1] <= v_max)
    )
    behind_wall = pts_rect[:, 2] > wall_depth

    mask_remove = in_bbox & behind_wall
    n_removed   = mask_remove.sum()
    if n_removed:
        print(f"    [occlusion] removed {n_removed:,} original points behind the wall.")
    return original[~mask_remove]


# ─────────────────────────────────────────────────────────────────────────────
# 3.  KITTI label helpers  (optional – for reading existing annotations)
# ─────────────────────────────────────────────────────────────────────────────

def write_wall_label(
    label_path: str,
    u_min: float, u_max: float,
    v_min: float, v_max: float,
    calib: KITTIObjCalib,
    disparity: float,
    obj_type: str = "Wall",
):
    """
    Append a pseudo-label for the synthesised wall to an existing label file.
    Uses a thin 3-D box whose width/height match the back-projected wall size
    and whose depth is 0.1 m (paper-thin).

    The 3-D box is defined in the cam2_rect frame as KITTI expects:
        (h, w, l, x, y, z, ry)
    where (x,y,z) is the bottom-centre of the box and ry is the yaw.
    """
    Z = calib.depth_from_disparity(disparity)

    # Back-project corners to estimate 3-D box dimensions
    X_left  = (u_min - calib.cx) * Z / calib.fx
    X_right = (u_max - calib.cx) * Z / calib.fx
    Y_top   = (v_min - calib.cy) * Z / calib.fy
    Y_bot   = (v_max - calib.cy) * Z / calib.fy

    w  = abs(X_right - X_left)      # width  (horizontal extent)
    h  = abs(Y_bot   - Y_top)       # height (vertical extent)
    l  = 0.10                       # depth  (thin wall)

    x  = (X_left + X_right) / 2     # centre x
    y  = Y_bot                      # bottom of box (KITTI convention)
    z  = Z                          # depth of wall

    ry = 0.0                        # wall faces the camera (no yaw)
    alpha = 0.0

    # 2-D bounding box (truncated=0, occluded=0, score=-1 → no score)
    line = (
        f"{obj_type} 0.00 0 {alpha:.2f} "
        f"{u_min:.2f} {v_min:.2f} {u_max:.2f} {v_max:.2f} "
        f"{h:.4f} {w:.4f} {l:.4f} "
        f"{x:.4f} {y:.4f} {z:.4f} {ry:.4f}\n"
    )

    with open(label_path, "a") as f:
        f.write(line)
    print(f"    [label] appended Wall annotation → {label_path}")


# ─────────────────────────────────────────────────────────────────────────────
# 4.  I/O helpers
# ─────────────────────────────────────────────────────────────────────────────

def load_bin(path: str) -> np.ndarray:
    """KITTI .bin → (N, 4) float32  [X, Y, Z, intensity]."""
    return np.fromfile(path, dtype=np.float32).reshape(-1, 4)


def save_bin(path: str, cloud: np.ndarray) -> None:
    """(N, 4) float32 → KITTI .bin file."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    cloud.astype(np.float32).tofile(path)
    print(f"    [✓] {len(cloud):,} points → {path}")


# ─────────────────────────────────────────────────────────────────────────────
# 5.  Optional visualisation
# ─────────────────────────────────────────────────────────────────────────────

def visualise_o3d(original: np.ndarray, wall: np.ndarray) -> None:
    if not HAS_O3D:
        print("[!] open3d not installed – skipping visualisation.")
        return

    def to_o3d(pts, colour):
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(pts[:, :3].astype(np.float64))
        pc.paint_uniform_color(colour)
        return pc

    orig_pc = to_o3d(original, [0.55, 0.55, 0.55])
    wall_pc = to_o3d(wall,     [1.00, 0.20, 0.10])

    o3d.visualization.draw_geometries(
        [orig_pc, wall_pc],
        window_name="KITTI Object – synthesised wall (red)",
        width=1280, height=720,
        point_show_normal=False,
    )


# ─────────────────────────────────────────────────────────────────────────────
# 6.  Per-frame processing
# ─────────────────────────────────────────────────────────────────────────────

def process_frame(
    frame_id:     str,            # e.g. "000042"
    dataset_root: Path,
    out_root:     Path,
    u_min: float, u_max: float,
    v_min: float, v_max: float,
    disparity: float,
    density: float,
    reflectance: float,
    add_noise_m: float,
    remove_occluded: bool,
    write_label: bool,
    visualise: bool,
):
    print(f"\n{'─'*60}")
    print(f"  Frame: {frame_id}")
    print(f"{'─'*60}")

    # ── paths ────────────────────────────────────────────────────────────
    calib_path  = dataset_root / "calib"   / f"{frame_id}.txt"
    bin_in      = dataset_root / "velodyne"/ f"{frame_id}.bin"
    label_in    = dataset_root / "label_2" / f"{frame_id}.txt"

    bin_out     = out_root / "velodyne" / f"{frame_id}.bin"
    label_out   = out_root / "label_2"  / f"{frame_id}.txt"

    for p in [calib_path, bin_in]:
        if not p.exists():
            print(f"  [!] Missing: {p}  – skipping frame.")
            return

    # ── calibration ──────────────────────────────────────────────────────
    calib = KITTIObjCalib(str(calib_path))
    print(f"  {calib}")
    depth = calib.depth_from_disparity(disparity)
    print(f"  disparity={disparity:.2f} px  →  wall depth Z={depth:.3f} m")

    # ── original cloud ────────────────────────────────────────────────────
    original = load_bin(str(bin_in))
    print(f"  Original cloud: {len(original):,} points")

    # ── optional occlusion removal ────────────────────────────────────────
    if remove_occluded:
        original = remove_occluded_original_points(
            original, calib, u_min, u_max, v_min, v_max, depth
        )

    # ── generate wall ─────────────────────────────────────────────────────
    wall = generate_wall_points(
        calib       = calib,
        u_min       = u_min,
        u_max       = u_max,
        v_min       = v_min,
        v_max       = v_max,
        disparity   = disparity,
        density     = density,
        reflectance = reflectance,
        add_noise_m = add_noise_m,
    )
    print(f"  Wall points:    {len(wall):,}")

    # ── merge & save ──────────────────────────────────────────────────────
    merged = np.vstack([original, wall])
    save_bin(str(bin_out), merged)

    # ── optional label ────────────────────────────────────────────────────
    if write_label:
        # Copy original label first so we append to a fresh copy
        os.makedirs(label_out.parent, exist_ok=True)
        if label_in.exists():
            import shutil
            shutil.copy2(str(label_in), str(label_out))
        write_wall_label(
            str(label_out),
            u_min, u_max, v_min, v_max,
            calib, disparity,
        )

    # ── visualisation ─────────────────────────────────────────────────────
    if visualise:
        visualise_o3d(original, wall)


# ─────────────────────────────────────────────────────────────────────────────
# 7.  CLI
# ─────────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=__doc__,
    )

    # Dataset paths
    p.add_argument("--dataset_root", required=True,
                   help="Root of KITTI object dataset "
                        "(must contain calib/, velodyne/, image_2/, image_3/)")
    p.add_argument("--out_root", default=None,
                   help="Output root (default: <dataset_root>_wall_synth/). "
                        "Mirrors the same sub-folder structure.")

    # Frame selection
    grp = p.add_mutually_exclusive_group(required=True)
    grp.add_argument("--frame_id", type=str,
                     help="Single frame ID, e.g. 000042")
    grp.add_argument("--frame_list", type=str,
                     help="Text file with one frame ID per line")
    grp.add_argument("--all_frames", action="store_true",
                     help="Process every frame found in velodyne/")

    # Wall definition
    p.add_argument("--u_min",     type=float, required=True,
                   help="Left  column bound of wall in left image (px)")
    p.add_argument("--u_max",     type=float, required=True,
                   help="Right column bound of wall in left image (px)")
    p.add_argument("--v_min",     type=float, required=True,
                   help="Top   row    bound of wall in left image (px)")
    p.add_argument("--v_max",     type=float, required=True,
                   help="Bottom row    bound of wall in left image (px)")
    p.add_argument("--disparity", type=float, required=True,
                   help="Constant disparity of the flat wall (pixels)")

    # Synthesis options
    p.add_argument("--density",         type=float, default=0.5,
                   help="Wall point density: pts per pixel edge (default 0.5)")
    p.add_argument("--reflectance",     type=float, default=0.5,
                   help="Fake LiDAR intensity [0-1] (default 0.5)")
    p.add_argument("--add_noise_m",     type=float, default=0.0,
                   help="Gaussian depth noise std-dev in metres (default 0 = flat)")
    p.add_argument("--remove_occluded", action="store_true",
                   help="Remove original LiDAR points occluded by the wall")
    p.add_argument("--write_label",     action="store_true",
                   help="Append a KITTI-format Wall label to label_2/")
    p.add_argument("--visualise",       action="store_true",
                   help="Show each frame in Open3D (requires open3d)")

    return p.parse_args()


def collect_frame_ids(args, dataset_root: Path) -> list[str]:
    if args.frame_id:
        return [args.frame_id.zfill(6)]
    if args.frame_list:
        with open(args.frame_list) as f:
            return [l.strip().zfill(6) for l in f if l.strip()]
    # --all_frames: discover from velodyne/
    bins = sorted((dataset_root / "velodyne").glob("*.bin"))
    return [b.stem for b in bins]


def main():
    args = parse_args()

    dataset_root = Path(args.dataset_root)
    if not dataset_root.is_dir():
        sys.exit(f"[ERROR] dataset_root not found: {dataset_root}")

    out_root = Path(args.out_root) if args.out_root \
               else dataset_root.parent / (dataset_root.name + "_wall_synth")
    print(f"Output root: {out_root}")

    frame_ids = collect_frame_ids(args, dataset_root)
    print(f"Processing {len(frame_ids)} frame(s): "
          f"{frame_ids[0]} … {frame_ids[-1]}")

    for fid in frame_ids:
        process_frame(
            frame_id        = fid,
            dataset_root    = dataset_root,
            out_root        = out_root,
            u_min           = args.u_min,
            u_max           = args.u_max,
            v_min           = args.v_min,
            v_max           = args.v_max,
            disparity       = args.disparity,
            density         = args.density,
            reflectance     = args.reflectance,
            add_noise_m     = args.add_noise_m,
            remove_occluded = args.remove_occluded,
            write_label     = args.write_label,
            visualise       = args.visualise,
        )

    print("\n[✓] Done.")


if __name__ == "__main__":
    main()