import argparse
import re
import sys
from pathlib import Path

import numpy as np

from load_airfoil_csv import PROJECT_ROOT, HINGE_X, load_selig_csv

DEFAULT_BASE_DIR = PROJECT_ROOT / "base-airfoils"


#CLI

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Find the closest base airfoils to an image-extracted airfoil CSV."
    )
    parser.add_argument(
        "image_csv",
        help="Path to the CSV produced by image.py (normalized Selig format).",
    )
    parser.add_argument(
        "--base-dir", type=Path, default=DEFAULT_BASE_DIR,
        help="Directory containing base airfoil CSVs (default: base-airfoils/).",
    )
    parser.add_argument(
        "--top", type=int, default=3,
        help="Number of top matches to return (default: 3).",
    )
    parser.add_argument(
        "--n-points", type=int, default=100,
        help="Number of interpolation points on the rear x-grid (default: 100).",
    )
    return parser.parse_args()


#Comparison

def interpolate_to_grid(surface_x, surface_y, grid):
    """Linearly interpolate a surface onto a common x-grid."""
    return np.interp(grid, surface_x, surface_y)


def least_squares_compare(image_norm: dict, base: dict, n_points: int) -> dict:
    """Compute least-squares residuals between image and base airfoils.

    Both inputs must have upper_x/y and lower_x/y arrays, normalized to [0,1].
    Returns dict with mse, rmse, and per-surface breakdowns.
    """
    grid = np.linspace(0.0, 1.0, n_points)

    img_upper = interpolate_to_grid(image_norm["upper_x"], image_norm["upper_y"], grid)
    img_lower = interpolate_to_grid(image_norm["lower_x"], image_norm["lower_y"], grid)
    base_upper = interpolate_to_grid(base["upper_x"], base["upper_y"], grid)
    base_lower = interpolate_to_grid(base["lower_x"], base["lower_y"], grid)

    upper_sq = (img_upper - base_upper) ** 2
    lower_sq = (img_lower - base_lower) ** 2

    total_sq = np.concatenate([upper_sq, lower_sq])
    mse = float(total_sq.mean())
    rmse = float(np.sqrt(mse))

    return {
        "mse": mse,
        "rmse": rmse,
        "upper_rmse": float(np.sqrt(upper_sq.mean())),
        "lower_rmse": float(np.sqrt(lower_sq.mean())),
        "n_points": n_points,
    }


#Discovery

THETA_RE = re.compile(r"theta\s*=?\s*(-?\d+)")


def parse_theta(filename: str) -> int | None:
    """Extract theta integer from a base-airfoil filename."""
    m = THETA_RE.search(filename)
    return int(m.group(1)) if m else None


def discover_base_airfoils(base_dir: Path) -> list[tuple[int, Path]]:
    """Scan base-dir for airfoil CSVs, deduplicate by theta, return sorted list."""
    seen: dict[int, Path] = {}
    for p in sorted(base_dir.glob("*.csv")):
        theta = parse_theta(p.stem)
        if theta is None:
            continue
        #prefer filenames with "=" for consistency; keep first seen otherwise
        if theta not in seen or "=" in p.stem:
            seen[theta] = p
    return sorted(seen.items())


#Batch rear-section indexing

def extract_rear(airfoil: dict, hinge_x: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Extract the rear section (x >= hinge_x) from an airfoil dict."""
    u_mask = airfoil["upper_x"] >= hinge_x
    l_mask = airfoil["lower_x"] >= hinge_x
    if not u_mask.any() or not l_mask.any():
        raise ValueError(
            f"No points at or beyond hinge x={hinge_x} — "
            f"upper x-range [{airfoil['upper_x'].min():.4f}, {airfoil['upper_x'].max():.4f}], "
            f"lower x-range [{airfoil['lower_x'].min():.4f}, {airfoil['lower_x'].max():.4f}]"
        )
    return (
        airfoil["upper_x"][u_mask], airfoil["upper_y"][u_mask],
        airfoil["lower_x"][l_mask], airfoil["lower_y"][l_mask],
    )


def build_rear_index(
    bases: list[tuple[int, Path]], hinge_x: float, n_grid: int,
) -> tuple[np.ndarray, np.ndarray, float]:
    """Load all base airfoils, interpolate rear sections onto a shared grid.

    Returns (matrix, grid, x_max) where matrix has shape (N_bases, 2*n_grid)
    with upper then lower y-values concatenated per row.
    """
    #first pass: find the smallest common x-max across all bases
    rears = []
    for _, path in bases:
        airfoil = load_selig_csv(path)
        rear = extract_rear(airfoil, hinge_x)
        rears.append(rear)

    x_max = min(
        min(r[0].max(), r[2].max()) for r in rears
    )
    grid = np.linspace(hinge_x, x_max, n_grid)

    #second pass: interpolate each rear onto the shared grid, stack into matrix
    rows = []
    for ru_x, ru_y, rl_x, rl_y in rears:
        upper_interp = np.interp(grid, ru_x, ru_y)
        lower_interp = np.interp(grid, rl_x, rl_y)
        rows.append(np.concatenate([upper_interp, lower_interp]))

    return np.array(rows), grid, x_max


#Vectorized batch comparison

def compare_rear_batch(
    image_data: dict, matrix: np.ndarray, grid: np.ndarray,
) -> np.ndarray:
    """Compute RMSE of image rear section against all base rears simultaneously."""
    ru_x, ru_y, rl_x, rl_y = extract_rear(image_data, HINGE_X)
    n_grid = len(grid)

    img_upper = np.interp(grid, ru_x, ru_y)
    img_lower = np.interp(grid, rl_x, rl_y)
    img_vec = np.concatenate([img_upper, img_lower])

    #broadcast: (N_bases, 2*n_grid) - (2*n_grid,) → squared differences
    diff_sq = (matrix - img_vec) ** 2
    mse = diff_sq.mean(axis=1)
    return np.sqrt(mse)


#Main

def main() -> int:
    args = parse_args()

    image_path = Path(args.image_csv)
    base_dir = Path(args.base_dir)

    if not image_path.is_file():
        print(f"Error: image CSV not found: {image_path}", file=sys.stderr)
        return 1
    if not base_dir.is_dir():
        print(f"Error: base airfoil directory not found: {base_dir}", file=sys.stderr)
        return 1
    if args.n_points < 2:
        print("Error: --n-points must be at least 2", file=sys.stderr)
        return 1
    if args.top < 1:
        print("Error: --top must be at least 1", file=sys.stderr)
        return 1

    #discover base airfoils
    bases = discover_base_airfoils(base_dir)
    if not bases:
        print(f"Error: no base airfoil CSVs found in {base_dir}", file=sys.stderr)
        return 1

    #load image airfoil
    try:
        image_data = load_selig_csv(image_path)
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    #build rear-section index and compare in batch
    try:
        matrix, grid, x_max = build_rear_index(bases, HINGE_X, args.n_points)
    except ValueError as exc:
        print(f"Error loading base airfoils: {exc}", file=sys.stderr)
        return 1

    rmse_all = compare_rear_batch(image_data, matrix, grid)

    #rank and pick top N
    top_n = min(args.top, len(bases))
    ranked_idx = np.argsort(rmse_all)[:top_n]

    #refinement: full-airfoil comparison on top matches only
    results = []
    for rank, idx in enumerate(ranked_idx, 1):
        theta, path = bases[idx]
        rear_rmse = float(rmse_all[idx])
        base_data = load_selig_csv(path)
        full = least_squares_compare(image_data, base_data, 200)
        results.append({
            "rank": rank,
            "name": path.stem,
            "theta": theta,
            "rear_rmse": rear_rmse,
            "full_rmse": full["rmse"],
            "upper_rmse": full["upper_rmse"],
            "lower_rmse": full["lower_rmse"],
        })

    #output
    print(f"Image   : {image_path.name}")
    print(f"Bases   : {len(bases)} airfoils in {base_dir.name}/")
    print(f"Grid    : {args.n_points} pts on rear section (x = {HINGE_X:.1f} .. {x_max:.4f})")
    print()
    print(f"{'Rank':<6}{'Theta':>6}  {'Rear RMSE':>12}  {'Full RMSE':>12}  {'Upper RMSE':>12}  {'Lower RMSE':>12}  Name")
    print("-" * 90)
    for r in results:
        print(
            f"{r['rank']:<6}{r['theta']:>6}°  "
            f"{r['rear_rmse']:>12.7f}  {r['full_rmse']:>12.7f}  "
            f"{r['upper_rmse']:>12.7f}  {r['lower_rmse']:>12.7f}  "
            f"{r['name']}"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
