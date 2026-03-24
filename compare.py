import argparse
import csv
import sys
from pathlib import Path

import numpy as np


#CLI

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare an image-extracted airfoil CSV against a base airfoil CSV "
                    "using least-squares residuals."
    )
    parser.add_argument(
        "image_csv",
        help="Path to the CSV produced by image.py (normalized Selig format).",
    )
    parser.add_argument(
        "base_csv",
        help="Path to the base airfoil CSV (normalized Selig format).",
    )
    parser.add_argument(
        "--n-points", type=int, default=200,
        help="Number of interpolation points on the common x-grid (default: 200).",
    )
    return parser.parse_args()


#Loader

def load_airfoil_csv(path: Path) -> dict:
    """Load a Selig-format airfoil CSV (x,y with header).

    Splits at the leading edge (minimum x) into upper and lower surfaces.
    Returns dict with upper_x, upper_y, lower_x, lower_y (all ascending in x).
    """
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        try:
            next(reader)  #skip header
        except StopIteration:
            raise ValueError(f"CSV file is empty: {path}")
        try:
            rows = [list(map(float, row)) for row in reader]
        except ValueError as exc:
            raise ValueError(f"Non-numeric data in CSV {path}: {exc}")

    if not rows:
        raise ValueError(f"CSV file contains no data rows: {path}")

    data = np.array(rows)
    if not np.all(np.isfinite(data)):
        raise ValueError(f"CSV contains NaN or Inf values: {path}")

    x_all = data[:, 0]
    y_all = data[:, 1]

    le_idx = int(np.argmin(x_all))  #leading-edge row

    if le_idx < 1 or le_idx >= len(x_all) - 1:
        raise ValueError(
            f"Leading edge at boundary (idx={le_idx}, n={len(x_all)}) — "
            f"cannot split into upper/lower surfaces: {path}"
        )

    #upper surface: TE → LE (reverse so x is ascending)
    upper_x = x_all[: le_idx + 1][::-1]
    upper_y = y_all[: le_idx + 1][::-1]

    #lower surface: LE → TE (already ascending in x)
    lower_x = x_all[le_idx:]
    lower_y = y_all[le_idx:]

    return {
        "upper_x": upper_x,
        "upper_y": upper_y,
        "lower_x": lower_x,
        "lower_y": lower_y,
    }


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


#Main

def main() -> int:
    args = parse_args()

    image_path = Path(args.image_csv)
    base_path = Path(args.base_csv)

    if not image_path.is_file():
        print(f"Error: image CSV not found: {image_path}", file=sys.stderr)
        return 1
    if not base_path.is_file():
        print(f"Error: base airfoil CSV not found: {base_path}", file=sys.stderr)
        return 1
    if args.n_points < 2:
        print("Error: --n-points must be at least 2", file=sys.stderr)
        return 1

    #load both as selig-format CSVs
    try:
        image_data = load_airfoil_csv(image_path)
        base_data = load_airfoil_csv(base_path)
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    #compare
    result = least_squares_compare(image_data, base_data, args.n_points)

    #output
    print(f"Base airfoil : {base_path.stem}")
    print(f"RMSE (total) : {result['rmse']:.6f}")
    print(f"RMSE (upper) : {result['upper_rmse']:.6f}")
    print(f"RMSE (lower) : {result['lower_rmse']:.6f}")
    print(f"MSE          : {result['mse']:.8f}")
    print(f"Grid points  : {result['n_points']}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
