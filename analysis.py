"""End-to-end pipeline: extract an airfoil contour from an image and find the
closest base-airfoil matches in a single command.

Usage:
    python pipeline.py <image_filename> [options]

Example:
    python pipeline.py airfoil_edge_test2_crop2.png --no-show --top 5
"""

import argparse
import sys
from pathlib import Path

import image as image_mod
import compare as compare_mod
from load_airfoil_csv import PROJECT_ROOT, HINGE_X


DEFAULT_BASE_DIR = PROJECT_ROOT / "base-airfoils"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Extract airfoil contour from an image and find closest base-airfoil matches.",
    )
    parser.add_argument(
        "input_image",
        help="Image filename inside the input-images/ directory.",
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
    parser.add_argument(
        "--no-show", action="store_true",
        help="Skip opening the preview window.",
    )
    parser.add_argument(
        "--csv-dir", type=Path,
        help="Directory for generated CSV files (default: output-csv/).",
    )
    parser.add_argument(
        "--preview-dir", type=Path,
        help="Directory for generated preview images (default: preview-images/).",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    # Step 1: extract contour from image
    print("=" * 60)
    print("Step 1: Extracting airfoil contour from image")
    print("=" * 60)
    try:
        csv_path = image_mod.run(
            input_image=args.input_image,
            csv_dir=args.csv_dir,
            preview_dir=args.preview_dir,
            no_show=args.no_show,
        )
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        print(f"Error during contour extraction: {exc}", file=sys.stderr)
        return 1

    # Step 2: compare against base airfoils
    print()
    print("=" * 60)
    print("Step 2: Comparing against base airfoils")
    print("=" * 60)
    try:
        results = compare_mod.run(
            image_csv=csv_path,
            base_dir=args.base_dir,
            top=args.top,
            n_points=args.n_points,
        )
    except (FileNotFoundError, ValueError) as exc:
        print(f"Error during comparison: {exc}", file=sys.stderr)
        return 1

    # Print results table
    base_dir = Path(args.base_dir)
    bases = compare_mod.discover_base_airfoils(base_dir)
    _, _, x_max = compare_mod.build_rear_index(bases, HINGE_X, args.n_points)
    compare_mod.print_results(
        results, csv_path.name, len(bases),
        base_dir.name, args.n_points, x_max,
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
