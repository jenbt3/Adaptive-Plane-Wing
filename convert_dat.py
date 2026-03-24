"""Batch-convert Selig-format .dat airfoil files to CSV for use with compare.py."""

import argparse
import csv
import sys
from pathlib import Path

DEFAULT_SOURCE = Path(__file__).resolve().parent / "dat-airfoils"
DEFAULT_OUTPUT = Path(__file__).resolve().parent / "base-airfoils"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert Selig .dat airfoil files to CSV (x,y header, comma-separated)."
    )
    parser.add_argument(
        "--source",
        type=Path,
        default=DEFAULT_SOURCE,
        help=f"Directory containing .dat files (default: {DEFAULT_SOURCE.name}/)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help=f"Directory for output CSVs (default: {DEFAULT_OUTPUT.name}/)",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Overwrite existing CSV files.",
    )
    return parser.parse_args()


def convert_dat_to_csv(dat_path: Path, csv_path: Path) -> None:
    """Read a Selig .dat file and write a CSV with x,y header and 7-decimal-place values."""
    coordinates: list[tuple[str, str]] = []

    with dat_path.open("r", encoding="utf-8") as f:
        lines = f.readlines()

    # First line is always the airfoil name — skip it
    for line in lines[1:]:
        stripped = line.strip()
        if not stripped:
            continue
        parts = stripped.split()
        if len(parts) != 2:
            continue
        try:
            x = float(parts[0])
            y = float(parts[1])
        except ValueError:
            continue
        coordinates.append((f"{x:.7f}", f"{y:.7f}"))

    if not coordinates:
        print(f"  WARNING: No coordinates found in {dat_path.name}, skipping.")
        return

    with csv_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y"])
        writer.writerows(coordinates)

    print(f"  Converted: {dat_path.name}  ->  {csv_path.name}  ({len(coordinates)} points)")


def main() -> None:
    args = parse_args()

    if not args.source.is_dir():
        print(f"Error: source directory not found: {args.source}", file=sys.stderr)
        sys.exit(1)

    args.output.mkdir(parents=True, exist_ok=True)

    dat_files = sorted(args.source.glob("*.dat"))
    if not dat_files:
        print(f"No .dat files found in {args.source}")
        sys.exit(0)

    converted = 0
    skipped = 0

    print(f"Source:  {args.source}")
    print(f"Output:  {args.output}")
    print(f"Found {len(dat_files)} .dat file(s)\n")

    for dat_path in dat_files:
        csv_path = args.output / (dat_path.stem + ".csv")

        if csv_path.exists() and not args.force:
            print(f"  Skipped (exists): {csv_path.name}")
            skipped += 1
            continue

        convert_dat_to_csv(dat_path, csv_path)
        converted += 1

    print(f"\nDone: {converted} converted, {skipped} skipped.")


if __name__ == "__main__":
    main()
