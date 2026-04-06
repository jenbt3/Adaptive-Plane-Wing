"""Shared I/O utilities and constants for the adaptive airfoil pipeline."""

import csv
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent
HINGE_X = 0.6  #normalized chord position where the fixed front meets the movable rear


def load_selig_csv(path: Path) -> dict:
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

    le_idx = int(np.argmin(x_all))

    if le_idx < 1 or le_idx >= len(x_all) - 1:
        raise ValueError(
            f"Leading edge at boundary (idx={le_idx}, n={len(x_all)}) — "
            f"cannot split into upper/lower surfaces: {path}"
        )

    #upper surface: TE → LE (reverse so x is ascending)
    upper_x = x_all[: le_idx + 1][::-1].copy()
    upper_y = y_all[: le_idx + 1][::-1].copy()

    #lower surface: LE → TE (already ascending in x)
    lower_x = x_all[le_idx:].copy()
    lower_y = y_all[le_idx:].copy()

    return {
        "upper_x": upper_x,
        "upper_y": upper_y,
        "lower_x": lower_x,
        "lower_y": lower_y,
    }
