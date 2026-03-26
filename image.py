import argparse
import csv
from pathlib import Path

import cv2
import numpy as np
from numpy.polynomial import Chebyshev

from load_airfoil_csv import PROJECT_ROOT, HINGE_X, load_selig_csv

#project directory layout
DEFAULT_INPUT_DIR = PROJECT_ROOT / "input-images"
DEFAULT_CSV_DIR = PROJECT_ROOT / "output-csv"
DEFAULT_PREVIEW_DIR = PROJECT_ROOT / "preview-images"
DEFAULT_BASELINE = PROJECT_ROOT / "base-airfoils" / "NACA 2412 theta = 0.csv"


#cli argument parsing

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Extract an airfoil contour from an image, save CSV and preview output."
    )
    parser.add_argument("input_image", help="Image filename inside the input-images/ directory.")
    parser.add_argument(
        "--output-dir", type=Path,
        help="Directory for generated files. Overrides both CSV and preview directories.",
    )
    parser.add_argument(
        "--csv-dir", type=Path,
        help="Directory for generated CSV files. Defaults to output-csv.",
    )
    parser.add_argument(
        "--preview-dir", type=Path,
        help="Directory for generated preview images. Defaults to preview-images.",
    )
    parser.add_argument(
        "--no-show", action="store_true",
        help="Save outputs without opening the preview window.",
    )
    return parser.parse_args()


#resolve output directory, falling back to default
def resolve_output_dir(provided_dir: Path | None, fallback_dir: Path) -> Path:
    if provided_dir is None:
        return fallback_dir
    expanded = provided_dir.expanduser()
    return expanded.resolve() if expanded.is_absolute() else (PROJECT_ROOT / expanded).resolve()


#contour utilities

#find all external contours in a binary mask
def find_contours(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    return contours


#true if contour bounding box touches any image edge
def touches_border(contour, width: int, height: int) -> bool:
    x, y, w, h = cv2.boundingRect(contour)
    return x <= 0 or y <= 0 or (x + w) >= width or (y + h) >= height


#select largest contour, preferring ones that don't touch the border
def pick_contour(contours, width: int, height: int):
    if not contours:
        return None
    non_border = [c for c in contours if not touches_border(c, width, height)]
    return max(non_border or contours, key=cv2.contourArea)


#remove noise with morphological open then close
def clean_mask(mask):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


#mask extraction

#build binary mask of the airfoil, trying blue hsv then grayscale otsu
def build_primary_mask(image):
    h, w = image.shape[:2]

    #try blue-channel detection first
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    blue_mask = clean_mask(cv2.inRange(hsv, (95, 50, 40), (135, 255, 255)))
    contours = find_contours(blue_mask)
    if contours:
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) >= h * w * 0.01:  #at least 1% of image area
            return blue_mask

    #fall back to grayscale otsu thresholding (normal + inverted)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, binary = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    _, inverted = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    #pick the best contour from both threshold results
    contour = pick_contour(find_contours(binary) + find_contours(inverted), w, h)
    if contour is None:
        raise RuntimeError("No contour could be detected in the input image.")

    #fill the chosen contour into a clean mask
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, thickness=cv2.FILLED)
    return mask


#curve fitting

#morphological smoothing + gaussian blur on a 1d signal
def smooth_curve(values, window, mode):
    signal = values.astype(np.float32).reshape(1, -1)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (window, 1))
    smoothed = cv2.morphologyEx(signal, mode, kernel)
    return cv2.GaussianBlur(smoothed, (window, 1), 0).ravel().round().astype(int)


#fit chebyshev polynomial with iterative mad-based outlier rejection
def fit_surface_curve(xs, ys, side, degree, iterations=6):
    valid = np.ones(len(ys), dtype=bool)
    domain = [xs.min(), xs.max()]

    #iteratively fit and reject outliers
    for _ in range(iterations):
        fit_deg = min(degree, valid.sum() - 1)
        if fit_deg < 1:
            break

        model = Chebyshev.fit(xs[valid], ys[valid], fit_deg, domain=domain)
        residuals = ys - model(xs)

        #robust spread estimate (mad to standard deviation proxy)
        median = np.median(residuals[valid])
        mad = np.median(np.abs(residuals[valid] - median))
        scale = max(1.0, 1.4826 * mad)

        #keep only inliers on the correct side of the surface
        threshold = median + 2.0 * scale if side == "top" else median - 2.0 * scale
        new_valid = residuals < threshold if side == "top" else residuals > threshold

        if np.array_equal(new_valid, valid):
            break
        valid = new_valid

    #final fit on cleaned data
    fit_deg = min(degree, valid.sum() - 1)
    if fit_deg < 1:
        return ys.copy()

    model = Chebyshev.fit(xs[valid], ys[valid], fit_deg, domain=domain)
    return model(xs).round().astype(int)


#extract smoothed top/bottom surface curves from the airfoil mask
def extract_surface_curves(mask):
    h, w = mask.shape[:2]
    xs = np.where(mask.any(axis=0))[0]  #columns that contain the airfoil
    if xs.size == 0:
        raise RuntimeError("No visible shape could be extracted from the image.")

    #find topmost and bottommost mask pixel per column
    cols = mask[:, xs] > 0
    assert cols.any(axis=0).all(), "Mask column with no True pixels after filtering"
    raw_top = np.argmax(cols, axis=0).astype(int)
    raw_bottom = (h - 1 - np.argmax(cols[::-1], axis=0)).astype(int)

    #smooth raw edges then fit chebyshev polynomials
    envelope_window = max(81, (w // 14) | 1)  #odd-sized kernel, >=81 px
    top_curve = fit_surface_curve(
        xs, smooth_curve(raw_top, envelope_window, cv2.MORPH_OPEN), "top", degree=6,
    )
    bottom_curve = fit_surface_curve(
        xs, smooth_curve(raw_bottom, envelope_window, cv2.MORPH_CLOSE), "bottom", degree=5,
    )

    #clamp to image bounds
    top_curve = np.clip(top_curve, 0, h - 1)
    bottom_curve = np.clip(bottom_curve, 0, h - 1)

    #detect trailing edge (thinner end) from raw mask data
    raw_thickness = (raw_bottom - raw_top).astype(float)
    n_sample = max(1, len(xs) // 20)
    te_on_right = raw_thickness[:n_sample].mean() >= raw_thickness[-n_sample:].mean()

    #blend fitted curves toward raw edge data near trailing edge so they converge
    blend_len = max(3, len(xs) // 10)
    if te_on_right:
        start = len(xs) - blend_len
        alpha = np.linspace(0, 1, blend_len)
        top_curve[start:] = np.round(
            top_curve[start:] * (1 - alpha) + raw_top[start:] * alpha
        ).astype(int)
        bottom_curve[start:] = np.round(
            bottom_curve[start:] * (1 - alpha) + raw_bottom[start:] * alpha
        ).astype(int)
    else:
        alpha = np.linspace(1, 0, blend_len)
        top_curve[:blend_len] = np.round(
            top_curve[:blend_len] * (1 - alpha) + raw_top[:blend_len] * alpha
        ).astype(int)
        bottom_curve[:blend_len] = np.round(
            bottom_curve[:blend_len] * (1 - alpha) + raw_bottom[:blend_len] * alpha
        ).astype(int)

    #ensure bottom >= top (allow equality at trailing edge for convergence)
    bottom_curve = np.maximum(bottom_curve, top_curve)
    top_curve = np.clip(top_curve, 0, h - 1)
    bottom_curve = np.clip(bottom_curve, 0, h - 1)

    return xs, top_curve, bottom_curve


#baseline loading

def load_baseline_front(path: Path, hinge_x: float = HINGE_X) -> dict:
    """Load the NACA 2412 theta=0 baseline and clip to x <= hinge_x.

    Returns front-section upper/lower arrays (ascending x, clipped at hinge_x)
    plus interpolated y-values exactly at the hinge.
    """
    base = load_selig_csv(path)

    #interpolate surface y-values at exactly hinge_x
    upper_y_at_hinge = float(np.interp(hinge_x, base["upper_x"], base["upper_y"]))
    lower_y_at_hinge = float(np.interp(hinge_x, base["lower_x"], base["lower_y"]))

    #clip upper surface to x <= hinge_x, append the hinge point
    mask_u = base["upper_x"] <= hinge_x
    front_upper_x = np.append(base["upper_x"][mask_u], hinge_x)
    front_upper_y = np.append(base["upper_y"][mask_u], upper_y_at_hinge)

    #clip lower surface to x <= hinge_x, append the hinge point
    mask_l = base["lower_x"] <= hinge_x
    front_lower_x = np.append(base["lower_x"][mask_l], hinge_x)
    front_lower_y = np.append(base["lower_y"][mask_l], lower_y_at_hinge)

    return {
        "upper_x": front_upper_x, "upper_y": front_upper_y,
        "lower_x": front_lower_x, "lower_y": front_lower_y,
        "upper_y_at_hinge": upper_y_at_hinge,
        "lower_y_at_hinge": lower_y_at_hinge,
    }


#trailing-section normalization

def detect_orientation(xs, top_curve, bottom_curve):
    """Determine which pixel end is the hinge (thicker) vs trailing edge (thinner).

    Returns (hinge_pixel_x, hinge_side) where hinge_side is 'left' or 'right'.
    """
    thickness = (bottom_curve - top_curve).astype(float)

    #average thickness over the first and last 5% of columns
    n = max(1, len(xs) // 20)
    left_thickness = thickness[:n].mean()
    right_thickness = thickness[-n:].mean()

    if left_thickness >= right_thickness:
        #left end is thicker → hinge is at pixel x_min
        return xs[0], "left"
    else:
        #right end is thicker → hinge is at pixel x_max
        return xs[-1], "right"


def normalize_trailing_section(xs, top_curve, bottom_curve, front):
    """Map extracted pixel curves of the trailing 40% to normalized airfoil coordinates.

    Uses the baseline front's hinge-point thickness to calibrate the pixel→chord scale.
    Returns (rear_x, rear_upper_y, rear_lower_y) sorted ascending in x, for x >= HINGE_X.
    """
    hinge_pixel_x, hinge_side = detect_orientation(xs, top_curve, bottom_curve)

    #pixel thickness at the hinge end (average over nearest 5% of columns)
    thickness_px = (bottom_curve - top_curve).astype(float)
    n = max(1, len(xs) // 20)
    if hinge_side == "left":
        hinge_thickness_px = thickness_px[:n].mean()
        hinge_top_px = float(top_curve[:n].mean())
        hinge_bot_px = float(bottom_curve[:n].mean())
    else:
        hinge_thickness_px = thickness_px[-n:].mean()
        hinge_top_px = float(top_curve[-n:].mean())
        hinge_bot_px = float(bottom_curve[-n:].mean())

    #baseline thickness at hinge in normalized chord units
    baseline_thickness = front["upper_y_at_hinge"] - front["lower_y_at_hinge"]
    if baseline_thickness <= 0 or hinge_thickness_px <= 0:
        raise RuntimeError("Cannot calibrate scale: zero thickness at hinge.")

    pixels_per_unit = hinge_thickness_px / baseline_thickness

    #pixel camber midpoint at hinge, and baseline camber at hinge
    camber_px = (hinge_top_px + hinge_bot_px) / 2.0
    camber_base = (front["upper_y_at_hinge"] + front["lower_y_at_hinge"]) / 2.0

    #x mapping: pixel offset from hinge → normalized chord
    px = xs.astype(float)
    if hinge_side == "left":
        #hinge is at left (min pixel x), TE is at right (max pixel x)
        rear_x = HINGE_X + (px - hinge_pixel_x) / pixels_per_unit
    else:
        #hinge is at right (max pixel x), TE is at left (min pixel x)
        rear_x = HINGE_X - (px - hinge_pixel_x) / pixels_per_unit

    #y mapping: negate pixel-y (down=positive) → aero-y (up=positive)
    top_f = top_curve.astype(float)
    bot_f = bottom_curve.astype(float)
    rear_upper_y = camber_base - (top_f - camber_px) / pixels_per_unit
    rear_lower_y = camber_base - (bot_f - camber_px) / pixels_per_unit

    #sort ascending in x
    order = np.argsort(rear_x)
    rear_x = rear_x[order]
    rear_upper_y = rear_upper_y[order]
    rear_lower_y = rear_lower_y[order]

    #clip to x >= HINGE_X (discard any pixels that mapped before the hinge)
    valid = rear_x >= HINGE_X
    return rear_x[valid], rear_upper_y[valid], rear_lower_y[valid]


def splice_airfoil(front, rear_x, rear_upper_y, rear_lower_y):
    """Combine the fixed front 60% (from baseline) with the extracted rear 40%.

    Returns (selig_x, selig_y) in Selig order: upper TE→LE, lower LE→TE.
    """
    #front surfaces already end at hinge_x; rear starts at hinge_x
    #skip first rear point if it duplicates the hinge x
    rear_start = 1 if len(rear_x) > 1 and np.isclose(rear_x[0], front["upper_x"][-1], atol=1e-6) else 0

    #upper surface: front ascending x + rear ascending x
    full_upper_x = np.concatenate([front["upper_x"], rear_x[rear_start:]])
    full_upper_y = np.concatenate([front["upper_y"], rear_upper_y[rear_start:]])

    #lower surface: front ascending x + rear ascending x
    full_lower_x = np.concatenate([front["lower_x"], rear_x[rear_start:]])
    full_lower_y = np.concatenate([front["lower_y"], rear_lower_y[rear_start:]])

    #selig order: upper TE→LE (x descending), then lower LE→TE (x ascending)
    #skip first lower point to avoid duplicating leading edge
    selig_x = np.concatenate([full_upper_x[::-1], full_lower_x[1:]])
    selig_y = np.concatenate([full_upper_y[::-1], full_lower_y[1:]])

    return selig_x, selig_y


#output helpers


#write normalized airfoil data to csv in selig format (x, y)
def write_csv(selig_x, selig_y, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y"])
        for x, y in zip(selig_x, selig_y):
            writer.writerow([f"{x:.7f}", f"{y:.7f}"])


#draw top (red) and bottom (green) curves onto image copy
def make_preview(image, xs, top_curve, bottom_curve):
    preview = image.copy()
    for pts, color in [
        (np.column_stack((xs, top_curve)), (0, 0, 255)),   #red = top
        (np.column_stack((xs, bottom_curve)), (0, 255, 0)), #green = bottom
    ]:
        cv2.polylines(preview, [pts.reshape(-1, 1, 2).astype(int)], False, color, 2)
    return preview


#display preview window, skip silently if gui unavailable
def show_preview(preview, image_path: Path) -> None:
    try:
        cv2.imshow(f"Preview: {image_path.name}", preview)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    except cv2.error as exc:
        print(f"Preview window could not be opened: {exc}")


#entry point

def main() -> int:
    args = parse_args()

    #input image is always in input-images/
    input_path = (DEFAULT_INPUT_DIR / args.input_image).resolve()
    if not input_path.is_file():
        raise FileNotFoundError(f"Image not found: {input_path}")

    image = cv2.imread(str(input_path))
    if image is None:
        raise RuntimeError(f"OpenCV could not load the image: {input_path}")

    #resolve output directories (--output-dir overrides both)
    override = args.output_dir
    csv_dir = resolve_output_dir(override or args.csv_dir, DEFAULT_CSV_DIR)
    preview_dir = resolve_output_dir(override or args.preview_dir, DEFAULT_PREVIEW_DIR)

    csv_path = csv_dir / f"{input_path.stem}_contour.csv"
    preview_path = preview_dir / f"{input_path.stem}_preview.png"

    #load baseline front 60% from NACA 2412 theta=0
    if not DEFAULT_BASELINE.is_file():
        raise FileNotFoundError(f"Baseline CSV not found: {DEFAULT_BASELINE}")
    front = load_baseline_front(DEFAULT_BASELINE)

    #core pipeline: mask, surface curves, preview overlay
    mask = build_primary_mask(image)
    xs, top_curve, bottom_curve = extract_surface_curves(mask)
    preview = make_preview(image, xs, top_curve, bottom_curve)

    #normalize extracted trailing 40% and splice with fixed front 60%
    rear_x, rear_upper_y, rear_lower_y = normalize_trailing_section(
        xs, top_curve, bottom_curve, front,
    )
    selig_x, selig_y = splice_airfoil(front, rear_x, rear_upper_y, rear_lower_y)
    write_csv(selig_x, selig_y, csv_path)
    preview_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(preview_path), preview)

    print(f"Input image: {input_path}")
    print(f"CSV output: {csv_path}")
    print(f"Preview image: {preview_path}")
    print(f"Curve samples written: {len(selig_x)}")

    if not args.no_show:
        show_preview(preview, input_path)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
