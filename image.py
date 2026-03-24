import argparse
import csv
from pathlib import Path

import cv2
import numpy as np
from numpy.polynomial import Chebyshev

#project directory layout
PROJECT_ROOT = Path(__file__).resolve().parent
DEFAULT_INPUT_DIR = PROJECT_ROOT / "input-images"
DEFAULT_CSV_DIR = PROJECT_ROOT / "output-csv"
DEFAULT_PREVIEW_DIR = PROJECT_ROOT / "preview-images"


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
    top = np.argmax(cols, axis=0).astype(int)
    bottom = (h - 1 - np.argmax(cols[::-1], axis=0)).astype(int)

    #smooth raw edges then fit chebyshev polynomials
    envelope_window = max(81, (w // 14) | 1)  #odd-sized kernel, >=81 px
    top_curve = fit_surface_curve(
        xs, smooth_curve(top, envelope_window, cv2.MORPH_OPEN), "top", degree=6,
    )
    bottom_curve = fit_surface_curve(
        xs, smooth_curve(bottom, envelope_window, cv2.MORPH_CLOSE), "bottom", degree=5,
    )

    #clamp to image bounds, ensure bottom is always below top
    top_curve = np.clip(top_curve, 0, h - 1)
    bottom_curve = np.clip(bottom_curve, 0, h - 1)
    bottom_curve = np.maximum(bottom_curve, top_curve + 1)

    return xs, top_curve, bottom_curve


#output helpers

#normalize pixel surfaces to chord=1 and arrange in selig order (TE→LE upper, LE→TE lower)
def normalize_to_selig(xs, top_curve, bottom_curve):
    x = xs.astype(float)
    top = top_curve.astype(float)
    bot = bottom_curve.astype(float)

    x_min, x_max = x.min(), x.max()
    chord = x_max - x_min
    if chord == 0:
        raise RuntimeError("Extracted airfoil has zero chord length.")

    x_norm = (x - x_min) / chord

    #y-reference: camber midpoint at trailing edge (max x)
    te_idx = int(np.argmax(x_norm))
    y_ref = (top[te_idx] + bot[te_idx]) / 2.0

    #negate to flip pixel-y (down) → aero-y (up), normalize by chord
    upper_y = -(top - y_ref) / chord
    lower_y = -(bot - y_ref) / chord

    #sort ascending in x for splitting
    order = np.argsort(x_norm)
    x_sorted = x_norm[order]
    upper_sorted = upper_y[order]
    lower_sorted = lower_y[order]

    #selig order: upper surface TE→LE (x descending), then lower LE→TE (x ascending)
    #skip first point of lower surface to avoid duplicating the leading edge
    selig_x = np.concatenate([x_sorted[::-1], x_sorted[1:]])
    selig_y = np.concatenate([upper_sorted[::-1], lower_sorted[1:]])

    return selig_x, selig_y


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

    #core pipeline: mask, surface curves, preview overlay
    mask = build_primary_mask(image)
    xs, top_curve, bottom_curve = extract_surface_curves(mask)
    preview = make_preview(image, xs, top_curve, bottom_curve)

    #normalize to chord=1 selig format and write
    selig_x, selig_y = normalize_to_selig(xs, top_curve, bottom_curve)
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
