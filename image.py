import argparse
import csv
from pathlib import Path

import cv2
import numpy as np
from numpy.polynomial import Chebyshev

PROJECT_ROOT = Path(__file__).resolve().parent
DEFAULT_INPUT_DIR = PROJECT_ROOT / "input-images"
DEFAULT_CSV_DIR = PROJECT_ROOT / "output-csv"
DEFAULT_PREVIEW_DIR = PROJECT_ROOT / "preview-images"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Extract an airfoil contour from an image, save CSV and preview output."
    )
    parser.add_argument(
        "input_image",
        nargs="?",
        help="Path to the input image file.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="Directory for generated files. Overrides both CSV and preview directories.",
    )
    parser.add_argument(
        "--csv-dir",
        type=Path,
        help="Directory for generated CSV files. Defaults to output-csv.",
    )
    parser.add_argument(
        "--preview-dir",
        type=Path,
        help="Directory for generated preview images. Defaults to preview-images.",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Save outputs without opening the preview window.",
    )
    return parser.parse_args()


def request_input_path(provided_path: str | None) -> Path:
    image_path = provided_path
    if not image_path:
        image_path = input("Input image file: ").strip()

    raw_path = Path(image_path).expanduser()
    candidates = []

    if raw_path.is_absolute():
        candidates.append(raw_path)
    else:
        candidates.append(Path.cwd() / raw_path)
        candidates.append(PROJECT_ROOT / raw_path)
        candidates.append(DEFAULT_INPUT_DIR / raw_path.name)
        candidates.append(DEFAULT_INPUT_DIR / raw_path)

    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()

    searched_paths = "\n".join(f" - {candidate.resolve()}" for candidate in candidates)
    raise FileNotFoundError(
        f"Input image not found for '{image_path}'. Searched:\n{searched_paths}"
    )


def resolve_output_dir(
    provided_dir: Path | None,
    fallback_dir: Path,
) -> Path:
    if provided_dir is None:
        return fallback_dir

    expanded = provided_dir.expanduser()
    if expanded.is_absolute():
        return expanded.resolve()
    return (PROJECT_ROOT / expanded).resolve()


def contours_from_mask(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    return contours


def touches_border(contour, width: int, height: int) -> bool:
    x, y, w, h = cv2.boundingRect(contour)
    return x <= 0 or y <= 0 or (x + w) >= width or (y + h) >= height


def pick_contour(contours, width: int, height: int):
    if not contours:
        return None

    non_border = [
        contour for contour in contours if not touches_border(contour, width, height)
    ]
    if non_border:
        return max(non_border, key=cv2.contourArea)

    # Fall back to the largest contour only if the image truly fills the frame.
    return max(contours, key=cv2.contourArea)


def largest_contour(mask):
    contours = contours_from_mask(mask)
    height, width = mask.shape[:2]
    return pick_contour(contours, width, height)


def clean_mask(mask):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def extract_blue_contour(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_blue = (95, 50, 40)
    upper_blue = (135, 255, 255)
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    blue_mask = clean_mask(blue_mask)

    contours = contours_from_mask(blue_mask)
    if not contours:
        return None

    image_area = image.shape[0] * image.shape[1]
    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    if area < image_area * 0.01:
        return None

    return contour


def extract_contour(image):
    blue_contour = extract_blue_contour(image)
    if blue_contour is not None:
        return blue_contour

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    _, binary = cv2.threshold(
        blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )
    _, inverted = cv2.threshold(
        blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
    )

    all_contours = contours_from_mask(binary) + contours_from_mask(inverted)
    contour = pick_contour(all_contours, image.shape[1], image.shape[0])
    if contour is None:
        raise RuntimeError("No contour could be detected in the input image.")

    return contour


def build_mask_from_contour(image, contour):
    mask = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    mask[:] = 0
    cv2.drawContours(mask, [contour], -1, 255, thickness=cv2.FILLED)
    return mask


def build_primary_mask(image):
    blue_contour = extract_blue_contour(image)
    if blue_contour is not None:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        blue_mask = cv2.inRange(hsv, (95, 50, 40), (135, 255, 255))
        return clean_mask(blue_mask)

    contour = extract_contour(image)
    return build_mask_from_contour(image, contour)


def smooth_curve(values, window: int, mode: int):
    signal = values.reshape(1, -1).astype("float32")
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (window, 1))
    smoothed = cv2.morphologyEx(signal, mode, kernel)
    smoothed = cv2.GaussianBlur(smoothed, (window, 1), 0)
    return smoothed.reshape(-1).round().astype(int)


def build_surface_envelope(values, side: str, window: int):
    if side == "top":
        return smooth_curve(values, window, cv2.MORPH_OPEN)
    return smooth_curve(values, window, cv2.MORPH_CLOSE)


def fit_surface_curve(xs, ys, side: str, degree: int, iterations: int = 6):
    valid = np.ones_like(ys, dtype=bool)

    for _ in range(iterations):
        fit_degree = min(degree, int(valid.sum()) - 1)
        if fit_degree < 1:
            break

        model = Chebyshev.fit(
            xs[valid],
            ys[valid],
            fit_degree,
            domain=[int(xs.min()), int(xs.max())],
        )
        fitted = model(xs)
        residuals = ys - fitted
        median = np.median(residuals[valid])
        mad = np.median(np.abs(residuals[valid] - median))
        scale = max(1.0, 1.4826 * mad)

        if side == "top":
            new_valid = residuals < (median + 2.0 * scale)
        else:
            new_valid = residuals > (median - 2.0 * scale)

        if np.array_equal(new_valid, valid):
            break
        valid = new_valid

    fit_degree = min(degree, int(valid.sum()) - 1)
    if fit_degree < 1:
        return ys.copy()

    model = Chebyshev.fit(
        xs[valid],
        ys[valid],
        fit_degree,
        domain=[int(xs.min()), int(xs.max())],
    )
    fitted = model(xs)
    return fitted.round().astype(int)


def extract_surface_curves(mask):
    xs = np.where(mask.any(axis=0))[0]
    if xs.size == 0:
        raise RuntimeError("No visible shape could be extracted from the image.")

    top = []
    bottom = []
    for x in xs:
        ys = np.where(mask[:, x] > 0)[0]
        top.append(int(ys.min()))
        bottom.append(int(ys.max()))

    top = np.array(top, dtype=int)
    bottom = np.array(bottom, dtype=int)

    envelope_window = max(81, ((mask.shape[1] // 14) | 1))
    top_envelope = build_surface_envelope(top, "top", envelope_window)
    bottom_envelope = build_surface_envelope(bottom, "bottom", envelope_window)

    top_curve = fit_surface_curve(xs, top_envelope, "top", degree=6)
    bottom_curve = fit_surface_curve(xs, bottom_envelope, "bottom", degree=5)

    top_curve = np.clip(top_curve, 0, mask.shape[0] - 1)
    bottom_curve = np.clip(bottom_curve, 0, mask.shape[0] - 1)
    bottom_curve = np.maximum(bottom_curve, top_curve + 1)

    return xs, top_curve, bottom_curve


def write_csv(xs, top_curve, bottom_curve, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["x", "top_y", "bottom_y"])
        for x, top_y, bottom_y in zip(xs, top_curve, bottom_curve):
            writer.writerow([int(x), int(top_y), int(bottom_y)])


def make_preview(image, xs, top_curve, bottom_curve):
    preview = image.copy()
    top_points = np.column_stack((xs, top_curve)).reshape(-1, 1, 2).astype(int)
    bottom_points = np.column_stack((xs, bottom_curve)).reshape(-1, 1, 2).astype(int)
    cv2.polylines(preview, [top_points], False, (0, 0, 255), 2)
    cv2.polylines(preview, [bottom_points], False, (0, 255, 0), 2)
    return preview


def show_preview(preview, image_path: Path) -> None:
    window_name = f"Preview: {image_path.name}"
    try:
        cv2.imshow(window_name, preview)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    except cv2.error as exc:
        print(f"Preview window could not be opened: {exc}")


def main() -> int:
    args = parse_args()
    input_path = request_input_path(args.input_image)

    image = cv2.imread(str(input_path))
    if image is None:
        raise RuntimeError(f"OpenCV could not load the image: {input_path}")

    if args.output_dir:
        csv_dir = resolve_output_dir(args.output_dir, DEFAULT_CSV_DIR)
        preview_dir = resolve_output_dir(args.output_dir, DEFAULT_PREVIEW_DIR)
    else:
        csv_dir = resolve_output_dir(args.csv_dir, DEFAULT_CSV_DIR)
        preview_dir = resolve_output_dir(args.preview_dir, DEFAULT_PREVIEW_DIR)

    csv_path = csv_dir / f"{input_path.stem}_contour.csv"
    preview_path = preview_dir / f"{input_path.stem}_preview.png"

    mask = build_primary_mask(image)
    xs, top_curve, bottom_curve = extract_surface_curves(mask)
    preview = make_preview(image, xs, top_curve, bottom_curve)

    write_csv(xs, top_curve, bottom_curve, csv_path)
    preview_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(preview_path), preview)

    print(f"Input image: {input_path}")
    print(f"CSV output: {csv_path}")
    print(f"Preview image: {preview_path}")
    print(f"Curve samples written: {len(xs)}")

    if not args.no_show:
        show_preview(preview, input_path)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
