# Adaptive Plane Wing

An adaptive airfoil wing system that extracts airfoil shapes from images, compares them against baseline designs, and controls wing shape in real-time using an ESP32 with servo motors and an IMU.

## Project Structure

```
Adaptive-Plane-Wing/
├── image.py            # Extract airfoil contours from images → CSV
├── compare.py          # Compare two airfoil CSVs (least-squares RMSE)
├── convert_dat.py      # Batch-convert Selig .dat files → CSV
├── final.cpp           # ESP32 firmware (servo control via IMU pitch angle)
├── base-airfoils/      # Reference airfoil CSVs (Selig format)
├── dat-airfoils/       # Raw Selig .dat airfoil files
├── input-images/       # Source images for contour extraction
├── output-csv/         # CSVs produced by image.py
├── preview-images/     # Annotated preview images from image.py
└── test/               # Test code
```

## Requirements

- Python 3.10+
- OpenCV (`opencv-python`)
- NumPy

```
pip install numpy opencv-python
```

## Scripts

### `image.py` — Airfoil Contour Extraction

Extracts an airfoil contour from a PNG image, fits Chebyshev polynomials to the upper/lower surfaces, and outputs a normalized Selig-format CSV.

```
python image.py <image_filename>
```

The image file must be in `input-images/`. Outputs go to `output-csv/` (CSV) and `preview-images/` (annotated image).

Options:
- `--csv-dir DIR` — Custom CSV output directory
- `--preview-dir DIR` — Custom preview output directory
- `--no-show` — Skip opening the preview window

### `compare.py` — Airfoil Comparison

Compares two Selig-format airfoil CSVs using least-squares residuals. Reports MSE and RMSE for the total profile, upper surface, and lower surface.

```
python compare.py <csv_a> <csv_b>
```

Options:
- `--n-points N` — Number of interpolation points on the common x-grid (default: 200)

Example:
```
python compare.py output-csv/sample_airfoil_contour.csv "base-airfoils/NACA 2412 theta 0.csv"
```

### `convert_dat.py` — DAT-to-CSV Batch Converter

Converts Selig-format `.dat` airfoil files into CSVs compatible with `compare.py` and `image.py`.

```
python convert_dat.py
```

This reads all `.dat` files from `dat-airfoils/` and writes CSVs to `base-airfoils/`. Existing CSVs are skipped by default.

Options:
- `--source DIR` — Source directory for `.dat` files (default: `dat-airfoils/`)
- `--output DIR` — Output directory for CSVs (default: `base-airfoils/`)
- `--force` — Overwrite existing CSV files

**Adding new airfoils:**
1. Drop `.dat` files into `dat-airfoils/`
2. Run `python convert_dat.py`
3. New CSVs appear in `base-airfoils/`, ready for comparison

## CSV Format

All airfoil CSVs use normalized Selig format:
- Header row: `x,y`
- Coordinates normalized to chord length 1.0
- Selig ordering: trailing edge → leading edge (upper surface), then leading edge → trailing edge (lower surface)
- 7 decimal places

## Firmware (`final.cpp`)

ESP32 Arduino firmware for real-time wing shape control:
- Reads pitch angle from an MPU6050 IMU via complementary filter
- Maps angle of attack to servo positions using a lookup table (`camber_position.h`)
- Controls 3 servo motors to adjust wing camber
