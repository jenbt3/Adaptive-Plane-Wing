# Adaptive Plane Wing — Airfoil Analysis Pipeline

An adaptive airfoil wing system that extracts airfoil shapes from images, compares them against NACA 2412 baseline designs at different angles, and identifies the best matching wing configuration. Includes ESP32 firmware for real-time servo control.

## What It Does

The project implements a **wing shape matching pipeline**:

1. **Extract** — Read an airfoil image, detect contour using OpenCV
2. **Normalize** — Fit Chebyshev polynomials to upper/lower surfaces with outlier rejection
3. **Calibrate** — Replace fixed front 60% with NACA 2412 baseline, keep extracted rear 40%
4. **Compare** — Find closest NACA 2412 base airfoil (at different theta angles) using vectorized RMSE
5. **Report** — Output best theta match and detailed comparison statistics

Base airfoils are NACA 2412 profiles at theta angles from −40° to +20° (32 variants).

## Project Structure

```
Adaptive-Plane-Wing/
├── load_airfoil_csv.py   # Shared I/O: constants (PROJECT_ROOT, HINGE_X), load_selig_csv()
├── image.py              # Extract contour from images → normalized CSV
├── compare.py            # Find closest base airfoils to image CSV
├── convert_dat.py        # Batch-convert Selig .dat files → CSV
├── final.cpp             # ESP32 firmware (servo control via IMU)
│
├── base-airfoils/        # NACA 2412 .csv files (32 theta variants, -40° to +20°)
├── dat-airfoils/         # Source Selig .dat airfoil format files
├── input-images/         # Input PNG images for contour extraction
├── output-csv/           # Generated airfoil CSVs (image.py output)
├── preview-images/       # Annotated preview images with surface curves (image.py output)
└── test/                 # Test code (C++)
```

## Requirements

**Python:**
- Python 3.10+
- NumPy
- OpenCV (`opencv-python`)

Install dependencies:
```bash
pip install numpy opencv-python
```

**C++ (for firmware):**
- Arduino IDE or PlatformIO
- ESP32 board support
- Libraries: `ESP32Servo`, `Adafruit_MPU6050`, `Adafruit_Sensor`

## Usage: Extract Airfoil from Image

```bash
python image.py <image_filename>
```

**Where:**
- `<image_filename>` — PNG file in `input-images/` (relative name, no path needed)

**Outputs:**
- `output-csv/<filename>_contour.csv` — Normalized airfoil in Selig format
- `preview-images/<filename>_preview.png` — Overlay showing detected contour

**Options:**
- `--output-dir DIR` — Override both CSV and preview directories
- `--csv-dir DIR` — CSV output directory (default: `output-csv/`)
- `--preview-dir DIR` — Preview image directory (default: `preview-images/`)
- `--no-show` — Skip opening preview window

**Example:**
```bash
python image.py wing_photo_1.png
```

## Usage: Compare Airfoil to Base Library

```bash
python compare.py <extracted_csv>
```

**Where:**
- `<extracted_csv>` — Path to CSV from `image.py` or any Selig-format CSV

**Outputs to console:**
- Ranked list of closest NACA 2412 base airfoils
- Theta angle (−40° to +20°) of best match
- Comparison metrics for rear section and full airfoil
- Per-surface RMSE (upper and lower)

**Options:**
- `--base-dir DIR` — Base airfoil directory (default: `base-airfoils/`)
- `--top N` — Return top N matches (default: 3)
- `--n-points N` — Interpolation grid points for rear section (default: 100)

**Comparison algorithm:**
1. Fast vectorized RMSE on rear section (x ≥ 0.6) using interpolated y-values
2. Rank by rear RMSE
3. Refine top N with full-airfoil least-squares comparison (200 points)
4. Report theta angle and per-surface metrics

**Example:**
```bash
python compare.py output-csv/wing_photo_1_contour.csv
python compare.py output-csv/wing_photo_1_contour.csv --top 5 --n-points 150
```

## Usage: Convert Selig .dat Files to CSV

```bash
python convert_dat.py
```

**What it does:**
- Reads all `.dat` files from `dat-airfoils/`
- Writes normalized CSV to `base-airfoils/` with `x,y` header, 7 decimal places
- Skips existing CSVs by default

**Options:**
- `--source DIR` — Input directory (default: `dat-airfoils/`)
- `--output DIR` — Output directory (default: `base-airfoils/`)
- `--force` — Overwrite existing CSVs

**Adding new airfoils:**
1. Place `.dat` files in `dat-airfoils/`
2. `python convert_dat.py`
3. New CSVs available in `base-airfoils/` for comparison

## Shared Module: `load_airfoil_csv.py`

Provides constants and utilities for all scripts:

**Constants:**
- `PROJECT_ROOT` — Directory containing this script
- `HINGE_X = 0.6` — Normalized chord position where fixed front meets movable rear

**Function:**
```python
load_selig_csv(path: Path) -> dict
```
Loads a Selig-format CSV. Splits at leading edge (minimum x) into upper and lower surfaces.

Returns:
```python
{
    "upper_x": ndarray,  # ascending in x
    "upper_y": ndarray,
    "lower_x": ndarray,  # ascending in x
    "lower_y": ndarray,
}
```

Raises `ValueError` if CSV is malformed or has NaN/Inf values.

## CSV Format: Selig Normalized

All airfoil CSVs follow this format:
- **Header:** `x,y`
- **Coordinates:** Normalized to chord length 1.0
- **Order:** Trailing edge → leading edge (upper surface), then leading edge → trailing edge (lower surface)
- **Precision:** 7 decimal places

Example coordinates (NACA 2412):
```
x,y
1.0000000,0.0000000
0.9500000,0.0024000
...
0.0000000,-0.0050000
0.0500000,-0.0045000
...
```

## Implementation Details: Image Processing Pipeline

### Contour Detection
- Tries HSV blue-channel detection first (for airfoil silhouettes on light backgrounds)
- Falls back to grayscale OTSU thresholding (normal and inverted)
- Selects largest contour, preferring ones not touching image borders
- Morphological open/close to remove noise

### Surface Fitting
- Extracts topmost and bottommost airfoil pixels per column (x)
- Smooths raw edges with morphological open/close
- Fits Chebyshev polynomials (degree 6 for upper, 5 for lower) to each surface
- Iterative MAD-based outlier rejection (6 iterations, 2σ threshold)

### Trailing-Section Normalization
- Detects orientation: which end is hinge (thicker, ~x=0.6) vs trailing edge (thinner)
- Calibrates pixel-to-chord scale using baseline thickness at hinge
- Maps pixel coordinates to normalized airfoil space
- Returns rear section (x ≥ HINGE_X) in ascending x order

### Airfoil Splicing
- Uses fixed front 60% (x < 0.6) from NACA 2412 baseline (theta=0)
- Replaces rear 40% (x ≥ 0.6) with extracted contour
- Outputs complete profile in Selig format

## Implementation Details: Comparison Algorithm

### Rear-Section Batch Indexing
- Loads all base airfoils, extracts rear sections (x ≥ 0.6)
- Finds smallest common x-range across all bases
- Interpolates all rear sections onto shared uniform grid
- Stores in matrix form for vectorized comparison

### Vectorized RMSE
- Interpolates image rear section onto the same grid
- Broadcasts matrix subtraction: (N_bases, 2×n_points) − (2×n_points,) 
- Computes RMSE per base airfoil in parallel (~1ms for 32 bases, 100 points)
- Ranks by rear RMSE, returns top N indices

### Refinement: Full Least-Squares
- For each top-N match, loads full base airfoil
- Interpolates both upper and lower surfaces onto 200-point uniform grid
- Computes MSE and RMSE for full profile, upper, and lower
- Reports detailed per-surface metrics

**Why two-stage comparison?**
- Fast rear-section indexing narrows candidates efficiently
- Full comparison avoids biasing toward front or rear extremes
- Theta angle extracted from filename via regex: `theta\s*=?\s*(-?\d+)`

## Known Issues & Limitations

### C++ Firmware Status
- `final.cpp` references `camber_position.h` (lookup table of servo positions vs. angle of attack)
- This file is **not included** in the project — must be generated or provided separately
- Firmware will not compile without this header

### Base Airfoil Coverage
- Limited to NACA 2412 at 32 discrete theta angles (−40° to +20°, typically in 2° steps)
- Comparison uses linear interpolation on rear section; accuracy depends on grid resolution (default 100 points)

## Data Flow Example

```
1. User provides:     wing_photo.png in input-images/

2. image.py:
   - Loads image
   - Detects blue or gray contour
   - Fits Chebyshev polynomials
   - Normalizes rear 40% against NACA 2412 baseline
   - Outputs: output-csv/wing_photo_contour.csv
            preview-images/wing_photo_preview.png

3. compare.py:
   - Loads extracted CSV and all 32 base airfoils
   - Fast batch RMSE on rear sections
   - Refines top 3 with full least-squares
   - Prints:
     Rank 1: theta=+6°, rear_rmse=0.0234, full_rmse=0.0567
     Rank 2: theta=+4°, rear_rmse=0.0241, full_rmse=0.0581
     Rank 3: theta=+8°, rear_rmse=0.0248, full_rmse=0.0598

4. User interprets:   Best match is NACA 2412 at theta=+6°
```

## ESP32 Firmware: `final.cpp`

Real-time wing control system:

**Hardware:**
- ESP32 microcontroller
- MPU6050 (6-axis IMU: 3-axis accelerometer + 3-axis gyroscope)
- 3 servo motors on pins 16, 17, 18
- I2C pins: SDA=4, SCL=5

**Algorithm:**
1. **Calibration:** At startup, averages 200 accelerometer readings to find level offset
2. **Filtering:** Complementary filter combines gyro (fast response) and accelerometer (drift correction)
   - Filter constant ALPHA = 0.98 (gyro-dominant)
3. **Servo Mapping:** Reads pitch angle, looks up servo position in `camber_position.h@`, writes same position to all 3 servos

**Servo Control:**
- Neutral position: 90°
- PWM frequency: 50 Hz
- All servos synchronized to same position

**Missing:** The `camber_position.h` header must define:
```cpp
struct ShapeConfig {
    int angleOfAttack;
    int servoPosition;
};

const ShapeConfig lookupTable[] = {
    { -40, 45 },
    { -38, 48 },
    ...
    { +20, 135 },
};

const int NUM_SHAPES = sizeof(lookupTable) / sizeof(lookupTable[0]);
```

Generate this by running `compare.py` on known wing configurations and recording servo positions for each theta angle.
