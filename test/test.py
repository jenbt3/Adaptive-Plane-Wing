import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import medfilt

# Load image
img = cv2.imread("test/image.jpg")

# Safety check
if img is None:
    raise FileNotFoundError("Image not found — check your path")

# --- Step 1: Isolate the blue wing using color masking ---
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
lower_blue = np.array([90, 50, 50])
upper_blue = np.array([130, 255, 255])
mask = cv2.inRange(hsv, lower_blue, upper_blue)

# Clean up the mask
kernel = np.ones((5,5), np.uint8)
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

# Apply mask
masked = cv2.bitwise_and(img, img, mask=mask)

# --- Step 2: Canny edge detection ---
gray = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (7, 7), 2)
edges = cv2.Canny(blurred, 30, 100)

# --- Step 3: Extract top and bottom edges per column ---
h, w = edges.shape
top_edge_x, top_edge_y = [], []
bottom_edge_x, bottom_edge_y = [], []

for col in range(w):
    col_pixels = np.where(edges[:, col] > 0)[0]
    if len(col_pixels) > 0:
        top_edge_x.append(col)
        top_edge_y.append(np.min(col_pixels))
        bottom_edge_x.append(col)
        bottom_edge_y.append(np.max(col_pixels))

top_edge_x = np.array(top_edge_x)
top_edge_y = np.array(top_edge_y)
bottom_edge_x = np.array(bottom_edge_x)
bottom_edge_y = np.array(bottom_edge_y)

# --- Step 4: Smooth out bracket gaps with median filter ---
top_edge_y_smooth = medfilt(top_edge_y.astype(float), kernel_size=25)
bottom_edge_y_smooth = medfilt(bottom_edge_y.astype(float), kernel_size=25)

# --- Step 5: Draw connected lines onto image ---
output = img.copy()

top_pts = np.array([[x, y] for x, y in zip(top_edge_x, top_edge_y_smooth.astype(int))], dtype=np.int32)
bottom_pts = np.array([[x, y] for x, y in zip(bottom_edge_x, bottom_edge_y_smooth.astype(int))], dtype=np.int32)

cv2.polylines(output, [top_pts],    isClosed=False, color=(0, 0, 255), thickness=2)  # red = top
cv2.polylines(output, [bottom_pts], isClosed=False, color=(0, 255, 0), thickness=2)  # green = bottom

# --- Step 6: Plot ---
plt.figure(figsize=(16, 8))

plt.subplot(2, 2, 1)
plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
plt.title("Original")
plt.axis("off")

plt.subplot(2, 2, 2)
plt.imshow(cv2.cvtColor(masked, cv2.COLOR_BGR2RGB))
plt.title("Blue mask applied")
plt.axis("off")

plt.subplot(2, 2, 3)
plt.imshow(edges, cmap="gray")
plt.title("Canny edges")
plt.axis("off")

plt.subplot(2, 2, 4)
plt.imshow(cv2.cvtColor(output, cv2.COLOR_BGR2RGB))
plt.title("Top (red) + Bottom (green) edges")
plt.axis("off")

plt.tight_layout()
plt.show()