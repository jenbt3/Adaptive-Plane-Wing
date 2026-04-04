import cv2

# Load image in grayscale
img = cv2.imread("image.JPG", cv2.IMREAD_GRAYSCALE)

# Apply Gaussian Blur to reduce noise
blur = cv2.GaussianBlur(img, (5, 5), 1.4)

# Apply Canny Edge Detector
edges = cv2.Canny(blur, threshold1=100, threshold2=200)

# Display result
cv2.imshow("Canny Edge Detection", edges)

cv2.waitKey(0)
cv2.destroyAllWindows()