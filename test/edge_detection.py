import cv2
import numpy as np

#load image in grayscale
img = cv2.imread('image.jpg', cv2.IMREAD_GRAYSCALE)

#sobel operator
sobelx = cv2.Sobel(img, cv2.CV_64F, 1, 0, ksize=3)  # Horizontal edges
sobely = cv2.Sobel(img, cv2.CV_64F, 0, 1, ksize=3)  # Vertical edges


# Compute gradient magnitude
gradient_magnitude = cv2.magnitude(sobelx, sobely)

# Convert to uint8
gradient_magnitude = cv2.convertScaleAbs(gradient_magnitude)

# Display result
cv2.imshow("Sobel Edge Detection", gradient_magnitude)

cv2.waitKey(0)
cv2.destroyAllWindows()