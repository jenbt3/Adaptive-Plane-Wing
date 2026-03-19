import os
import cv2

# image_path = os.path.join('data', 'bird.jpg')
# img =cv2.imread(image_path)

# cv2.imwrite(os.path.join('data', 'bird_out.jpg'), img)

# cv2.imshow('frame', img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

img = cv2.imread(os.path.join('data', 'bird.jpg'))
resized_img = cv2.resize(img, (1312, 1125))

print(img.shape)
print(resized_img.shape)

cv2.imshow('frame', img)
cv2.imshow('resized', resized_img)
cv2.waitKey(0)
cv2.destroyAllWindows()