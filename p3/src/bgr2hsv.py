import cv2
import numpy as np


# read the input RGB image as BGR format
hsv_img = cv2.imread('muy_cerca_hsv.jpg')

# hsv_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
# cv2.imwrite('muy_cerca_hsv.jpg', hsv_img)

# Display the HSV image
#cv2.imshow('HSV image', hsv_img)
print(hsv_img[1048, 1910]) # borde izq
print(hsv_img[1525, 1814]) # centro
cv2.waitKey(0)
cv2.destroyAllWindows()