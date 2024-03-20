import cv2
import numpy as np

# https://www.tutorialspoint.com/how-to-find-the-hsv-values-of-a-color-using-opencv-python
def rgb_to_hsv(RGB):

    # Define a numpy.ndarray for the color
    # Here insert the bgr values which you want to convert to hsv
    pixel = np.uint8([[RGB]])

    # convert the color to HSV
    hsv   = cv2.cvtColor(pixel, cv2.COLOR_BGR2HSV)
    hsvDw = hsv[0][0][0] - 10,100,100
    hsvUp = hsv[0][0][0] + 10,255,255
    return hsv, [hsvDw, hsvUp] 

def image_rgb_to_hsv(path, write=False, show=False):

    # Read the input RGB image as BGR format
    image = cv2.cvtColor(cv2.imread(path), cv2.COLOR_BGR2HSV)
    # Write the HSV image
    if write:
        filename = (path.split('/')[-1]).split('.')[-2:]
        cv2.imwrite(filename[0] + "-hsv." + filename[1], image)
    # Display the HSV image
    if show:
        cv2.imshow('HSV image', image)
        #print(image[1048, 1910]) # borde izq
        #print(image[1525, 1814]) # centro
        cv2.waitKey(0)
        cv2.destroyAllWindows()
