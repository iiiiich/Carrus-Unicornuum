import cv2 as cv
import numpy as np

# Defining the real distance and height of the object in the reference image
known_height = 100
known_distance = 270

# Creating arrays with the lower and upper HSV limits
lower = np.array([52, 122, 87])
upper = np.array([89, 244, 255])


# Function to calculate the focal length of the camera
def focal_length_finder():
    focal_length = (reference_image_obj_height * known_distance) / known_height
    return focal_length


# Function to find objects within the HSV limits and their height in pixels
def obj_data(frame):
    obj_height = 0
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower, upper)
    _, mask1 = cv.threshold(mask, 254, 255, cv.THRESH_BINARY)
    contours, _ = cv.findContours(mask1, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    for contour in contours:
        min_area = 600
        if cv.contourArea(contour) > min_area:
            x, y, w, h = cv.boundingRect(contour)
            cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            obj_height = h
    return obj_height


# Reading the reference image and finding the height of the object in pixels in the reference image
reference_image = cv.imread("reference_image.png")
reference_image_obj_height = obj_data(reference_image)

# Calculating and printing the focal length of the camera
focal_length = focal_length_finder()
print(focal_length)
