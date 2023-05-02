import cv2 as cv
import numpy as np

# Defining the real distance and height of the object in the reference image
knownHeight = 100
knownDistance = 270

# Creating arrays with the lower and upper HSV limits
lower = np.array([52, 122, 87])
upper = np.array([89, 244, 255])


# Function to calculate the focal length of the camera
def focalLengthFinder():
    focalLength = (referenceImageObjHeight * knownDistance) / knownHeight
    return focalLength


# Function to find objects within the HSV limits and their height in pixels
def objData(frame):
    objHeight = 0
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower, upper)
    _, mask1 = cv.threshold(mask, 254, 255, cv.THRESH_BINARY)
    contours, _ = cv.findContours(mask1, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    for contour in contours:
        minArea = 600
        if cv.contourArea(contour) > minArea:
            x, y, w, h = cv.boundingRect(contour)
            cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            objHeight = h
    return objHeight


# Reading the reference image and finding the height of the object in pixels in the reference image
referenceImage = cv.imread("referenceImage.png")
referenceImageObjHeight = objData(referenceImage)

# Calculating and printing the focal length of the camera
focalLength = focalLengthFinder()
print(focalLength)
