import cv2 as cv
import numpy as np

# Initialising camera
cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()


def empty(a):
    pass


# Creating window with trackbars to choose lower and upper HSV limits
cv.namedWindow("HSV")
cv.resizeWindow("HSV", 640, 240)
cv.createTrackbar("H Min", "HSV", 0, 179, empty)
cv.createTrackbar("H Max", "HSV", 179, 179, empty)
cv.createTrackbar("S Min", "HSV", 0, 255, empty)
cv.createTrackbar("S Max", "HSV", 255, 255, empty)
cv.createTrackbar("V Min", "HSV", 0, 255, empty)
cv.createTrackbar("V Max", "HSV", 255, 255, empty)

while True:
    # Reading camera frame
    success, frame = cap.read()
    if not success:
        print("Can't receive frame. Exiting")
        break

    # Converting frame to HSV color space
    frameHsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # Getting trackbar positions of the HSV window
    hMin = cv.getTrackbarPos("H Min", "HSV")
    hMax = cv.getTrackbarPos("H Max", "HSV")
    sMin = cv.getTrackbarPos("S Min", "HSV")
    sMax = cv.getTrackbarPos("S Max", "HSV")
    vMin = cv.getTrackbarPos("V Min", "HSV")
    vMax = cv.getTrackbarPos("V Max", "HSV")

    # Creating arrays with the lower and upper HSV limits
    lower = np.array([hMin, sMin, vMin])
    upper = np.array([hMax, sMax, vMax])

    # Filtering out the areas within the HSV limits
    mask = cv.inRange(frameHsv, lower, upper)
    result = cv.bitwise_and(frame, frame, mask=mask)

    # Displaying the mask and result
    cv.imshow('Mask', mask)
    cv.imshow('Result', result)
    if cv.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv.destroyAllWindows()
