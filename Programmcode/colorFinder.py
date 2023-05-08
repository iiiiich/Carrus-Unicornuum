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
    frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # Getting trackbar positions of the HSV window
    h_min = cv.getTrackbarPos("H Min", "HSV")
    h_max = cv.getTrackbarPos("H Max", "HSV")
    s_min = cv.getTrackbarPos("S Min", "HSV")
    s_max = cv.getTrackbarPos("S Max", "HSV")
    v_min = cv.getTrackbarPos("V Min", "HSV")
    v_max = cv.getTrackbarPos("V Max", "HSV")

    # Creating arrays with the lower and upper HSV limits
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])

    # Filtering out the areas within the HSV limits
    mask = cv.inRange(frame_hsv, lower, upper)
    result = cv.bitwise_and(frame, frame, mask=mask)

    # Displaying the mask and result
    cv.imshow('Mask', mask)
    cv.imshow('Result', result)
    if cv.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv.destroyAllWindows()
