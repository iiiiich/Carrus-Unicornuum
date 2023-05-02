import cv2 as cv

# Initialising camera
cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

while True:
    # Reading camera frame
    success, frame = cap.read()
    if not success:
        print("Can't receive frame. Exiting")
        break

    # Displaying camera frame
    cv.imshow("Reference Image", frame)

    # Saving image as "referenceImage.png" when "a" is pressed
    if cv.waitKey(1) == ord("a"):
        print("pressed a")
        frame = cv.imwrite("/home/pi/Desktop/referenceImage.png", frame)
        break
    
cap.release()
cv.destroyAllWindows()