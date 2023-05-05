import RPi.GPIO as GPIO
import time
import cv2 as cv
import numpy as np
import math

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

speed = 65

# HSV limits for color detection
lowerRed = np.array([0, 186, 80])
upperRed = np.array([12, 255, 235])
lowerGreen = np.array([45, 112, 60])
upperGreen = np.array([77, 255, 241])

# Variables for camera distance calculation
knownHeight = 100
focalLength = 567.0

# Initialising button, motors and ultrasonic distance sensors
buttonPin = 40
Ena, In1, In2 = 3, 5, 7
servo = 29
trigFront, echoFront = 13, 18
trigRight, echoRight = 11, 15
trigLeft, echoLeft = 12, 16

GPIO.setup(buttonPin, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
GPIO.setup(Ena, GPIO.OUT)
GPIO.setup(In1, GPIO.OUT)
GPIO.setup(In2, GPIO.OUT)
GPIO.setup(servo, GPIO.OUT)
GPIO.setup(trigFront, GPIO.OUT)
GPIO.setup(echoFront, GPIO.IN)
GPIO.setup(trigRight, GPIO.OUT)
GPIO.setup(echoRight, GPIO.IN)
GPIO.setup(trigLeft, GPIO.OUT)
GPIO.setup(echoLeft, GPIO.IN)

motorPwm = GPIO.PWM(Ena, 100)
motorPwm.start(0)
servoPwm = GPIO.PWM(servo, 50)
servoPwm.start(0)
GPIO.output(In1, GPIO.LOW)
GPIO.output(In2, GPIO.LOW)

# Initialising camera
cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()


# Function to move forward
def forward(x):
    motorPwm.ChangeDutyCycle(x)
    GPIO.output(In1, GPIO.HIGH)
    GPIO.output(In2, GPIO.LOW)


# Function to move backward
def backward(x):
    motorPwm.ChangeDutyCycle(x)
    GPIO.output(In1, GPIO.LOW)
    GPIO.output(In2, GPIO.HIGH)


# Function to stop
def stop():
    GPIO.output(In1, GPIO.LOW)
    GPIO.output(In2, GPIO.LOW)


# Function to steer forward
def steerForward():
    servoPwm.ChangeDutyCycle(7.3)


# Function to steer left
def steerLeft():
    servoPwm.ChangeDutyCycle(9.3)


# Function to steer right
def steerRight():
    servoPwm.ChangeDutyCycle(5.3)


# Function to get measurements from ultrasonic distance sensors
def ultrasonicDistanceMeasurement():
    # Initialising variables for ultrasonic distance measurement
    pulseStartFront, pulseEndFront, pulseDurationFront = time.time(), time.time(), 0
    pulseStartRight, pulseEndRight, pulseDurationRight = time.time(), time.time(), 0
    pulseStartLeft, pulseEndLeft, pulseDurationLeft = time.time(), time.time(), 0
    global distanceFrontMeasured, distanceRightMeasured, distanceLeftMeasured
    
    # Triggering ultrasonic distance sensors
    GPIO.output(trigFront, GPIO.LOW)
    GPIO.output(trigRight, GPIO.LOW)
    GPIO.output(trigLeft, GPIO.LOW)
    time.sleep(0.02)
    
    # Getting front distance
    GPIO.output(trigFront, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trigFront, GPIO.LOW)
    while GPIO.input(echoFront) == 0:
        pulseStartFront = time.time()
    while GPIO.input(echoFront) == 1:
        pulseEndFront = time.time()
    pulseDurationFront = pulseEndFront - pulseStartFront
    distanceFrontMeasured = pulseDurationFront * 17150
    distanceFrontMeasured = round(distanceFrontMeasured, 2)
    
    # Getting right distance
    GPIO.output(trigRight, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trigRight, GPIO.LOW)
    while GPIO.input(echoRight) == 0:
        pulseStartRight = time.time()
    while GPIO.input(echoRight) == 1:
        pulseEndRight = time.time()
    pulseDurationRight = pulseEndRight - pulseStartRight
    distanceRightMeasured = pulseDurationRight * 17150
    distanceRightMeasured = round(distanceRightMeasured, 2)
    
    # Getting left distance
    GPIO.output(trigLeft, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trigLeft, GPIO.LOW)
    while GPIO.input(echoLeft) == 0:
        pulseStartLeft = time.time()
    while GPIO.input(echoLeft) == 1:
        pulseEndLeft = time.time()
    pulseDurationLeft = pulseEndLeft - pulseStartLeft
    distanceLeftMeasured = pulseDurationLeft * 17150
    distanceLeftMeasured = round(distanceLeftMeasured, 2)


# Function to get the height and x-coordinate in the frame of the biggest object within the HSV limits
def objData(frame, lower, upper, color):
    x, w, h = 400, 0, 0
    minArea = 600
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower, upper)
    _, mask1 = cv.threshold(mask, 254, 255, cv.THRESH_BINARY)
    contours, _ = cv.findContours(mask1, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    for contour in contours:
        if cv.contourArea(contour) > minArea:
            x, y, w, h = cv.boundingRect(contour)
            cv.rectangle(frame, (x, y), (x+w, y+h), color, 2)
            minArea = cv.contourArea(contour)  
    return (x + w/2), h


# Function to calculate the distance of an object in the camera frame
def distanceCalculation(focalLength, knownHeight, objHeightInFrame):
    global cameraAngle
    distance = (knownHeight * focalLength) / objHeightInFrame
    horizontalDistance = math.sqrt(distance**2 - 81**2)
    return horizontalDistance


stop()
steerForward()
time.sleep(0.1)
servoPwm.ChangeDutyCycle(0)

if __name__ == "__main__":
    while True:
        # Initialising variables
        lastDistanceFront, lastDistanceRight, lastDistanceLeft = 0, 0, 0
        distancesFront, distancesRight, distancesLeft = [], [], []
        distanceFront, distanceRight, distanceLeft = 0, 0, 0
        distanceFrontMeasured, distanceRightMeasured, distanceLeftMeasured = 0, 0, 0
        cornerCounter = 0
        distanceMeasurementIsUpToDate = False 
        lastDistanceMeasurementTime, currentDistanceMeasurementTime = time.time(), time.time()
        distanceRightChange, distanceLeftChange = 0, 0
        steeringBlocked, steeringBlocked2 = False, False
        measurementCounter = 0
        isRunning = False
        hasIncreased, hasDecreased = False, False
        steeringDirection = ""
        scriptStartTime = time.time()
        hasCorrectedLeft, hasCorrectedRight = False, False
        steeringEndTime = time.time()
        distanceFrontChange = 0
        stoppingEnabled, stoppingEnabledTime = False, time.time()
        redObjX, greenObjX = 0, 0
        distanceNearestObstacle = 0
        nearestObstacleX = 400
        nearestObstacleColor = ""
        avoidingRedObstacle, avoidingGreenObstacle = False, False
        avoidingStartTime = time.time()
        avoidingDistance = 0
        isObstacleRace = False
        frameCounter = 0
        justAvoidedObstacle, justAvoidedObstacle2 = False, False
        avoidingEndTime = time.time()
        avoidingDuration, avoidingDistance = 0, 0
        avoidingColor = ""
        avoidingAllowed = True
        cornerDirection = ""
        cornerAllowed = True
        backwardTime = scriptStartTime
        
        # Initialising variables for ultrasonic distance measurement
        pulseStartFront, pulseEndFront, pulseDurationFront = time.time(), time.time(), 0
        pulseStartRight, pulseEndRight, pulseDurationRight = time.time(), time.time(), 0
        pulseStartLeft, pulseEndLeft, pulseDurationLeft = time.time(), time.time(), 0
        distanceFront, distanceRight, distanceLeft = 0, 0, 0
        

        while True:
            # Starting if button is pressed
            if GPIO.input(buttonPin) == GPIO.HIGH:
                isRunning = True
                steerForward()
                time.sleep(0.5)
                forward(speed)
                scriptStartTime = time.time()
                break
        

        while isRunning:
            hasCorrectedLeft = False
            hasCorrectedRight = False
            

            # Stopping if button is pressed
            if GPIO.input(buttonPin) == GPIO.HIGH and (time.time() - scriptStartTime) > 2:
                isRunning = False
                steerForward()
                stop()
                time.sleep(0.2)
                servoPwm.ChangeDutyCycle(0)
                break
            

            # Getting measurements from ultrasonic distance sensors
            ultrasonicDistanceMeasurement()
            
            # Taking each three measurements from the ultrasonic distance sensors and forming the median of these
            if distanceFrontMeasured > 0 and distanceFrontMeasured < 300:
                distancesFront.append(distanceFrontMeasured)
                distanceMeasurementIsUpToDate = False
            if distanceRightMeasured > 0 and distanceRightMeasured < 300:
                distancesRight.append(distanceRightMeasured)
            if distanceLeftMeasured > 0 and distanceLeftMeasured < 300:
                distancesLeft.append(distanceLeftMeasured)
            if len(distancesFront) == 3:
                distancesFront.sort()
                lastDistanceFront = distanceFront
                distanceFront = round(distancesFront[1], 2)
                distancesFront = []
                distanceMeasurementIsUpToDate = True
                lastDistanceMeasurementTime = currentDistanceMeasurementTime
                currentDistanceMeasurementTime = time.time()
            if len(distancesRight) == 3:
                distancesRight.sort()
                lastDistanceRight = distanceRight
                distanceRight = round(distancesRight[1], 2)
                distancesRight = []
            if len(distancesLeft) == 3:
                distancesLeft.sort()
                lastDistanceLeft = distanceLeft
                distanceLeft = round(distancesLeft[1], 2)
                distancesLeft = []
            
            # Waiting until a new median of the measurements of the ultrasonic distance sensors is formed before allowing steering again
            if measurementCounter > 0:
                measurementCounter += 1
                if measurementCounter >= 4 and (time.time() - steeringStartTime) >= 0.5:
                    measurementCounter = 0
                    steeringBlocked2 = False
            

            # Reading camera image
            success, frame = cap.read()
            if not success:
                print("Can't receive frame. Exiting ...")
                break
            
            # Cropping camera image
            frame = frame[80:, 0:]
            
            # Calculating the distances to the nearest red and the nearest green obstacle
            redObjX, redObjHeightInFrame = objData(frame, lowerRed, upperRed, (0,0,255))
            if redObjHeightInFrame != 0:
                distanceRed = distanceCalculation(focalLength, knownHeight, redObjHeightInFrame)
                if not isObstacleRace:
                    isObstacleRace = True
                    print("Detected obstacle race")
                cv.putText(frame, f"distance: {round((distanceRed/10), 2)} cm", (35, 40), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
            else: 
                distanceRed = 0
            greenObjX, greenObjHeightInFrame = objData(frame, lowerGreen, upperGreen, (0,255,0))
            if greenObjHeightInFrame != 0:
                distanceGreen = distanceCalculation(focalLength, knownHeight, greenObjHeightInFrame)
                if not isObstacleRace:
                    isObstacleRace = True
                    print("Detected obstacle race")
                cv.putText(frame, f"distance: {round((distanceGreen/10), 2)} cm", (35, 80), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            else:
                distanceGreen = 0
                
            # Determining which obstacle is the nearest and its x-coordinate in the frame
            if distanceRed == 0 and distanceGreen == 0:
                distanceNearestObstacle = 0
                nearestObstacleX = 400
                nearestObstacleColor = ""
            elif distanceRed != 0 and distanceGreen != 0 and distanceRed < distanceGreen:
                distanceNearestObstacle = distanceRed
                nearestObstacleX = redObjX
                nearestObstacleColor = "red"
            elif distanceRed != 0 and distanceGreen != 0 and distanceGreen <= distanceRed:
                distanceNearestObstacle = distanceGreen
                nearestObstacleX = greenObjX
                nearestObstacleColor = "green"
            elif distanceRed == 0:
                distanceNearestObstacle = distanceGreen
                nearestObstacleX = greenObjX
                nearestObstacleColor = "green"
            else: 
                distanceNearestObstacle = distanceRed
                nearestObstacleX = redObjX
                nearestObstacleColor = "red"
            
            # Displaying frame (currently not used)
            # cv.imshow("Frame", frame)
            if cv.waitKey(1) & 0xFF == 27:
                break
            

            # Eventually resetting variables to allow certain actions
            if justAvoidedObstacle and not justAvoidedObstacle2:
                justAvoidedObstacle = False
            if not justAvoidedObstacle and justAvoidedObstacle2:
                justAvoidedObstacle2 = False
            
            if (time.time() - avoidingEndTime) > 1 or nearestObstacleColor != avoidingColor or distanceNearestObstacle > avoidingDistance and not avoidingAllowed:
                avoidingAllowed = True
            

            # Driving around obstacles
            if distanceRed != 0 and nearestObstacleColor == "red" and distanceRight > 25 and not steeringBlocked and avoidingAllowed and redObjX > 250:
                avoidingDuration = (distanceRight ** (1 / 3)) / 5
                if avoidingDuration > 0.75:
                    avoidingDuration = 0.75
                avoidingDuration = 0.4
                print("Now correcting right (red obstacle)")
                steerRight()
                time.sleep(avoidingDuration)
                steerForward()
                time.sleep(avoidingDuration)
                steerLeft()
                time.sleep(avoidingDuration / 1.3)
                steerForward()
                justAvoidedObstacle = True
                justAvoidedObstacle2 = True
                avoidingEndTime = time.time()
                avoidingDistance = distanceRed
                avoidingColor = "red"
            if distanceGreen != 0 and nearestObstacleColor == "green" and distanceLeft > 25 and not steeringBlocked and avoidingAllowed and greenObjX < 550:
                avoidingDuration = (distanceRight ** (1 / 3)) / 5
                if avoidingDuration > 0.75:
                    avoidingDuration = 0.75
                avoidingDuration = 0.4
                print("Now correcting left (green obstacle)")
                steerLeft()
                time.sleep(avoidingDuration)
                steerForward()
                time.sleep(avoidingDuration)
                steerRight()
                time.sleep(avoidingDuration / 1.3)
                steerForward()
                justAvoidedObstacle = True
                justAvoidedObstacle2 = True
                avoidingEndTime = time.time()
                avoidingDistance = distanceGreen
                avoidingColor = "green"

            if distanceGreen != 0 and nearestObstacleColor == "green" and distanceGreen < 300 and greenObjX < 550:
                steerForward()
                backward()
                time.sleep(0.5)
                forward()
                if steeringBlocked:
                    steeringEndTime = time.time()
                    steerForward()
                    print("Steering stopped")
                    hasIncreased = False
                    hasDecreased = False
                    steeringBlocked = False
                    steeringBlocked2 = False
                    steeringDirection = "forward"
                    cornerCounter += 1
                backwardTime = time.time()
            if distanceRed != 0 and nearestObstacleColor == "red" and distanceRed < 300 and redObjX < 550:
                steerForward()
                backward()
                time.sleep(0.5)
                forward()
                if steeringBlocked:
                    steeringEndTime = time.time()
                    steerForward()
                    print("Steering stopped")
                    hasIncreased = False
                    hasDecreased = False
                    steeringBlocked = False
                    steeringBlocked2 = False
                    steeringDirection = "forward"
                    cornerCounter += 1
                backwardTime = time.time()
            
            if cornerAllowed and (time.time() - backwardTime) < 1.5 and backwardTime != scriptStartTime:
                cornerAllowed = False
            if not cornerAllowed and (time.time() - backwardTime) < 1.5:
                cornerAllowed = True
            

            # Making a 90°-turn in the corners
            # Starting corner turn
            if cornerAllowed and distanceLeft > 130 and distanceFront < 80 and not steeringBlocked and not steeringBlocked2 and cornerCounter < 12 and (time.time() - steeringEndTime) >= 1.5 and cornerDirection != "right":
                steeringBlocked = True
                steeringBlocked2 = True
                print("\nNow steering left\n")
                steerLeft()
                steeringDirection = "left"
                measurementCounter = 1
                steeringStartTime = time.time()
                if cornerDirection == "":
                    cornerDirection = "left"
            if cornerAllowed and distanceRight > 130 and distanceFront < 80 and not steeringBlocked and not steeringBlocked2 and cornerCounter < 12 and (time.time() - steeringEndTime) >= 1.5 and cornerDirection != "left":
                steeringBlocked = True
                steeringBlocked2 = True
                print("\nNow steering right\n")
                steerRight()
                steeringDirection = "right"
                measurementCounter = 1
                steeringStartTime = time.time()
                if cornerDirection == "":
                    cornerDirection = "right"
                
            if steeringBlocked and not hasIncreased and round(lastDistanceFront) < (round(distanceFront) - 25):
                hasIncreased = True
            
            if steeringBlocked and hasIncreased and round(lastDistanceFront) > (round(distanceFront) - 10):
                hasDecreased = True
            
            # Stopping corner turn
            if steeringBlocked and hasIncreased and hasDecreased and not steeringBlocked2:
                steeringEndTime = time.time()
                if steeringDirection == "left":
                    steerForward()
                    print("Steering stopped")
                    hasIncreased = False
                    hasDecreased = False
                    steeringBlocked = False
                    steeringDirection = "forward"
                    cornerCounter += 1
                    if cornerCounter == 4:
                        forward(65)
                if steeringDirection == "right":
                    steerForward()
                    print("Steering stopped")
                    hasIncreased = False
                    hasDecreased = False
                    steeringBlocked = False
                    steeringDirection = "forward"
                    cornerCounter += 1
                    if cornerCounter == 4:
                        forward(65)
            if isObstacleRace and steeringBlocked and not steeringBlocked2:
                if distanceNearestObstacle != 0 and nearestObstacleX < 550 and nearestObstacleX > 250:
                    steeringEndTime = time.time()
                    steerForward()
                    print("Steering stopped")
                    hasIncreased = False
                    hasDecreased = False
                    steeringBlocked = False
                    steeringDirection = "forward"
                    cornerCounter += 1
                

            # Calculating the change of distanceRight and distanceLeft per second
            if (currentDistanceMeasurementTime - lastDistanceMeasurementTime) != 0 and distanceMeasurementIsUpToDate:
                distanceRightChange = (distanceRight - lastDistanceRight) / (currentDistanceMeasurementTime - lastDistanceMeasurementTime)
                distanceLeftChange = (distanceLeft - lastDistanceLeft) / (currentDistanceMeasurementTime - lastDistanceMeasurementTime)
            if abs(distanceRightChange) > 60:
                distanceRightChange = 0
            if abs(distanceLeftChange) > 60:
                distanceLeftChange = 0
                
            # Calculating the change of distanceFront per second
            if (currentDistanceMeasurementTime - lastDistanceMeasurementTime) != 0 and distanceMeasurementIsUpToDate:
                distanceFrontChange = (distanceFront - lastDistanceFront) / (currentDistanceMeasurementTime - lastDistanceMeasurementTime)
            
            # Preventing getting too close to a wall
            if (distanceLeft + distanceRight) < 110:
                # Correcting if the distance to the sides gets too small
                if (distanceRight < 20 and distanceLeft > 35 and (round(lastDistanceRight) > round(distanceRight) or isObstacleRace) and distanceMeasurementIsUpToDate and not steeringBlocked):
                    steerLeft()
                    print("Now correcting left (distanceRight too small)")
                    hasCorrectedLeft = True
                    time.sleep(0.15)
                    steerForward()
                if (distanceLeft < 20 and distanceRight > 35 and (round(lastDistanceLeft) > round(distanceLeft) or isObstacleRace) and distanceMeasurementIsUpToDate and not steeringBlocked):
                    steerRight()
                    print("Now correcting right (distanceLeft too small)")
                    hasCorrectedRight = True
                    time.sleep(0.15)
                    steerForward()
                    
                # Correcting if the distance to the sides changes too fast
                if distanceRightChange <= -3 and distanceMeasurementIsUpToDate and not steeringBlocked and distanceLeftChange >= 1.5 and distanceLeft >= distanceRight and not justAvoidedObstacle:
                    steerLeft()
                    print("Now correcting left (distanceChange)")
                    hasCorrectedLeft = True
                    time.sleep(0.1)
                    steerForward()
                if ‚distanceLeftChange <= -3 and distanceMeasurementIsUpToDate and not steeringBlocked and distanceRightChange >= 1.5 and distanceRight >= distanceLeft and not justAvoidedObstacle:
                    steerRight()
                    print("Now correcting right (distanceChange)")
                    hasCorrectedRight = True
                    time.sleep(0.1)
                    steerForward()
                    
                # Correcting if the distance to the front gets to small
                if distanceFront < 25 and distanceFront != 0 and not steeringBlocked and distanceMeasurementIsUpToDate and not isObstacleRace:
                    if distanceLeft > distanceRight:
                        steerLeft()
                        print("Now correcting left (distanceFront too small)")
                        hasCorrectedLeft = True
                        time.sleep(0.15)
                        steerForward()
                    else:
                        steerRight()
                        print("Now correcting right (distanceFront too small)")
                        hasCorrectedRight = True
                        time.sleep(0.15)
                        steerForward()
            

            # Stopping after three rounds
            if cornerCounter >= 12:
                if distanceFront > 150:
                    stoppingEnabled = True
                    stoppingEnabledTime = time.time()
                if stoppingEnabled and distanceFront < 150 and (time.time() - stoppingEnabledTime) >= 1.5:
                    stop()
                    steerForward()
                    time.sleep(0.25)
                    servoPwm.ChangeDutyCycle(0)
                    isRunning = False
                    print("\nDone!\n")


cap.release()
cv.destroyAllWindows()
