import math
import time

import RPi.GPIO as GPIO
import cv2 as cv
import numpy as np

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

speed = 65

# HSV limits for color detection
lower_red = np.array([3, 155, 90])
upper_red = np.array([9, 255, 255])
lower_green = np.array([60, 147, 62])
upper_green = np.array([80, 255, 169])

# Variables for camera distance calculation
known_height = 100
focal_length = 567.0

# Initialising button, motors and ultrasonic distance sensors
button_pin = 40
ena, in1, in2 = 3, 5, 7
servo = 29
trig_front, echo_front = 13, 18
trig_right, echo_right = 11, 15
trig_left, echo_left = 12, 16

GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(ena, GPIO.OUT)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(servo, GPIO.OUT)
GPIO.setup(trig_front, GPIO.OUT)
GPIO.setup(echo_front, GPIO.IN)
GPIO.setup(trig_right, GPIO.OUT)
GPIO.setup(echo_right, GPIO.IN)
GPIO.setup(trig_left, GPIO.OUT)
GPIO.setup(echo_left, GPIO.IN)

motor_pwm = GPIO.PWM(ena, 100)
motor_pwm.start(0)
servo_pwm = GPIO.PWM(servo, 50)
servo_pwm.start(0)
GPIO.output(in1, GPIO.LOW)
GPIO.output(in2, GPIO.LOW)

# Initialising camera
cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()


# Function to move forward
def forward(x):
    motor_pwm.ChangeDutyCycle(x)
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)


# Function to move backward
def backward(x):
    motor_pwm.ChangeDutyCycle(x)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)


# Function to stop
def stop():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)


# Function to steer forward
def steer_forward():
    servo_pwm.ChangeDutyCycle(7.3)


# Function to steer left
def steer_left():
    servo_pwm.ChangeDutyCycle(9.3)


# Function to steer right
def steer_right():
    servo_pwm.ChangeDutyCycle(5.3)


# Function to get measurements from ultrasonic distance sensors
def ultrasonic_distance_measurement():
    # Initialising variables for ultrasonic distance measurement
    pulse_start_front, pulse_end_front, pulse_duration_front = time.time(), time.time(), 0
    pulse_start_right, pulse_end_right, pulse_duration_right = time.time(), time.time(), 0
    pulse_start_left, pulse_end_left, pulse_duration_left = time.time(), time.time(), 0
    global distance_front_measured, distance_right_measured, distance_left_measured

    # Triggering ultrasonic distance sensors
    GPIO.output(trig_front, GPIO.LOW)
    GPIO.output(trig_right, GPIO.LOW)
    GPIO.output(trig_left, GPIO.LOW)
    time.sleep(0.02)

    # Getting front distance
    GPIO.output(trig_front, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig_front, GPIO.LOW)
    while GPIO.input(echo_front) == 0:
        pulse_start_front = time.time()
    while GPIO.input(echo_front) == 1:
        pulse_end_front = time.time()
    pulse_duration_front = pulse_end_front - pulse_start_front
    distance_front_measured = pulse_duration_front * 17150
    distance_front_measured = round(distance_front_measured, 2)

    # Getting right distance
    GPIO.output(trig_right, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig_right, GPIO.LOW)
    while GPIO.input(echo_right) == 0:
        pulse_start_right = time.time()
    while GPIO.input(echo_right) == 1:
        pulse_end_right = time.time()
    pulse_duration_right = pulse_end_right - pulse_start_right
    distance_right_measured = pulse_duration_right * 17150
    distance_right_measured = round(distance_right_measured, 2)

    # Getting left distance
    GPIO.output(trig_left, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig_left, GPIO.LOW)
    while GPIO.input(echo_left) == 0:
        pulse_start_left = time.time()
    while GPIO.input(echo_left) == 1:
        pulse_end_left = time.time()
    pulse_duration_left = pulse_end_left - pulse_start_left
    distance_left_measured = pulse_duration_left * 17150
    distance_left_measured = round(distance_left_measured, 2)


# Function to get the height and x-coordinate in the frame of the biggest object within the HSV limits
def obj_data(frame, lower, upper, color):
    x, w, h = 400, 0, 0
    min_area = 600
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower, upper)
    _, mask1 = cv.threshold(mask, 254, 255, cv.THRESH_BINARY)
    contours, _ = cv.findContours(mask1, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    for contour in contours:
        if cv.contourArea(contour) > min_area:
            x, y, w, h = cv.boundingRect(contour)
            cv.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            min_area = cv.contourArea(contour)
    return (x + w / 2), h


# Function to calculate the distance of an object in the camera frame
def distance_calculation(focal_length, known_height, obj_height_in_frame):
    distance = (known_height * focal_length) / obj_height_in_frame
    horizontal_distance = math.sqrt(distance ** 2 - 81 ** 2)
    return horizontal_distance


stop()
steer_forward()
time.sleep(0.1)
servo_pwm.ChangeDutyCycle(0)

if __name__ == "__main__":
    while True:
        # Initialising variables
        last_distance_front, last_distance_right, last_distance_left = 0, 0, 0
        distances_front, distances_right, distances_left = [], [], []
        distance_front_measured, distance_right_measured, distance_left_measured = 0, 0, 0
        corner_counter = 0
        distance_measurement_is_up_to_date = False
        last_distance_measurement_time, current_distance_measurement_time = time.time(), time.time()
        distance_right_change, distance_left_change = 0, 0
        steering_blocked, steering_blocked2 = False, False
        measurement_counter = 0
        is_running = False
        has_increased, has_decreased = False, False
        steering_direction = ""
        script_start_time = time.time()
        has_corrected_left, has_corrected_right = False, False
        steering_end_time = time.time()
        distance_front_change = 0
        stopping_enabled, stopping_enabled_time = False, time.time()
        red_obj_x, green_obj_x = 0, 0
        distance_nearest_obstacle = 0
        nearest_obstacle_x = 400
        nearest_obstacle_color = ""
        avoiding_red_obstacle, avoiding_green_obstacle = False, False
        avoiding_start_time = time.time()
        avoiding_distance = 0
        is_obstacle_race = False
        frame_counter = 0
        just_avoided_obstacle, just_avoided_obstacle2 = False, False
        avoiding_end_time = time.time()
        avoiding_duration = 0
        avoiding_color = ""
        avoiding_allowed = True
        corner_direction = ""
        corner_allowed = True
        backward_time = script_start_time
        pulse_start_front, pulse_end_front, pulse_duration_front = time.time(), time.time(), 0
        pulse_start_right, pulse_end_right, pulse_duration_right = time.time(), time.time(), 0
        pulse_start_left, pulse_end_left, pulse_duration_left = time.time(), time.time(), 0
        distance_front, distance_right, distance_left = 0, 0, 0
        steering_start_time = time.time()
        red_obj_height_in_frame, green_obj_height_in_frame = 0, 0
        distance_red, distance_green = 0, 0

        while True:
            # Starting if button is pressed
            if GPIO.input(button_pin) == GPIO.HIGH:
                is_running = True
                steer_forward()
                time.sleep(0.5)
                forward(speed)
                script_start_time = time.time()
                break

        while is_running:
            has_corrected_left = False
            has_corrected_right = False

            # Stopping if button is pressed
            if GPIO.input(button_pin) == GPIO.HIGH and (time.time() - script_start_time) > 2:
                is_running = False
                steer_forward()
                stop()
                time.sleep(0.2)
                servo_pwm.ChangeDutyCycle(0)
                break

            # Getting measurements from ultrasonic distance sensors
            ultrasonic_distance_measurement()

            # Taking each three measurements from the ultrasonic distance sensors and forming the median of these
            if 0 < distance_front_measured < 300:
                distances_front.append(distance_front_measured)
                distance_measurement_is_up_to_date = False
            if 0 < distance_right_measured < 300:
                distances_right.append(distance_right_measured)
            if 0 < distance_left_measured < 300:
                distances_left.append(distance_left_measured)
            if len(distances_front) == 3:
                distances_front.sort()
                last_distance_front = distance_front
                distance_front = round(distances_front[1], 2)
                distances_front = []
                distance_measurement_is_up_to_date = True
                last_distance_measurement_time = current_distance_measurement_time
                current_distance_measurement_time = time.time()
            if len(distances_right) == 3:
                distances_right.sort()
                last_distance_right = distance_right
                distance_right = round(distances_right[1], 2)
                distances_right = []
            if len(distances_left) == 3:
                distances_left.sort()
                last_distance_left = distance_left
                distance_left = round(distances_left[1], 2)
                distances_left = []

            # Waiting until a new median of the measurements of the ultrasonic distance sensors is formed before allowing steering again
            if measurement_counter > 0:
                measurement_counter += 1
                if measurement_counter >= 4 and (time.time() - steering_start_time) >= 0.5:
                    measurement_counter = 0
                    steering_blocked2 = False

            # Reading camera image
            success, frame = cap.read()
            if not success:
                print("Can't receive frame. Exiting ...")
                break

            # Cropping camera image
            frame = frame[80:, 0:]

            # Calculating the distances to the nearest red and the nearest green obstacle
            red_obj_x, red_obj_height_in_frame = obj_data(frame, lower_red, upper_red, (0, 0, 255))
            if red_obj_height_in_frame != 0:
                distance_red = distance_calculation(focal_length, known_height, red_obj_height_in_frame)
                if not is_obstacle_race:
                    is_obstacle_race = True
                    print("Detected obstacle race")
                cv.putText(frame, f"distance: {round((distance_red / 10), 2)} cm", (35, 40), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                distance_red = 0
            green_obj_x, green_obj_height_in_frame = obj_data(frame, lower_green, upper_green, (0, 255, 0))
            if green_obj_height_in_frame != 0:
                distance_green = distance_calculation(focal_length, known_height, green_obj_height_in_frame)
                if not is_obstacle_race:
                    is_obstacle_race = True
                    print("Detected obstacle race")
                cv.putText(frame, f"distance: {round((distance_green / 10), 2)} cm", (35, 80), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                distance_green = 0

            # Determining which obstacle is the nearest and its x-coordinate in the frame
            if distance_red == 0 and distance_green == 0:
                distance_nearest_obstacle = 0
                nearest_obstacle_x = 400
                nearest_obstacle_color = ""
            elif distance_red != 0 and distance_green != 0 and distance_red < distance_green:
                distance_nearest_obstacle = distance_red
                nearest_obstacle_x = red_obj_x
                nearest_obstacle_color = "red"
            elif distance_red != 0 and distance_green != 0 and distance_green <= distance_red:
                distance_nearest_obstacle = distance_green
                nearest_obstacle_x = green_obj_x
                nearest_obstacle_color = "green"
            elif distance_red == 0:
                distance_nearest_obstacle = distance_green
                nearest_obstacle_x = green_obj_x
                nearest_obstacle_color = "green"
            else:
                distance_nearest_obstacle = distance_red
                nearest_obstacle_x = red_obj_x
                nearest_obstacle_color = "red"

            # Displaying frame (currently not used)
            # cv.imshow("Frame", frame)
            if cv.waitKey(1) & 0xFF == 27:
                break

            # Eventually resetting variables to allow certain actions
            if just_avoided_obstacle and not just_avoided_obstacle2:
                just_avoided_obstacle = False
            if not just_avoided_obstacle and just_avoided_obstacle2:
                just_avoided_obstacle2 = False

            if ((time.time() - avoiding_end_time) > 1 or nearest_obstacle_color != avoiding_color or distance_nearest_obstacle > avoiding_distance) and not avoiding_allowed:
                avoiding_allowed = True

            # Driving around obstacles
            if distance_red != 0 and nearest_obstacle_color == "red" and distance_right > 25 and not steering_blocked and avoiding_allowed and red_obj_x > 250 and distance_red < 500:
                avoiding_duration = 0.65
                print("Now correcting right (red obstacle)")
                steer_forward()
                backward(speed)
                time.sleep(0.8)
                forward(speed)
                time.sleep(0.3)
                steer_right()
                time.sleep(avoiding_duration)
                steer_forward()
                time.sleep(avoiding_duration)
                steer_left()
                time.sleep(avoiding_duration / 2)
                steer_forward()
                just_avoided_obstacle = True
                just_avoided_obstacle2 = True
                avoiding_end_time = time.time()
                avoiding_distance = distance_red
                avoiding_color = "red"
            if distance_green != 0 and nearest_obstacle_color == "green" and distance_left > 25 and not steering_blocked and avoiding_allowed and green_obj_x < 550 and distance_green < 500:
                avoiding_duration = 0.65
                print("Now correcting left (green obstacle)")
                steer_forward()
                backward(speed)
                time.sleep(0.8)
                forward(speed)
                time.sleep(0.3)
                steer_left()
                time.sleep(avoiding_duration)
                steer_forward()
                time.sleep(avoiding_duration)
                steer_right()
                time.sleep(avoiding_duration / 2)
                steer_forward()
                just_avoided_obstacle = True
                just_avoided_obstacle2 = True
                avoiding_end_time = time.time()
                avoiding_distance = distance_green
                avoiding_color = "green"

            if (distance_green != 0 and nearest_obstacle_color == "green" and distance_green < 300 and green_obj_x < 600) or (distance_red != 0 and nearest_obstacle_color == "red" and distance_red < 300 and red_obj_x > 200):
                steer_forward()
                backward(speed)
                time.sleep(0.5)
                forward(speed)
                if steering_blocked:
                    steering_end_time = time.time()
                    steer_forward()
                    print("Steering stopped")
                    has_increased = False
                    has_decreased = False
                    steering_blocked = False
                    steering_blocked2 = False
                    steering_direction = "forward"
                    corner_counter += 1
                backward_time = time.time()

            if corner_allowed and (time.time() - backward_time) < 1.5 and backward_time != script_start_time:
                corner_allowed = False
            if not corner_allowed and (time.time() - backward_time) < 1.5:
                corner_allowed = True

            # Making a 90°-turn in the corners
            # Starting corner turn
            if corner_allowed and distance_left > 130 and distance_front < 80 and not steering_blocked and not steering_blocked2 and corner_counter < 12 and (time.time() - steering_end_time) >= 1.5 and corner_direction != "right":
                steering_blocked = True
                steering_blocked2 = True
                print("\nNow steering left\n")
                steer_left()
                steering_direction = "left"
                measurement_counter = 1
                steering_start_time = time.time()
                if corner_direction == "":
                    corner_direction = "left"
            if corner_allowed and distance_right > 130 and distance_front < 80 and not steering_blocked and not steering_blocked2 and corner_counter < 12 and (time.time() - steering_end_time) >= 1.5 and corner_direction != "left":
                steering_blocked = True
                steering_blocked2 = True
                print("\nNow steering right\n")
                steer_right()
                steering_direction = "right"
                measurement_counter = 1
                steering_start_time = time.time()
                if corner_direction == "":
                    corner_direction = "right"

            if steering_blocked and not has_increased and round(last_distance_front) < (round(distance_front) - 25):
                has_increased = True

            if steering_blocked and has_increased and round(last_distance_front) > (round(distance_front) - 10):
                has_decreased = True

            # Stopping corner turn
            if steering_blocked and has_increased and has_decreased and not steering_blocked2:
                steering_end_time = time.time()
                if steering_direction == "left":
                    steer_forward()
                    print("Steering stopped")
                    has_increased = False
                    has_decreased = False
                    steering_blocked = False
                    steering_direction = "forward"
                    corner_counter += 1
                    if corner_counter == 4:
                        forward(65)
                if steering_direction == "right":
                    steer_forward()
                    print("Steering stopped")
                    has_increased = False
                    has_decreased = False
                    steering_blocked = False
                    steering_direction = "forward"
                    corner_counter += 1
                    if corner_counter == 4:
                        forward(65)
            if is_obstacle_race and steering_blocked and not steering_blocked2:
                if distance_nearest_obstacle != 0 and 550 > nearest_obstacle_x > 250:
                    steering_end_time = time.time()
                    steer_forward()
                    print("Steering stopped")
                    has_increased = False
                    has_decreased = False
                    steering_blocked = False
                    steering_direction = "forward"
                    corner_counter += 1

            # Calculating the change of distance_front, distance_right and distance_left per second
            if (current_distance_measurement_time - last_distance_measurement_time) != 0 and distance_measurement_is_up_to_date:
                distance_front_change = (distance_front - last_distance_front) / (current_distance_measurement_time - last_distance_measurement_time)
                distance_right_change = (distance_right - last_distance_right) / (current_distance_measurement_time - last_distance_measurement_time)
                distance_left_change = (distance_left - last_distance_left) / (current_distance_measurement_time - last_distance_measurement_time)
            if abs(distance_right_change) > 60:
                distance_right_change = 0
            if abs(distance_left_change) > 60:
                distance_left_change = 0

            # Preventing getting too close to a wall
            if (distance_left + distance_right) < 110:
                # Correcting if the distance to the sides gets too small
                if distance_right < 20 and distance_left > 35 and distance_measurement_is_up_to_date and not steering_blocked and (round(last_distance_right) > round(distance_right) or is_obstacle_race):
                    steer_left()
                    print("Now correcting left (distance_right too small)")
                    has_corrected_left = True
                    time.sleep(0.15)
                    steer_forward()
                if distance_left < 20 and distance_right > 35 and distance_measurement_is_up_to_date and not steering_blocked and (round(last_distance_left) > round(distance_left) or is_obstacle_race):
                    steer_right()
                    print("Now correcting right (distance_left too small)")
                    has_corrected_right = True
                    time.sleep(0.15)
                    steer_forward()

                # Correcting if the distance to the sides changes too fast
                if distance_right_change <= -3 and distance_measurement_is_up_to_date and not steering_blocked and distance_left_change >= 1.5 and distance_left >= distance_right and not just_avoided_obstacle:
                    steer_left()
                    print("Now correcting left (distanceChange)")
                    has_corrected_left = True
                    time.sleep(0.1)
                    steer_forward()
                if distance_left_change <= -3 and distance_measurement_is_up_to_date and not steering_blocked and distance_right_change >= 1.5 and distance_right >= distance_left and not just_avoided_obstacle:
                    steer_right()
                    print("Now correcting right (distanceChange)")
                    has_corrected_right = True
                    time.sleep(0.1)
                    steer_forward()

                # Correcting if the distance to the front gets to small
                if distance_front < 25 and distance_front != 0 and not steering_blocked and distance_measurement_is_up_to_date and not is_obstacle_race:
                    if distance_left > distance_right:
                        steer_left()
                        print("Now correcting left (distance_front too small)")
                        has_corrected_left = True
                        time.sleep(0.15)
                        steer_forward()
                    else:
                        steer_right()
                        print("Now correcting right (distance_front too small)")
                        has_corrected_right = True
                        time.sleep(0.15)
                        steer_forward()

            # Stopping after three rounds
            if corner_counter >= 12:
                if distance_front > 150:
                    stopping_enabled = True
                    stopping_enabled_time = time.time()
                if stopping_enabled and distance_front < 165 and (time.time() - stopping_enabled_time) >= 1.2:
                    stop()
                    steer_forward()
                    time.sleep(0.25)
                    servo_pwm.ChangeDutyCycle(0)
                    is_running = False
                    print("\nDone!\n")

cap.release()
cv.destroyAllWindows()
