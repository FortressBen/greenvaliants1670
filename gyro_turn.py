from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

hub = PrimeHub() 
motors = MotorPair('A' , 'E')
motorLeft = Motor('A')
motorRight = Motor('E')
motorBack = Motor('C')
motorFront = Motor('D')
color = ColorSensor('F')
motors.set_motor_rotation(13.75 * 2, 'cm')
motors.set_default_speed(30)
motors.set_stop_action('hold')
motorFront.set_stop_action('brake')
hub.motion_sensor.reset_yaw_angle()
_selectTrip = 0

def vrooom():
    

    def incrementTrip():
        selectTrip(_selectTrip + 1)
    def selectTrip(newTrip):
        global _selectTrip
        _selectTrip = newTrip

        if _selectTrip > 4:
            _selectTrip = 1
        hub.light_matrix.write(str(_selectTrip))
        

    def runSelectedTrip():
        if _selectTrip == 1:
            trip1()
        elif _selectTrip == 2:
            trip2()
        elif _selectTrip == 3:
            trip3()

    selectTrip(1)

    currentColor = None
    lastColor = None
    while True:
        currentColor = color.get_color()

        if currentColor != color.get_color():
            print(color.get_color())
        else:
            pass
        if currentColor == 'blue':
            selectTrip(1)
        elif currentColor == 'yellow':
            selectTrip(2)
        elif currentColor == 'violet':
            selectTrip(3)
        if hub.right_button.was_pressed():
            incrementTrip()
        if hub.left_button.was_pressed():
            runSelectedTrip()
        
        lastColor = currentColor













def test_robot():
    straight(degreesToMove=100,speed=25)
    gyro_turn(input_angle = 45, relative= False)
    two_wheel_move(speed = 30,leftDegrees = 500,rightDegrees = 315)
    grind(Leftspeed = 20,Rightspeed = 20, runSeconds = 3)
    rotMotion(printSeconds = 3)

def trip1():
    gyro_turn(90, relative= False)

def trip2():
    gyro_turn(45, relative= False)

def trip3():
    gyro_turn(120, relative= False)



def rela_to_abs(rela_angle):
    current_angle = hub.motion_sensor.get_yaw_angle()
    abs_angle = current_angle + rela_angle
    return abs_angle

def reverse(value):
    return -1 * (value)

def makeMark():
    motorFront.set_degrees_counted(0)
    motorFront.run_for_degrees(360, speed=80)

def map_gyro_angle(x):
    modulusX = x % 360
    if modulusX >= 0 and modulusX <= 180:
        return modulusX
    else:
        return modulusX - 360

def sign(inputValue):
    if inputValue < 0:
        return -1
    else:
        return 1

def gyro_turn(input_angle = 90, relative = False):
    SLOW_DOWN_ANGLE_BUFFER = 25
    STOP_AT_TARGET_TOLERANCE = 1
    FAST_SPEED = 60
    SLOW_SPEED = 20
    MAX_TURN_STEERING= 100
    if relative == True:
        desired_angle = rela_to_abs(input_angle)
    else:
        desired_angle = input_angle

    def compute_steering_for_move(desired_angle):
        STEERING_MAX_TURN = 100
        return sign(desired_angle)*STEERING_MAX_TURN
    
    sanitized_target_angle = map_gyro_angle(desired_angle)

    def turn_at_speed_until_tolerence(speed, tolerance_degrees):

        def at_desired_angle():
            absValueOfDifference = abs(hub.motion_sensor.get_yaw_angle() - sanitized_target_angle)
            if absValueOfDifference <= tolerance_degrees:
                return True

        steering = compute_steering_for_move(sanitized_target_angle)
        motors.start_at_power(speed,steering)
        wait_until(at_desired_angle)

        motors.stop()

    turn_at_speed_until_tolerence(FAST_SPEED,SLOW_DOWN_ANGLE_BUFFER)
    turn_at_speed_until_tolerence(SLOW_SPEED,STOP_AT_TARGET_TOLERANCE)


def grind(Leftspeed=20, Rightspeed = 20, runSeconds=3):

    grindTimer = Timer()
    def done_grinding():
        if grindTimer.now() >= runSeconds:
            return True

        if motorLeft.was_stalled() and motorRight.was_stalled():
            return True

        return False

    motorRight.start_at_power(Rightspeed)
    motorLeft.start_at_power(-Leftspeed)
    wait_until(done_grinding)
    motorRight.stop()
    motorLeft.stop()

def two_wheel_move(speed, leftDegrees, rightDegrees):
    l = 2 * speed
    la = rightDegrees / leftDegrees
    lav = la + 1
    sL = l / lav
    r = 2 * speed
    ra = leftDegrees / rightDegrees
    rav = ra + 1
    sR = r / rav
    intsL = int(sL)
    intsR = int(sR)
    def check_degrees_counted():
        motorLeft.set_degrees_counted(0)
        motorRight.set_degrees_counted(0)
        BothMotorsStillRunning = True
        rightMotorStopped = False
        leftMotorStopped = False
        while BothMotorsStillRunning == True:
            leftDegreesRan = -motorLeft.get_degrees_counted()
            rightDegreesRan = motorRight.get_degrees_counted()
            if leftDegreesRan >= leftDegrees:
                motorLeft.stop()
                leftMotorStopped = True
            else:
                pass
            if rightDegreesRan >= rightDegrees:
                motorRight.stop()
                rightMotorStopped = True
            else:
                pass
            if rightMotorStopped and leftMotorStopped:
                BothMotorsStillRunning = False
    motorLeft.start(-intsL)
    motorRight.start(intsR)
    check_degrees_counted()

def straight(degreesToMove = 500, speed=35):
    two_wheel_move(speed, degreesToMove, degreesToMove)

def rotMotion(printSeconds = 3):
    rotMotionTimer = Timer()
    motorLeft.set_degrees_counted(0)
    motorRight.set_degrees_counted(0)
    motorFront.set_degrees_counted(0)
    hub.motion_sensor.reset_yaw_angle()
    while True:
        current_angle = hub.motion_sensor.get_yaw_angle()
        leftDegreesRan = motorLeft.get_degrees_counted()
        rightDegreesRan = motorRight.get_degrees_counted()
        frontDegreesRan = motorFront.get_degrees_counted()
        if rotMotionTimer.now() >= printSeconds:
            print("gyro_turn(" + str(current_angle) + ")")
            print("two_wheel_move(" + str(leftDegreesRan) + "," + str(rightDegreesRan) + ")")
            rotMotionTimer.reset()
        if hub.left_button.was_pressed() or hub.right_button.was_pressed():
            break

def acquire_line(speed):
    BLACK_MIDDLE = 20
    motors.start_at_power(speed)
    def colorReflected():
        if color.get_reflected_light() < BLACK_MIDDLE:
            return True
        else: 
            return False
    
    wait_until(colorReflected)
    motors.stop()

def line_follower(move_degrees, speed=20):
    BLACK_THRESHOLD = 64
    KP = 0.2
    KI = 0
    sum_of_error = 0
    loopCounter = 0
    initial_degrees = abs(motorRight.get_degrees_counted())
    target_degrees = initial_degrees + abs(move_degrees)

    def more_degrees_to_go():
        current_degrees = abs(motorRight.get_degrees_counted())
        return current_degrees <= target_degrees

    while more_degrees_to_go():
        currentColor = color.get_reflected_light()
        error = currentColor - BLACK_THRESHOLD
        sum_of_error += error
        adjustingSpeed = KP * error + KI * sum_of_error
        leftMotorInput = reverse(int(speed - adjustingSpeed))
        rightMotorInput = int(speed + adjustingSpeed)
        motorRight.start_at_power(rightMotorInput)
        motorLeft.start_at_power(leftMotorInput)
    motorRight.stop()
    motorLeft.stop()

vrooom()


motors.set_stop_action('coast')
motors.stop()
raise SystemExit