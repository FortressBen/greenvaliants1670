from spike import PrimeHub, LightMatrix, Button, StatusLight,MotionSensor, Speaker, ColorSensor, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
import hub as rawhub
import time
import math

hub = PrimeHub()

motor_left = Motor('B')
motor_right = Motor('A')
motor_front_left = Motor('C')
motor_front_right = Motor('D')
color = ColorSensor('F')
left_motor = rawhub.port.A.motor
right_motor = rawhub.port.B.motor
motor_pair = left_motor.pair(right_motor)

STOP_HOLD = left_motor.STOP_HOLD
STOP_BRAKE = left_motor.STOP_BRAKE
STOP_FLOAT = left_motor.STOP_FLOAT
motor_front_left.set_stop_action('brake')
hub.motion_sensor.reset_yaw_angle()
_select_trip = 0
MAX_SPEED = 100

##############################################################
# Tunable Constants
GYRO_TURN_FAST_SPEED = 20
GYRO_TURN_SLOW_SPEED = 8
BLACK_MIDDLE = 40
BLACK_EDGE = 50
SLOW_DOWN_ANGLE_BUFFER = 30
##############################################################

##############################################################
# Available Functions
# straight(degrees_to_move=500, speed=20)
# two_wheel_move(left_degrees=652, right_degrees=398, speed=20)
# acquire_line(speed=20)
# rot_motion()
# line_follower(move_degrees=1000, speed=20)
# gyro_turn(input_angle=90, relative=False, timeout=6, left_or_right=OneWheelTurn.BOTH)
# grind(left_speed=20, right_speed=20, run_seconds=3)
#vrooom()
###############################################################

class TurnType:
    BOTH = 0
    LEFT = 1
    RIGHT = 2

def motor_front_move():
    motor_front_left.set_degrees_counted(0)
    motor_front_right.set_degrees_counted(0)
    motor_front_left.run_for_degrees(-80, speed=MAX_SPEED)
    motor_front_right.run_for_degrees(4500, speed=MAX_SPEED)
    motor_front_right.run_for_degrees(-4500, speed=MAX_SPEED)

def tuning():
    grind()

def test_trip():
    gyro_turn(input_angle=90, relative=False, timeout=6, left_or_right=OneWheelTurn.RIGHT)

def the_trip_with_the_crates():
    two_wheel_move(left_degrees=362, right_degrees=272, speed=25)
    acquire_line(speed=20)
    line_follower(move_degrees=600, speed=20, gain=0.6)
    line_follower(move_degrees=750, speed=40, gain=0.2)
    straight(degrees_to_move=473, speed=25)
    gyro_turn(input_angle=180, relative=False, timeout=6, left_or_right=OneWheelTurn.RIGHT)
    rot_motion()

def the_trip_with_the_chest():
    gyro_turn(40, relative=False)

def the_one_with_the_crane():
    gyro_turn(45, relative=False)

def the_ending_trip():
    gyro_turn(120, relative=False)

###############################################################
# Utility Functions
def get_left_motor_degrees():
    return motor_left.get_degrees_counted() * -1

def get_right_motor_degrees():
    return motor_right.get_degrees_counted()

def wait_for_ms(millis):
    m = time.ticks_ms()
    end_ms = m + millis
    while time.ticks_ms() < end_ms:
        pass

def rela_to_abs(rela_angle):
    current_angle = hub.motion_sensor.get_yaw_angle()
    abs_angle = current_angle + rela_angle
    return abs_angle

def reverse(value):
    return -1 * (value)

def sign(input_value):
    if input_value < 0:
        return -1
    else:
        return 1

def is_within_tolerance(expected, actual, tolerance):
    if abs(abs(expected) - abs(actual)) <= tolerance:
        return True
###############################################################

def make_mark():
    motor_front_left.set_degrees_counted(0)
    motor_front_left.run_for_degrees(80, speed=80)
    motor_front_left.run_for_degrees(-80, speed=80)

def gyro_turn(input_angle = 90, relative = False, timeout = 6, left_or_right = TurnType.BOTH):
    STOP_AT_TARGET_TOLERANCE = 1
    def map_gyro_angle(x):
        modulus_x = x % 360
        if modulus_x >= 0 and modulus_x <= 180:
            return modulus_x
        else:
            return modulus_x - 360
    if relative == True:
        desired_angle = rela_to_abs(input_angle)
    else:
        desired_angle = input_angle

    def compute_sign_for_move(desired_angle):
        return -sign(desired_angle)

    sanitized_target_angle = map_gyro_angle(desired_angle)

    def turn_at_speed_until_tolerance(speed, tolerance_degrees):

        def at_desired_angle():
            abs_value_of_difference = abs(hub.motion_sensor.get_yaw_angle() - sanitized_target_angle)
            if abs_value_of_difference <= tolerance_degrees:
                return True

        sign = compute_sign_for_move(desired_angle)
        if left_or_right == TurnType.BOTH:
            motor_pair.run_at_speed(sign*speed, sign*speed)
        elif left_or_right == TurnType.LEFT:
            motor_pair.run_at_speed(0, sign*speed)
        else:
            motor_pair.run_at_speed(sign*speed, 0)
        wait_until(at_desired_angle)

    turn_at_speed_until_tolerance(GYRO_TURN_FAST_SPEED, SLOW_DOWN_ANGLE_BUFFER)
    turn_at_speed_until_tolerance(GYRO_TURN_SLOW_SPEED, STOP_AT_TARGET_TOLERANCE)
    motor_pair.brake()
    print("Gyro Turn Complete", hub.motion_sensor.get_yaw_angle())

def grind(left_speed=40, right_speed=20, run_seconds=3):
    grind_timer = Timer()

    def done_grinding():
        if grind_timer.now() >= run_seconds:
            return True

        if motor_left.was_stalled() and motor_right.was_stalled():
            return True

        return False

    motor_right.start_at_power(right_speed)
    motor_left.start_at_power(-left_speed)
    wait_until(done_grinding)
    motor_right.stop()
    motor_left.stop()
    print("Grind Complete")

def two_wheel_move(left_degrees=100, right_degrees=100, speed=30):
    MAX_POWER = 100
    ACCEL_MS_TO_FULL_SPEED = 600
    DECEL_MS_TO_FULL_SPEED = 1500
    motor_pair.preset(0,0)
    motor_pair.run_to_position(right_degrees, -left_degrees, speed, MAX_POWER, ACCEL_MS_TO_FULL_SPEED, DECEL_MS_TO_FULL_SPEED, stop=STOP_HOLD)

    def is_done():
        if is_within_tolerance(left_degrees, get_left_motor_degrees(), 3) and is_within_tolerance(right_degrees, get_right_motor_degrees(), 3):
            return True

    while not is_done():
        pass

    print("Two Wheel Move Complete")

def straight(degrees_to_move=500, speed=35):
    two_wheel_move(left_degrees=degrees_to_move,right_degrees=degrees_to_move, speed=speed)
    ("Straight Complete")

def rot_motion(print_seconds=3):
    rot_motion_timer = Timer()
    motor_left.set_degrees_counted(0)
    motor_right.set_degrees_counted(0)
    motor_front_left.set_degrees_counted(0)
    hub.motion_sensor.reset_yaw_angle()
    hub.speaker.beep(100, 1)

    while True:
        current_angle = hub.motion_sensor.get_yaw_angle()
        left_degrees_ran = motor_left.get_degrees_counted()
        right_degrees_ran = motor_right.get_degrees_counted()
        front_degrees_ran = motor_front_left.get_degrees_counted()
        if rot_motion_timer.now() >= print_seconds:
            print("gyro_turn(" + str(current_angle) + ", relative = True)")
            print("two_wheel_move(speed = 20, left_degrees = " + str(-left_degrees_ran) + ", right_degrees = " + str(right_degrees_ran) + ")")
            rot_motion_timer.reset()

        if hub.left_button.was_pressed() or hub.right_button.was_pressed():
            break

def acquire_line(speed=20):
    motor_pair.run_at_speed(speed,-speed)
    def color_reflected():
        if color.get_reflected_light() < BLACK_MIDDLE:
            current_color = color.get_reflected_light()
            return True
        else:
            return False

    wait_until(color_reflected)
    motor_pair.hold()
    hub.speaker.beep(90, 0.5)
    print("Acquire line Complete")

#def acquire_line(turn_speed=20):
    #def color_reflected():
        #gyro_turn(input_angle=-0.5, speed=turn_speed, timeout=6)
        #if color.get_reflected_light() < BLACK_MIDDLE:
            #current_color = color.get_reflected_light()
            #return True
        #else:
            #return False
    #wait_until(color_reflected())
    #motor_pair.hold()
    #hub.speaker.beep(90, 0.5)
    #print("Acquire line complete")
    
def line_follower(move_degrees=1000, speed=20, gain=0.2):
    KO = gain
    prop_gain_t = KO + (0.05/40) * (speed - 20)
    prop_gain = max(prop_gain_t, KO)
    inter_gain = 0
    sum_of_error = 0
    loop_counter = 0
    initial_degrees = abs(motor_right.get_degrees_counted())
    target_degrees = initial_degrees + abs(move_degrees)

    def more_degrees_to_go():
        current_degrees = abs(motor_right.get_degrees_counted())
        return current_degrees <= target_degrees

    while more_degrees_to_go():
        current_color = color.get_reflected_light()
        error = current_color - BLACK_EDGE
        sum_of_error += error
        adjusting_speed = prop_gain * error + inter_gain * sum_of_error
        left_motor_input = reverse(int(speed - adjusting_speed))
        right_motor_input = int(speed + adjusting_speed)
        motor_right.start_at_power(right_motor_input)
        motor_left.start_at_power(left_motor_input)
    motor_right.stop()
    motor_left.stop()
    print("Line follower Complete")

def vrooom():
    map_colors = {
        'blue': 1,
        'yellow': 2,
        'violet': 3,
        'cyan': 4,
        }

    map_trips = {
        1: the_trip_with_the_crates,
        2: the_trip_with_the_chest,
        3: the_one_with_the_crane,
        4: the_ending_trip
        }

    display_map = {
        1: 'CLOCK12',
        2: 'CLOCK3',
        3: 'CLOCK6',
        4: 'CLOCK9'
    }

    def increment_trip():
        select_trip(_select_trip + 1)

    def select_trip(new_trip):
        global _select_trip
        _select_trip = new_trip

        if _select_trip > 4:
            _select_trip = 1
        hub.light_matrix.show_image(display_map[_select_trip])

    def run_selected_trip():
        map_trips[_select_trip]()

    select_trip(1)

    current_color = None
    last_color = None

    while True:
        wait_for_seconds(0.5)
        current_color = color.get_color()

        if current_color in map_colors:
            color_to_run = map_colors[current_color]
            select_trip(color_to_run)

        left_pressed = hub.left_button.was_pressed()
        right_pressed = hub.right_button.was_pressed()
        if left_pressed and right_pressed:
            test_trip()
        else:
            if right_pressed:
                increment_trip()

            if left_pressed:
                run_selected_trip()

        last_color = current_color

#vrooom()
#the_trip_with_the_crates()
#rot_motion(print_seconds=3)
#motor_front_move()
test_trip()
raise SystemExit("END OF PROGRAM")
