from spike import PrimeHub, LightMatrix, Button, StatusLight,MotionSensor, Speaker, ColorSensor, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from hub import battery
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
MAX_SPEED = 80
##############################################################
# Tunable Constants
GYRO_TURN_FAST_SPEED = 20
GYRO_TURN_SLOW_SPEED = 7
BLACK_MIDDLE = 30
BLACK_EDGE = 55
SLOW_DOWN_ANGLE_BUFFER = 30
MIN_POWER_TO_MOVE = 11
POST_MOVE_WAIT_MS = 500
##############################################################
##############################################################
# Available Functions
# straight(degrees_to_move=500, speed=20)
# two_wheel_move(left_degrees=652, right_degrees=398, speed=20)
# acquire_line(speed=20)
# rot_motion(print_seconds=3)
# line_follower(move_degrees=1000, speed=20)
# gyro_turn(input_angle=90, relative=False, timeout=6, left_or_right=TurnType.BOTH)
# grind(left_speed=20, right_speed=20, run_seconds=3)
# turn_until_line(left_or_right=TurnType.RIGHT)
# vrooom()
###############################################################
class TurnType:
    BOTH = 0
    LEFT = 1
    RIGHT = 2

class TurnDirection:
    CLOCKWISE = -1
    COUNTERCLOCKWISE = 1


def test_gyro_turn():
    hub.motion_sensor.reset_yaw_angle()
    TIMEOUT_SECS=4
    def run_turns(list_of_turns):
        for tt in list_of_turns:
            gyro_turn_2(input_angle = tt[0], relative = tt[1], timeout = TIMEOUT_SECS, left_or_right = TurnType.BOTH, counter_or_clock= tt[2])
            wait_for_seconds(2)


    run_turns([
        (45,False,TurnDirection.CLOCKWISE),
        (179,False,TurnDirection.CLOCKWISE),
        (246,False,TurnDirection.CLOCKWISE),
        (-45,False,TurnDirection.CLOCKWISE),
        (45,False,TurnDirection.CLOCKWISE),
        (246,False,TurnDirection.CLOCKWISE),
        (45,False,TurnDirection.CLOCKWISE),
        (-45,False,TurnDirection.CLOCKWISE),
        (179,False,TurnDirection.CLOCKWISE),
        (-45,False,TurnDirection.CLOCKWISE),
        (246,False,TurnDirection.CLOCKWISE),
        (179,False,TurnDirection.CLOCKWISE),
        (45,False,TurnDirection.CLOCKWISE),
        (45,False,TurnDirection.COUNTERCLOCKWISE),
        (179,False,TurnDirection.COUNTERCLOCKWISE),
        (246,False,TurnDirection.COUNTERCLOCKWISE),
        (-45,False,TurnDirection.COUNTERCLOCKWISE),
        (45,False,TurnDirection.COUNTERCLOCKWISE),
        (246,False,TurnDirection.COUNTERCLOCKWISE),
        (45,False,TurnDirection.COUNTERCLOCKWISE),
        (-45,False,TurnDirection.COUNTERCLOCKWISE),
        (179,False,TurnDirection.COUNTERCLOCKWISE),
        (-45,False,TurnDirection.COUNTERCLOCKWISE),
        (246,False,TurnDirection.COUNTERCLOCKWISE),
        (179,False,TurnDirection.COUNTERCLOCKWISE),
        (45,False,TurnDirection.COUNTERCLOCKWISE)
    ])


def tuning():
    grind(left_speed=-40, right_speed=-40, timeout=6, run_seconds=3)

def test_motors_up_down():
    motor_front_left.set_degrees_counted(0)
    motor_front_right.set_degrees_counted(0)
    motor_front_left.run_for_degrees(-80, speed=MAX_SPEED)
    motor_front_right.run_for_degrees(4500, speed=MAX_SPEED)
    motor_front_right.run_for_degrees(-4500, speed=MAX_SPEED)

def test_trip():
    grind(left_speed=-30, right_speed=-30, run_seconds=23)

def the_trip_with_the_crates():
    grind(left_speed=-20,right_speed=-20, run_seconds=0.5)
    two_wheel_move(left_degrees=624, right_degrees=475, speed=30)
    two_wheel_move(left_degrees=519, right_degrees=471, speed=30)
    motor_left.set_degrees_counted(0)
    motor_right.set_degrees_counted(0)
    turn_until_line(left_or_right=TurnType.LEFT)
    print("aquire_line",motor_left.get_degrees_counted())
    print("aquire_line",motor_right.get_degrees_counted())
    wait_for_ms(100)
    motor_left.run_for_degrees(-40,30)
    line_follower(move_degrees=660, speed=35, gain=0.19)
    hub.speaker.beep(100, 0.125)
    line_follower(move_degrees=180, speed=20, gain=0.25)
    hub.speaker.beep(100, 0.125)
    line_follower(move_degrees=390, speed=35, gain=0.19)
    motor_left.set_degrees_counted(0)
    motor_right.set_degrees_counted(0)
    motor_left.run_for_degrees(-432,30) #original -429
    two_wheel_move(left_degrees=-480, right_degrees=-480, speed=30)
    grind(left_speed=-40, right_speed=-40, run_seconds=0.5)
    two_wheel_move(left_degrees=395, right_degrees=395, speed=30)
    wait_for_ms(150)
    two_wheel_move(left_degrees=395, right_degrees=395, speed=30)
    grind(left_speed=20,right_speed=20,run_seconds=0.75)
    motor_front_right.run_for_degrees(3300, speed=MAX_SPEED)
    motor_front_right.run_for_seconds(0.1,90)
    motor_front_left.run_for_seconds(1,-MAX_SPEED)
    motor_front_left.run_for_seconds(1,MAX_SPEED)
    motor_front_right.run_for_degrees(-3300, speed=MAX_SPEED)
    grind(left_speed=-25,right_speed=-25, run_seconds=0.5)
    hub.speaker.beep(100, 0.125)
    acquire_line(speed=-20)
    hub.speaker.beep(100, 0.125)
    motor_left.run_for_degrees(-50,20)
    gyro_turn_2(input_angle=252, relative=False, timeout=2, left_or_right=TurnType.BOTH, counter_or_clock=TurnDirection.CLOCKWISE)
    hub.speaker.beep(100, 0.125)
    motor_front_left.run_for_seconds(1,-MAX_SPEED)
    two_wheel_move(left_degrees=884, right_degrees=836, speed=35)
    two_wheel_move(left_degrees=452, right_degrees=519, speed=35)
    motor_front_left.run_for_seconds(1,MAX_SPEED)
    two_wheel_move(left_degrees=1534, right_degrees=1280, speed=45)

def the_trip_with_the_chest():
    motor_front_left.run_for_seconds(seconds=0.1,speed=10)
    motor_front_left.set_degrees_counted(0)
    motor_front_right.set_degrees_counted(0)
    grind(left_speed=-40, right_speed=-40, run_seconds=0.3)
    motor_front_left.run_for_degrees(45, speed=40)
    two_wheel_move(left_degrees=963, right_degrees=919, speed=40)
    two_wheel_move(left_degrees=38, right_degrees=-38, speed=20)
    two_wheel_move(left_degrees=85, right_degrees=85, speed=40)
    motor_front_left.run_for_degrees(120, speed=40)
    motor_front_left.run_for_degrees(-140, speed=40)
    two_wheel_move(left_degrees=65, right_degrees=65, speed=30)
    motor_front_left.run_for_degrees(50, speed=40)
    motor_front_left.run_for_degrees(-80,speed=40)
    two_wheel_move(left_degrees=130, right_degrees=150, speed=30)
    two_wheel_move(left_degrees=66, right_degrees=3, speed=30)
    motor_front_right.run_for_degrees(-80, speed=30)
    two_wheel_move(left_degrees=-220, right_degrees=-242, speed=30)
    grind(left_speed=-50, right_speed=-50, run_seconds=3)
    motor_front_right.run_for_degrees(-200, speed=40)

def the_trip_with_the_crane():
    grind(left_speed=-20,right_speed=-20, run_seconds=0.5)
    two_wheel_move(left_degrees=624, right_degrees=475, speed=30)
    two_wheel_move(left_degrees=519, right_degrees=486, speed=30)
    turn_until_line(left_or_right=TurnType.LEFT)
    line_follower(move_degrees=590, speed=35, gain=0.19)
    hub.speaker.beep(100, 0.125)
    two_wheel_move(left_degrees=25, right_degrees=42, speed=15)
    two_wheel_move(left_degrees=-145, right_degrees=145, speed=MIN_POWER_TO_MOVE)
    two_wheel_move(left_degrees=254, right_degrees=246, speed=18)
    grind(left_speed=25, right_speed=25, run_seconds=0.125)
    hub.speaker.beep(101, 0.25)
    grind(left_speed=25, right_speed=-15, run_seconds=0.625)
    hub.speaker.beep(100, 0.5)
    motor_front_right.run_for_degrees(-100, MAX_SPEED)
    hub.speaker.beep(102, 0.5)
    two_wheel_move(left_degrees=-300, right_degrees=-300, speed=20)
    hub.speaker.beep(105, 0.5)
    two_wheel_move(left_degrees=180, right_degrees=-36, speed=30)
    hub.speaker.beep(110, 0.5)
    #original on line below: left=625, right=642, speed=30
    two_wheel_move(left_degrees=625, right_degrees=642, speed=30)
    two_wheel_move(left_degrees=200, right_degrees=157, speed=30)
    grind(left_speed=20, right_speed=20, run_seconds=0.75)
    two_wheel_move(left_degrees=-241, right_degrees=-13, speed=30)
    grind(left_speed=20, right_speed=20, run_seconds=0.8)

def the_ending_trip():
    two_wheel_move(left_degrees=1044, right_degrees=1103, speed=30)
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

def check_battery():
    print(battery.info())
###############################################################
#Move Functions
def make_mark():
    motor_front_left.set_degrees_counted(0)
    motor_front_left.run_for_degrees(80, speed=80)
    motor_front_left.run_for_degrees(-80, speed=80)

def turn_until_line(left_or_right=TurnType.LEFT, speed=10):

    def stop_at_edge():
        if color.get_reflected_light() < BLACK_MIDDLE:
            current_color = color.get_reflected_light()
            return True
        else:
            return False
    if left_or_right == TurnType.LEFT:
        motor_pair.run_at_speed(0, -speed)
    else:
        motor_pair.run_at_speed(speed, 0)
    wait_until(stop_at_edge)
    wait_for_ms(10)
    wait_until(stop_at_edge)
    motor_pair.hold()
    hub.speaker.beep(90, 0.2)
    #print("Found line")

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
            print(hub.motion_sensor.get_yaw_angle())
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
    #print("Gyro Turn Complete", hub.motion_sensor.get_yaw_angle())

def gyro_turn_2(input_angle = 90, relative = False, timeout = 6, left_or_right = TurnType.BOTH, counter_or_clock = TurnDirection.CLOCKWISE):
    start_millis = time.ticks_ms()
    def limited_power(max_power,y):
        sy = sign(y)
        if max_power >= abs(y):
            return y
        else:
            return max_power*sy
    def is_timed_out():
        when_time_out = start_millis + timeout*1000
        if time.ticks_ms() > when_time_out:
            return True
        else:
            return False
    STOP_AT_TARGET_TOLERANCE = 1
    def map_gyro_angle(x):
        modulus_x = x % 360
        if modulus_x >= 0 and modulus_x <= 179:
            return modulus_x
        else:
            return modulus_x - 360
    if relative == True:
        desired_angle = rela_to_abs(input_angle)
    else:
        desired_angle = input_angle

    sanitized_target_angle = map_gyro_angle(desired_angle)
    MAX_POWER = 30
    error = 1000000
    power = 0
    gain = 0.8
    while abs(error) > STOP_AT_TARGET_TOLERANCE:
        error = hub.motion_sensor.get_yaw_angle() - sanitized_target_angle
        raw_power = abs(gain * error) + MIN_POWER_TO_MOVE
        power = limited_power(MAX_POWER, raw_power)
        if counter_or_clock == TurnDirection.CLOCKWISE:
            power = -power
        #print(error, raw_power,gain*error,power)
        if is_timed_out():
            print("TIMEOUT")
            break
        if left_or_right == TurnType.BOTH or left_or_right == TurnType.RIGHT:
            motor_right.start_at_power(int(power))
        if left_or_right == TurnType.BOTH or left_or_right == TurnType.LEFT:
            motor_left.start_at_power(int(power))
    motor_left.set_stop_action("brake")
    motor_right.set_stop_action("brake")
    motor_left.stop()
    motor_right.stop()
    wait_for_ms(POST_MOVE_WAIT_MS)
    #print("Gyro Turn Complete :: wanted =" ,input_angle,"relative =" ,relative,"calc =" ,desired_angle, "sanitized =" ,sanitized_target_angle, "ended_at =" ,hub.motion_sensor.get_yaw_angle())
    #if not work, try looking for 4 zeros in a row
    #print("Gyro Turn Complete", hub.motion_sensor.get_yaw_angle()

def grind(left_speed=40, right_speed=20, timeout=6, run_seconds=3):
    start_millis = time.ticks_ms()
    def is_timed_out():
        when_time_out = start_millis + timeout*1000
        if time.ticks_ms() > when_time_out:
            return True
        else:
            return False
    grind_timer = Timer()
    def done_grinding():
        if grind_timer.now() >= run_seconds:
            return True
        else:
            return False
    while done_grinding() == False:
        motor_right.start_at_power(right_speed)
        motor_left.start_at_power(-left_speed)
        if is_timed_out():
            print("TIMEOUT")
            break
    motor_right.stop()
    motor_left.stop()
    #print("Grind Complete")

def two_wheel_move(left_degrees=100, right_degrees=100, speed=30):
    MAX_POWER = 100
    ACCEL_MS_TO_FULL_SPEED = 600
    DECEL_MS_TO_FULL_SPEED = 1500
    motor_pair.preset(0,0)
    motor_pair.run_to_position(right_degrees, -left_degrees, speed, MAX_POWER, ACCEL_MS_TO_FULL_SPEED, DECEL_MS_TO_FULL_SPEED, STOP_HOLD)
    def is_done():
        if is_within_tolerance(left_degrees, get_left_motor_degrees(), 3) and is_within_tolerance(right_degrees, get_right_motor_degrees(), 3):
            return True
    while not is_done():
        pass
    print(get_left_motor_degrees(), get_right_motor_degrees())
    #print("Two Wheel Move Complete")

def straight(degrees_to_move=500, speed=35):
    two_wheel_move(left_degrees=degrees_to_move, right_degrees=degrees_to_move, speed=speed)

def rot_motion(print_seconds=3):
    motor_pair.float()
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
            print("two_wheel_move(left_degrees=" + str(-left_degrees_ran) + ", right_degrees=" + str(right_degrees_ran) + ", speed=30)")
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
    #print("Acquire line Complete")

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
    #print("Line follower Complete")

def delete_extra_presses():
    hub.left_button.was_pressed()
    hub.right_button.was_pressed()

def vrooom():

    delete_extra_presses()

    map_colors = {
        'blue': 1,
        'yellow': 2,
        'violet': 3,
        'cyan': 4,
        }

    map_trips = {
        1: the_trip_with_the_crates,
        2: the_trip_with_the_chest,
        3: the_trip_with_the_crane,
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

        if rawhub.button.right.presses() > 1:
            print("Aborting...")
            raise SystemExit("Button Pressed Twice")
        else:
            left_pressed = hub.left_button.was_pressed()
            right_pressed = hub.right_button.was_pressed()
        if left_pressed and right_pressed:
            test_trip()
        else:
            if right_pressed:
                increment_trip()

            if left_pressed:
                print("Starting Trip")
                run_selected_trip()
                print("Ending Trip")
        last_color = current_color
vrooom()
#test_gyro_turn()
raise SystemExit("END OF PROGRAM")
