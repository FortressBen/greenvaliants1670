#standard objects
#owner: architect coder
leftMotor = Motor('A')
rightMotor = Motor('B')
colorSensor = ColorSensor('C')
frontPTO = Motor('D')
rearPTO = Motor('E')
leftButton = whatever
hub = PrimeHub()

#owner: architect coder
def main_loop():
    #thou shall not block
    #thou shall not implement logic here.
    while True:
        run_trip_requested_by_button_press_and_color_sensor()  

#owners: architect 
def run_trip_requested_by_button_press_and_color_sensor():

    def select_trip_from_color(indicated_color):
        if indicated_color == 'Red':
            return treadmill_and_stuff_trip
        elif indicated_color == 'Green':
            return stuff_near_base_trip
        else:
            return None

    if hub.left_button.was_pressed():
        indicated_color = color_sensor.get_color()
        trip_to_run = select_trip_from_color(indicated_color)
        trip_to_run()
    elif hub.right_button.was_pressed():
        robot_ready_pose()
    else:
        pass

#owner: TFB coder
def robot_ready_pose():
    def stop_with_brake_mode(motor,brake_mode='coast'):
        motor.set_stop_action(brake_mode)
        motor.stop()

    stop_with_brake_mode(leftMotor)
    stop_with_brake_mode(rightMotor)
    stop_with_brake_mode(rearPTO)
    stop_with_brake_mode(frontPTO)
    
#trips. good names are VERY VERY hard. short, easy to remember, descriptive
#owner: TFB coder
def treadmill_and_stuff_trip():

    # some functions are mission specific.
    # if its a simple motor command, that's just fine
    # inner functions are only used here.
    def get_to_line():
        tank_drive(200,200,power=30)
        gyro_turn(30,timeout_secs=2,relative=False)

    def follow_long_line_to_wall():
        follow_line(left_wheel_end_degrees=500,base_power=35)

    def align_to_wall():
        gyro_turn(180,timeout+secs=2,relative=False)
        grind()

    def dump_bricks():
        two_wheel_arc(300,200,power=40)
        frontPTO.run_to_degrees(20)

    def back_to_base():
        tank_drive(-100,-100,power=40)
        gyro_turn(80,timeout_secs=2,relative=False
        tank_drive(-100,-100,power=40)

    #goal: the steps are pretty much exactly how you'd describe
    #them while pointing at the table, talking to a judge
    #note that all of the steps are at the same level of detail
    get_to_line()
    follow_long_line_to_wall()
    align_to_wall()
    dump_bricks()
    back_to_base()

#owner: TFB coder
def stuff_near_base_trip():
    ..

#common functions
#owner:architect coder
def gyro_turn(heading=0, power=20,timeout_secs=3):
    ...
def grind( power=-20, timeout_secs=3):
    ...
def follow_line( limit_degrees=1000, base_power=20, ):
    ...
def tank_drive(left_wheel_end_degrees=0,right_wheel_degrees=0):
    ...
 
#owner:architect coder
#allows defining methods out of dependency order
main_loop()
raise(SystemError('done'))