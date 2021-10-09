#
# This will run the robot back and forth, measuring the average speed (in deg/sec) vs nominal speed
# NOTE: this doesnt consider the accleration and deceleration phases, so this is not really exactly
# right, but it gives a rough idea. 
# to make it better, the default moves use 100ms accel time and 150ms decel time, so factoring those in
# would make the measurements more reliable
#

from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
from runtime import MultiMotor, VirtualMachine
import time
hub = PrimeHub() 

motors = MotorPair('B' , 'A')

data = []
t=0
DISTANCE=600
for s in range(10,90,5):

    def log_point(t,speed, degrees):
        data.append((t,speed,degrees/t*1000))
    t = time.ticks_ms()
    motors.move_tank(DISTANCE,unit="degrees",left_speed=s,right_speed=s)  
    log_point(time.ticks_ms()-t,s,DISTANCE)  
    t = time.ticks_ms()
    motors.move_tank(-DISTANCE,unit="degrees",left_speed=s,right_speed=s)
    log_point(time.ticks_ms()-t,-s,DISTANCE)

s = "speed\tdeg/sec,"
for d in data:
    s += (str(d[1]) + "\t"+ str(d[2]) + ",")

print(s)