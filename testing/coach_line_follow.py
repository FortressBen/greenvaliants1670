from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
import time
import os
hub = PrimeHub()

class LoopTimer:
    def __init__(self,report_interval):
        self.start_time = 0
        self.loops = 0
        self.total_loops = 0
        self.report_interval=report_interval
        self.reset()

    def reset(self):
        self.start_time = time.ticks_ms()
        self.loops = 0

    def start(self):
        self.start_time=time.ticks_ms()

    def loop(self):
        self.total_loops += 1
        self.loops += 1
        if self.loops % self.report_interval == 0:
            print("Loop Avg: {t} ms".format(t=self.avg_time()))

    def avg_time(self):
        current_time = time.ticks_ms()
        r= (current_time - self.start_time)/ self.loops
        self.reset()
        return r

class SpeedMap:
    def __init__(self,default_speed):
        self.regions = []
        self.default_speed = default_speed

    def add_region(self,speed,start_degrees,end_degrees):
        self.regions.append((speed,start_degrees,end_degrees))

    def get_speed(self,degrees):
        for r in self.regions:
            if degrees >= r[1] and degrees <= r[2]:
                return r[0]
        return self.default_speed

class ErrorTracker:
    def __init__(self,threshold,speed_up_count):
        self.number_ok = 0
        self.is_sped_up = False
        self.threshold = threshold
        self.speed_up_count = speed_up_count
    
    def can_speed_up(self,error):
        if error < self.threshold:
            self.number_ok += 1
        else:
            self.number_ok = 0
        should_speed_up = self.number_ok > self.speed_up_count
        if (not self.is_sped_up) and should_speed_up:
            print("Speeding UP")
        if self.is_sped_up and (not should_speed_up):
            print("Slowing Down")
        self.is_sped_up = should_speed_up
        return should_speed_up

        return is_sped_up
class DataLogger:
    PROJECT_FOLDER="/projects"
    def __init__(self,header):
        self.data=[]
        self.header=["time_ms"] + header 
        self.sample_count =0
    def add_data(self,tp):
        self.sample_count += 1
        if self.sample_count % 100:
            self.sample_count = 0
            self.data.append([time.ticks_ms()] + tp)

    def save(self,filename):
        f = open(DataLogger.PROJECT_FOLDER +'/' + filename,'w')
        f.write(",".join([ str(x) for x in self.header ] ) + "\n")
        for p in self.data:
            msg = ",".join([ str(x) for x in p ] )
            f.write(msg + "\n")
        f.close()

left = Motor('B')
right = Motor('A')
color = ColorSensor('C')
data = DataLogger(["cv","err","total_err","padj","iadj","leftpwr","rightpwr"])
line_1_map = SpeedMap(30)
line_1_map.add_region(45,0,700)
line_1_map.add_region(45,1500,2000)

loop_timer = LoopTimer(300)

def power_adjust_deadband(motor_input):
    if motor_input < 0 and motor_input > -13:
        return -15
    elif motor_input > 0 and motor_input < 13:
        return 15
    else:
        return motor_input

def limited_value(val, abs_bound):
    if val > abs_bound:
        return abs_bound
    elif val < -abs_bound:
        return -abs_bound
    else:
        return val

def gryo_guided_straight(speed, yaw_angle, degrees):
    
def follow_line(logger,left_motor,right_motor,color_sensor,max_loops=2000):

    SLOW_SPEED=20
    left_motor.set_degrees_counted(0)
    right_motor.set_degrees_counted(0)

    kp = 0.08
    ki = 0.000000001
    kd= 0.0
    SENSOR_TARGET = 250
    MAX_ADJ_SPEED= 50
    MAX_TOTAL_ERROR = int(MAX_ADJ_SPEED/ki)   #cant do more than reverse the wheels!
    #SLOW_DOWN_ERROR_THRESHOLD = SENSOR_TARGET *0.4
    #SLOW_SPEED = 15

    total_error = 0
    prev_error = 0
    while loop_timer.total_loops < max_loops:
        currentSpeed = line_1_map.get_speed(right_motor.get_degrees_counted())
        cv = color_sensor.get_red()
        error = cv - SENSOR_TARGET

        if loop_timer.total_loops % 50 == 0:
            print ("gyro yaw: {y}".format(y=hub.motion_sensor.get_yaw_angle()))
        #if error > 150:
        #    print ( "High Error!: Left={l}, Righ={r}".format(l=left_motor.get_degrees_counted(), r=right_motor.get_degrees_counted()))

        total_error += error
        total_error = limited_value(total_error,MAX_TOTAL_ERROR)

        p_adj = (kp * error)
        i_adj = (ki * total_error)
        d_adj = (kd * (error - prev_error))

        adj = p_adj + i_adj + d_adj

        leftPower = currentSpeed - adj
        rightPower = currentSpeed + adj

        leftPower = limited_value(leftPower,MAX_ADJ_SPEED)
        rightPower = limited_value(rightPower,MAX_ADJ_SPEED)        
        #leftPower = power_adjust_deadband(leftPower)
        #rightPower = power_adjust_deadband(rightPower)
        leftPower = int(leftPower)
        rightPower = int(rightPower)
        right_motor.start_at_power(rightPower)
        left_motor.start_at_power(-1*(leftPower))
        prev_error = error
        #logger.add_data([cv,error,total_error,p_adj,i_adj,leftPower,rightPower])
        #print("error={e},te={te},p_adj={pa},i_adj={ia},left={lpwr},right={rpwr}".format(te=total_error,e=error,pa=p_adj,ia=i_adj,lpwr=leftPower,rpwr=rightPower))
        loop_timer.loop()

    left_motor.stop()
    right_motor.stop()
print("*"*80)
start_time = time.ticks_ms()
follow_line(data, left,right,color,max_loops=1400)
end_time = time.ticks_ms()
print("****Done, in {s} ms".format(s=str((end_time - start_time))))
#logger.save("line-log.csv")
raise SystemExit("done")