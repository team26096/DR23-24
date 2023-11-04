# Imports --------------------
from hub import light_matrix, button, motion_sensor, light, sound, port
#from app import sound, music
import color, color_sensor, device, motor, motor_pair, orientation, runloop
import sys
from math import *

# Global contants --------------------
# Circumference of the drive wheel in cm
WHEEL_CIRCUMFERENCE = 26.69

# Common functions --------------------

# Helper function to return degrees based on distance in cm
def degreesForDistance(distance_cm):
    # Add multiplier for gear ratio if needed
    return int((distance_cm/WHEEL_CIRCUMFERENCE) * 360)

yaw = 0
cs_left = 0
cs_right = 0

def get_all_values():
    global cs_left, cs_right, yaw
    cs_left = color_sensor.reflection(port.D)
    cs_right = color_sensor.reflection(port.F)
    yaw = motion_sensor.tilt_angles()[0] * -0.1
    print("yaw={}", yaw)
    #print("csl={}", cs_left)
    #print("csr={}", cs_right)


def wait_for_yaw(angle=90):
    if angle > 0:
        while yaw <= angle: get_all_values()
    elif angle < 0:
        while yaw >= angle: get_all_values()

def wait_for_left_color(clr=0):
    cs_left = color_sensor.reflection(port.D)
    print("wflc entry csl={}", cs_left)
    if clr == 0:
        while cs_left > 20: 
            cs_left = color_sensor.reflection(port.D)
            runloop.sleep_ms(100)
    else:
        while cs_left < 90:
            cs_left = color_sensor.reflection(port.D)
            runloop.sleep_ms(100)
    print("wflc exit csl={}", cs_left)

def spin_gyro_turn(steer=100, speed=50, angle=90, stop=False):
    motion_sensor.reset_yaw(0)
    motor_pair.move(motor_pair.PAIR_1, steer, velocity=speed, acceleration=100)
    wait_for_yaw(angle=angle)
    if stop: motor_pair.stop(motor_pair.PAIR_1)

def pivot_gyro_turn(left_speed=0, right_speed=50, angle=90, stop=False):
    motion_sensor.reset_yaw(0)
    motor_pair.move_tank(motor_pair.PAIR_1, left_speed, right_speed)
    wait_for_yaw(angle=angle)
    if stop: motor_pair.stop(motor_pair.PAIR_1)

def move_until_left_black(steer=100, speed=50, angle=90, stop=False):
    print("Inside move until black")
    motor_pair.move(motor_pair.PAIR_1, steer, velocity=speed, acceleration=100)
    wait_for_left_color(0)
    if stop: motor_pair.stop(motor_pair.PAIR_1)
    print("Done with move until black")

def move_until_left_white(steer=100, speed=50, angle=90, stop=False):
    print("Inside move until white")
    motor_pair.move(motor_pair.PAIR_1, steer, velocity=speed, acceleration=100)
    wait_for_left_color(1)
    if stop: motor_pair.stop(motor_pair.PAIR_1)
    print("Done with move until white")

# Main programs --------------------
async def runA():
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degreesForDistance(20), 0)
    #spin_gyro_turn(100, 50, 90, True)
    pivot_gyro_turn(80, 0, 900, True)
    #sys.exit("Finished")

async def runD():
    print("Inside runD")
    motor.run_for_degrees(port.C, 600, -500) # move rack left
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
    move_until_left_black(0, 200, 0, True)
    move_until_left_white(0, 200, 0, True)
    move_until_left_black(0, 200, 0, True)
    move_until_left_white(0, 200, 0, True)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degreesForDistance(12), 0)
    move_until_left_black(0, -80, 0, True)
    # move_until_left_white(0, -80, 0, True)
    # move_until_left_black(0, -80, 0, True)
    pivot_gyro_turn(0, -80, 45, True)
    motor.run_for_degrees(port.C, 600, 500) # move rack right
    print("Done with runD")

runloop.run(runD())
