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
def get_all_values():
    global cs_left, cs_right, yaw
    cs_left = color_sensor.reflection(port.D)
    cs_right = color_sensor.reflection(port.F)
    yaw = motion_sensor.tilt_angles()[0] * -1
    print("yaw={}", yaw)

def wait_for_yaw(angle=90):
    if angle > 0:
        while yaw <= angle: get_all_values()
    elif angle < 0:
        while yaw >= angle: get_all_values()

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

# Load program wait --------------------
# print('waiting to start')
# light.color(light.POWER, color.BLUE) #blue indicates program loaded and ready
# runloop.until(button.pressed(button.LEFT)) #press left button to start program
# runloop.sleep_ms(500)
# light_matrix.show_image(light_matrix.IMAGE_DIAMOND) #program starting to run
# light.color(light.POWER, color.GREEN)

# Main programs --------------------
async def runA():
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degreesForDistance(20), 0)
    #spin_gyro_turn(100, 50, 90, True)
    pivot_gyro_turn(80, 0, 900, True)
    #sys.exit("Finished")

runloop.run(runA())
