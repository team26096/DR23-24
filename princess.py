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

"""
The function moves the robot for distance requested, and uses the gyro sensor to keep a
straight line. It uses the yaw value to adjust steering (if needed) to correct direction.
Usually a low steering correction is sufficient.
#arg1: distance_to_cover (in degrees - 360 degrees is one full rotation of robot wheel)
#arg2: how much steering correction to do at a time to adjust direction. Note this has to
    be a low value, otherwise the movement could be erratic. recommended values are 1
    to 10.
"""
async def gyro_assisted_move(angle, stop_function, **kwargs):
    stop_function(angle, **kwargs)

"""
This function turns the gyro in place to specified deci degrees (-1800 to 1800)
This is achieved by setting a steering value of -100 or + 100 based on which
side we want to turn.
"""
async def gyro_in_place_turn_for_decidegrees(decidegrees):
    initial_yaw = motion_sensor.tilt_angles()[0]
    target_yaw = initial_yaw + decidegrees
    if decidegrees > 0:
        steering = 100
        # since steering is 100, the yaw value will increase.
        # so 'move' until the current value is less than target_yaw
        while motion_sensor.tilt_angles()[0] < target_yaw:
            motor_pair.move(motor_pair.PAIR_1,steering)
    elif decidegrees < 0:
        steering = -100
        # since steering is -100, the yaw value will decrease.
        # so 'move' until the current value is greater than target_yaw
        while motion_sensor.tilt_angles()[0] > target_yaw:
            motor_pair.move(motor_pair.PAIR_1,steering)
    else:
        # if decidegrees is 0, there is nothing to do!
        return
    motor_pair.stop(motor_pair.PAIR_1)

def follow_forever():
    return True

def follow_angle_for_distance(angle, distance_to_cover, steering_proportion_fatcor=9):
    # get initial reading from left motor
    initial_position = abs(motor.relative_position(port.A))
    # print(initial_position)
    distance_covered = 0
    motor_pair.move(motor_pair.PAIR_1, angle)
    while (distance_covered < distance_to_cover):
        current_position = abs(motor.relative_position(port.A))
        distance_covered = current_position - initial_position
        current_yaw = motion_sensor.tilt_angles()[0]
        # print("current_yaw = {}".format(current_yaw))
        # if current_yaw < angle:
        #     # steer slightly to the right
        #     motor_pair.move(motor_pair.PAIR_1, int(-1000/(angle - current_yaw)))
        # elif current_yaw > angle:
        #     motor_pair.move(motor_pair.PAIR_1, int(1000/(angle - current_yaw)))
        # else:
        #     motor_pair.move(motor_pair.PAIR_1, angle)
        print(current_yaw)
        if abs(current_yaw) != abs(angle):
            # The reason we are dividing by 18 is this:
            # The range of steering (Difference between power to be given to two wheels) is -100 to 100 (total range of <200>)
            # Yaw angle range is -180 to +179, that is 360, or <3600> in decidegrees
            # Thus 3600 / 200 = 18  - So steering is adjusted by a factor of 18
            # So if yaw is off by 18 decidegrees (about 1.8 or 2 degress), a steering will be adjusted by a factor of 1 (18)
            motor_pair.move(motor_pair.PAIR_1, int ((current_yaw - angle) / steering_proportion_fatcor ), velocity=500)

    motor_pair.stop(motor_pair.PAIR_1)
    print("Total distance travelled = ", distance_covered)
    print("Gyro follow angle for distance DONE")


# Main programs --------------------

async def sample_gyro_assisted_move():
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
    motion_sensor.set_yaw_face(motion_sensor.FRONT)
    motion_sensor.reset_yaw(0)
    # get current yaw_value
    yaw_value = motion_sensor.tilt_angles()[0]
    # Turn the robot 45 degrees (450 decidegrees) in place
    #await gyro_in_place_turn_for_decidegrees(450)
    # New yaw value should be
    # yaw_value += 450
    await gyro_assisted_move(450, follow_angle_for_distance, distance_to_cover=3000)

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
