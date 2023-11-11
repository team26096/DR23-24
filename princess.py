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
    #print("yaw={}", yaw)
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

def wait_for_right_color(clr=0):
    cs_right = color_sensor.reflection(port.F)
    print("wflc entry csl={}", cs_right)
    if clr == 0:
        while cs_right > 20:
            cs_right = color_sensor.reflection(port.F)
            runloop.sleep_ms(100)
    else:
        while cs_right < 90:
            cs_right = color_sensor.reflection(port.F)
            runloop.sleep_ms(100)
    print("wflc exit csl={}", cs_right)


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

def move_until_right_black(steer=100, speed=50, angle=90, stop=False):
    print("Inside move until black")
    motor_pair.move(motor_pair.PAIR_1, steer, velocity=speed, acceleration=100)
    wait_for_right_color(0)
    if stop: motor_pair.stop(motor_pair.PAIR_1)
    print("Done with move until black")

def move_until_right_white(steer=100, speed=50, angle=90, stop=False):
    print("Inside move until white")
    motor_pair.move(motor_pair.PAIR_1, steer, velocity=speed, acceleration=100)
    wait_for_right_color(1)
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

def follow_angle_for_distance(angle, distance_to_cover, speed=200, steering_proportion_factor=9):
    # get initial reading from left motor
    initial_position = abs(motor.relative_position(port.A))
    # print(initial_position)
    distance_covered = 0
    motor_pair.move(motor_pair.PAIR_1, angle)
    while (distance_covered < distance_to_cover):
        current_position = abs(motor.relative_position(port.A))
        distance_covered = current_position - initial_position
        if distance_covered < 0 : distance_covered = distance_covered * -1
        current_yaw = motion_sensor.tilt_angles()[0]
        if abs(current_yaw) != abs(angle):
            # The reason we are dividing by 18 is this:
            # The range of steering (Difference between power to be given to two wheels) is -100 to 100 (total range of <200>)
            # Yaw angle range is -180 to +179, that is 360, or <3600> in decidegrees
            # Thus 3600 / 200 = 18- So steering is adjusted by a factor of 18
            # So if yaw is off by 18 decidegrees (about 1.8 or 2 degress), a steering will be adjusted by a factor of 1 (18)
            motor_pair.move(motor_pair.PAIR_1, int ((current_yaw - angle) / steering_proportion_factor ), velocity=speed)

    motor_pair.stop(motor_pair.PAIR_1)
    print("Total distance travelled = ", distance_covered)
    print("Gyro follow angle for distance DONE")

def follow_angle_for_left_color_black(angle, speed=200, steering_proportion_factor=9):
    print("follow_angle_for_left_color_black start")

    # get initial reading from left color sensor
    cs_left = color_sensor.reflection(port.D)
    # print(cs_left)
    motor_pair.move(motor_pair.PAIR_1, angle)
    while (cs_left > 20):
        current_yaw = motion_sensor.tilt_angles()[0]
        # print(current_yaw)
        if abs(current_yaw) != abs(angle):
            motor_pair.move(motor_pair.PAIR_1, int ((current_yaw - angle) / steering_proportion_factor ), velocity=speed)
        cs_left = color_sensor.reflection(port.D)

    motor_pair.stop(motor_pair.PAIR_1)
    print("follow_angle_for_left_color_black DONE")

def follow_angle_for_left_color_white(angle, speed=200, steering_proportion_factor=9):
    print("follow_angle_for_left_color_white start")

    # get initial reading from left color sensor
    cs_left = color_sensor.reflection(port.D)
    # print(cs_left)
    motor_pair.move(motor_pair.PAIR_1, angle)
    while (cs_left < 90):
        current_yaw = motion_sensor.tilt_angles()[0]
        # print(current_yaw)
        if abs(current_yaw) != abs(angle):
            motor_pair.move(motor_pair.PAIR_1, int ((current_yaw - angle) / steering_proportion_factor ), velocity=speed)
        cs_left = color_sensor.reflection(port.D)

    motor_pair.stop(motor_pair.PAIR_1)
    print("follow_angle_for_left_color_white DONE")

def follow_angle_for_right_color_black(angle, speed=200, steering_proportion_factor=9):
    print("follow_angle_for_right_color_black start")

    # get initial reading from right color sensor
    cs_right = color_sensor.reflection(port.F)
    # print(cs_right)
    motor_pair.move(motor_pair.PAIR_1, angle)
    while (cs_right > 20):
        current_yaw = motion_sensor.tilt_angles()[0]
        # print(current_yaw)
        if abs(current_yaw) != abs(angle):
            motor_pair.move(motor_pair.PAIR_1, int ((current_yaw - angle) / steering_proportion_factor ), velocity=speed)
        cs_right = color_sensor.reflection(port.F)

    motor_pair.stop(motor_pair.PAIR_1)
    print("follow_angle_for_right_color_black DONE")

def follow_angle_for_right_color_white(angle, speed=200, steering_proportion_factor=9):
    print("follow_angle_for_right_color_white start")

    # get initial reading from left color sensor
    cs_right = color_sensor.reflection(port.F)
    # print(cs_right)
    motor_pair.move(motor_pair.PAIR_1, angle)
    while (cs_right < 90):
        current_yaw = motion_sensor.tilt_angles()[0]
        # print(current_yaw)
        if abs(current_yaw) != abs(angle):
            motor_pair.move(motor_pair.PAIR_1, int ((current_yaw - angle) / steering_proportion_factor ), velocity=speed)
        cs_right = color_sensor.reflection(port.F)

    motor_pair.stop(motor_pair.PAIR_1)
    print("follow_angle_for_right_color_white DONE")

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

async def runG():
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
    # move horizontal rack to right to avoid collision with camera
    motor.run_for_degrees(port.B, 600, 500) # move rack left
    motion_sensor.reset_yaw(0)
    runloop.sleep_ms(1000)
    move_until_right_black(0, -200, 0, True)
    await gyro_assisted_move(0, follow_angle_for_distance, distance_to_cover=degreesForDistance(12), speed=150)
    # move horizontal rack to left to engage with the camera
    await motor.run_for_degrees(port.B, 600, -500) # move rack left
    # pull camera and submarine so that submarine is out of its space
    await gyro_assisted_move(0, follow_angle_for_distance, distance_to_cover=degreesForDistance(15), speed=150)
    # move camera into the target area
    await motor.run_for_degrees(port.B, 660, 500) # move rack left
    # push rolling camera lever down
    await motor.run_for_degrees(port.C, 500, 500) # move rack down
    # disengage from the orange lever
    await motor.run_for_degrees(port.C, 500, -500) # move rack up
    # disengage from the camera
    motor.run_for_degrees(port.B, 100, -500) # move rack right
    move_until_right_black(0, -200, 0, True)




async def runD():
    print("Inside runD")
    # move horizontal rack to left to avoid collision with lights and sound
    motor.run_for_degrees(port.B, 500, -500) # move rack left

    # initialize motor pair
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
    # reset yaw to 0
    # motion_sensor.set_yaw_face(motion_sensor.FRONT)
    motion_sensor.reset_yaw(0)
    # sleep to ensure drop off is complete
    runloop.sleep_ms(1000)
    # move robot to approach audience drop off with checkpoints along the way
    await gyro_assisted_move(0, follow_angle_for_distance, distance_to_cover=degreesForDistance(34), speed=300)
    await gyro_assisted_move(0, follow_angle_for_right_color_white, speed=200)
    await gyro_assisted_move(0, follow_angle_for_right_color_black, speed=200)

    # move forward to complete audience drop off
    await gyro_assisted_move(0, follow_angle_for_distance, distance_to_cover=degreesForDistance(15), speed=100)

    # sleep to ensure drop off is complete
    runloop.sleep_ms(250)
    # move robot backward to align with hologram performer
    await gyro_assisted_move(0, follow_angle_for_distance, distance_to_cover=degreesForDistance(10), speed=-100)
    # align with hologram performer
    pivot_gyro_turn(0, -80, 42, True)
    # move horizontal rack to right to align with lights and sound
    motor.run_for_degrees(port.B, 500, 500)

    # move forwward to push hologram performer lever and complete the mission
    await gyro_assisted_move(-420, follow_angle_for_distance, distance_to_cover=degreesForDistance(15), speed=80)

    # Rotate left motor anti clockwise to flick the lever of speaker lights and sound
    motor.run_for_degrees(port.C, -200, 500)

    # move horizontal rack left to complete sound mixer and align with lights lever
    await motor.run_for_degrees(port.B, 600, -250)

    # move robot backward to pull light lever
    await gyro_assisted_move(-420, follow_angle_for_distance, distance_to_cover=degreesForDistance(15), speed=-80)

    # align with hologram performer
    pivot_gyro_turn(100, 0, 70, True)

    # go back to base
    await gyro_assisted_move(-700, follow_angle_for_distance, distance_to_cover=degreesForDistance(55), speed=100)

    print("Done with runD")

runloop.run(runG())
