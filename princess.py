# Imports --------------------
from hub import light_matrix, button, motion_sensor, light, sound, port
#from app import sound, music
import color, color_sensor, device, motor, motor_pair, orientation, runloop
import sys
from math import *

# Global contants --------------------
# Circumference of the drive wheel in cm
WHEEL_CIRCUMFERENCE = 27.004

# Common functions --------------------

# Helper function to return degrees based on distance in cm
def degreesForDistance(distance_cm):
    # Add multiplier for gear ratio if needed
    return int((distance_cm/WHEEL_CIRCUMFERENCE) * 360)

def get_yaw_value():
    return motion_sensor.tilt_angles()[0] * -0.1

def get_left_color_value():
    return color_sensor.reflection(port.D)

def get_right_color_value():
    return color_sensor.reflection(port.F)

def wait_for_yaw(angle=90):
    if angle > 0:
        while get_yaw_value() <= angle: runloop.sleep_ms(10)
    elif angle < 0:
        while get_yaw_value() >= angle: runloop.sleep_ms(10)

def wait_for_left_color(clr=0):
    print("wait_for_left_color START, entry csl={}", get_left_color_value())

    if clr == 0:
        while get_left_color_value() > 20:
            runloop.sleep_ms(10)
    else:
        while get_left_color_value() < 90:
            runloop.sleep_ms(10)

    print("wait_for_left_color END, exit csl={}", get_left_color_value())

def wait_for_right_color(clr=0):
    print("wait_for_right_color START, entry csl={}", get_right_color_value())
    if clr == 0:
        while get_right_color_value() > 20:
            runloop.sleep_ms(10)
    else:
        while get_right_color_value() < 90:
            runloop.sleep_ms(10)
    print("wait_for_right_color END, exit csl={}", get_right_color_value())

def spin_gyro_turn(steer=100, speed=50, angle=90, stop=False):
    motor_pair.move(motor_pair.PAIR_1, steer, velocity=speed, acceleration=100)
    wait_for_yaw(angle=angle)
    if stop: motor_pair.stop(motor_pair.PAIR_1)

async def pivot_gyro_turn(left_speed=0, right_speed=50, angle=90, stop=False):
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

def follow_for_forever():
    return True

def follow_for_distance(initial_position=0,
                        distance_to_cover=0):
    current_position = abs(motor.relative_position(port.A))
    distance_covered = current_position - initial_position
    if distance_covered < 0 : distance_covered = distance_covered * -1
    if (distance_covered >= distance_to_cover):
        return False
    else:
        return True

def follow_for_left_black():
    if (get_left_color_value() <= 20):
        return False
    else:
        return True

def follow_for_right_black():
    if (get_right_color_value() <= 20):
        return False
    else:
        return True

def follow_for_left_white():
    if (get_left_color_value() >= 90):
        return False
    else:
        return True

def follow_for_right_white():
    if (get_right_color_value() >= 90):
        return False
    else:
        return True

async def follow_gyro_angle(kp,
                            ki,
                            kd,
                            speed,
                            target_angle,
                            sleep_time,
                            follow_for, **kwargs):
    print("follow_gyro_angle START")
    # get initial reading from left motor

    integral = 0.0
    last_error = 0.0
    derivative = 0.0
    while (follow_for(**kwargs)):
        current_angle = get_yaw_value()
        error = current_angle - target_angle
        integral = integral + error
        derivative = error - last_error
        last_error = error

        steering_value = (error * kp) + (integral * ki) + (derivative * kd)
        # print("follow_gyro_angle steering={}, kp={}, ki={}, kd={}", steering_value, kp, ki, kd)

        if sleep_time:
            runloop.sleep_ms(sleep_time)
        # kp value should be +ve for forward movement (postive speed value), and -ve for backward movement (negative speed value)
        motor_pair.move(motor_pair.PAIR_1, int(steering_value), velocity=speed)

    motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)
    print("follow_gyro_angle END")

"""
This function turns the gyro in place to specified deci degrees (-1800 to 1800)
This is achieved by setting a steering value of -100 or + 100 based on which
side we want to turn.
"""
async def gyro_in_place_turn_for_decidegrees(decidegrees):
    initial_yaw = motion_sensor.tilt_angles()[0] * -0.1
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

def follow_angle_for_left_color_black(angle, speed=200, steering_proportion_factor=9):
    print("follow_angle_for_left_color_black START")
    # get initial reading from left color sensor
    cleft = color_sensor.reflection(port.D)
    # print(cs_left)
    motor_pair.move(motor_pair.PAIR_1, angle)
    while (cleft > 20):
        current_yaw = motion_sensor.tilt_angles()[0]
        # print(current_yaw)
        if abs(current_yaw) != abs(angle):
            motor_pair.move(motor_pair.PAIR_1, int ((current_yaw - angle) / steering_proportion_factor ), velocity=speed)
        cleft = color_sensor.reflection(port.D)

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
async def runSix():
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
    # reset yaw to 0
    motion_sensor.set_yaw_face(motion_sensor.TOP)
    motion_sensor.reset_yaw(0)

    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=400, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(25))

async def runTwo():
    # define motor pair
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)

    # reset yaw to 0
    motion_sensor.set_yaw_face(motion_sensor.TOP)
    motion_sensor.reset_yaw(0)
    runloop.sleep_ms(250)

    # intialize the motor relative position
    motor.reset_relative_position(port.A, 0)

    # go forward and position to turn 90 degrees
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=200, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(20))

    # turn 85 degrees and position to get out of base
    await pivot_gyro_turn(80, -80, 85, True)

    # go forward and position to turn 90 degrees
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=300, target_angle=90, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(15))

    # get out of base and look for left color sensor on black near the light show
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=300, target_angle=90, sleep_time=0, follow_for=follow_for_left_black)

    # move back little to align with camera
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-200, target_angle=90, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(4.5)))

    # push rolling camera lever down
    await motor.run_for_degrees(port.C, 800, 300) # move rack down

    # sleep to stabilize vertical rack
    runloop.sleep_ms(500)

    # move back to pull camera and submarine
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-200, target_angle=90, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(5.5)))

    # turn right to pull camera in the target area
    await pivot_gyro_turn(0, -100, 125, True)

    # turn left to the original position
    await pivot_gyro_turn(0, 84, 80, True)

    # disengage from the orange lever
    await motor.run_for_degrees(port.C, 800, -500) # move rack up

    # move forward to drop audience member and expert
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=200, target_angle=84, sleep_time=0, follow_for=follow_for_left_black)

    # Drop audience members and expert to target areas
    await motor.run_for_degrees(port.B, 200, -400)
    await motor.run_for_degrees(port.B, 400, 400)
    await motor.run_for_degrees(port.B, 400, -400)

    # go forward to align with rolling camera
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=200, target_angle=90, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(53)))

    # sleep to stabilize the rack
    runloop.sleep_ms(500)

    # bring rack down to engage with rolling camera
    await motor.run_for_degrees(port.C, 720, 300) # move rack down

    # go back to push rolling camera
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-200, target_angle=100, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(23)))

    # bring rack up to disengage with rolling camera
    await motor.run_for_degrees(port.C, 500, -400) # move rack down

    # go forward to right base
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=300, target_angle=125, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(70)))

async def runFour():
    print("Inside runD")
    # move horizontal rack to left to avoid collision with lights and sound
    motor.run_for_degrees(port.B, 500, -500) # move rack left

    # initialize motor pair
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
    # reset yaw to 0
    motion_sensor.set_yaw_face(motion_sensor.TOP)
    motion_sensor.reset_yaw(0)
    # sleep to ensure drop off is complete
    runloop.sleep_ms(1000)
    # move robot to approach audience drop off with checkpoints along the way
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=400, target_angle=-3, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(34)))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=400, target_angle=-3, sleep_time=0, follow_for=follow_for_right_white)
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=300, target_angle=-3, sleep_time=0, follow_for=follow_for_right_black)

    # move forward to complete audience drop off
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=200, target_angle=-3, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(17)))

    # sleep to ensure drop off is complete
    runloop.sleep_ms(250)
    # move robot backward to align with hologram performer
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-100, target_angle=-3, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(12)))

    # align with hologram performer
    await pivot_gyro_turn(0, -80, 42, True)

    # move forwward to push hologram performer lever and complete the mission
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=80, target_angle=42, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(15)))

    # Rotate left motor anti clockwise to flick the lever of sound speaker
    motor.run_for_degrees(port.C, -100, 200)

    # move robot backward to align with lights
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-150, target_angle=42, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(2))

    # turn left to complete alignment with lights lever
    await pivot_gyro_turn(-150, 0, 30, True)

    # move robot backward to pull light lever
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-150, target_angle=30, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(10))

    # align with hologram performer
    await pivot_gyro_turn(-300, 0, -20, True)

    # Rotate left motor anti clockwise to flick the lever of sound speaker
    motor.run_for_degrees(port.C, 100, 200)

    # go back to base
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-500, target_angle=-20, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(58)))

    print("Done with runD")

async def testGyro():
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
    motion_sensor.set_yaw_face(motion_sensor.TOP)
    motion_sensor.reset_yaw(0)
    motor.reset_relative_position(port.A, 0)
    motor.reset_relative_position(port.E, 0)
    runloop.sleep_ms(500)
    # follow_angle_for_distance(0, 30, 100, 1.2)
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-500, target_angle=0, sleep_time=0, follow_for=follow_for_right_black)
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-500, target_angle=0, sleep_time=0, follow_for=follow_for_right_white)
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-500, target_angle=0, sleep_time=0, follow_for=follow_for_right_black)

    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=500, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(120)))

async def runFive():
    print("Inside runC")
    # move horizontal rack to left to avoid hitting holagram performer
    motor.run_for_degrees(port.C, 700, -500) # move rack left

    # initialize motor pair
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
    # reset yaw to 0
    motion_sensor.set_yaw_face(motion_sensor.TOP)
    motion_sensor.reset_yaw(0)
    # sleep to ensure drop off is complete
    runloop.sleep_ms(1000)
    # move robot forward to start going to augmented reality
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=400, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(34)))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=300, target_angle=0, sleep_time=0, follow_for=follow_for_right_white)
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=300, target_angle=0, sleep_time=0, follow_for=follow_for_right_black)

    # turning to go towards augmented reality
    await pivot_gyro_turn(0, 100, -90, True)

    # move backward to hook on to augmented reality
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=300, target_angle=-90, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(20)))

    # go forward till right sensor see's white checkpoint close to augmented reality
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=300, target_angle=-90, sleep_time=0, follow_for=follow_for_right_white)

    # move horizontal rack to right to align with augmented reality hook
    await motor.run_for_degrees(port.C, 700, 500) # move rack right

    # move backward to hook on to augmented reality
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-200, target_angle=-90, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(15)))

    # move horizontal rack to left to pull augmented reality lever
    await motor.run_for_degrees(port.C, 700, -500) # move rack left
    
    # move backward to complete augmented reality
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-200, target_angle=-90, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(14)))

    # move forward to complete augmented reality
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=300, target_angle=-90, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(29)))

    # turning right to align for anna and audience member drop of
    await pivot_gyro_turn(100, 0, -30, True)

    # move forward to drop anna and audience member
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=200, target_angle=-30, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(25)))

    # move backward to leave anna and audience member
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-200, target_angle=-30, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(15)))

    # turning back so robot can go back to base
    await pivot_gyro_turn(0, 100, -90, True)

    # move horizontal rack to right to avoid hitting light show
    await motor.run_for_degrees(port.C, 250, 500) # move rack right

    # move forward to drop anna and audience member
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=400, target_angle=-90, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(63)))

    # turning back so robot can go back to base
    await pivot_gyro_turn(0, 100, -135, True)

    # move horizontal rack to left to avoid hitting 3d cinema
    motor.run_for_degrees(port.C, 350, -500) # move rack left

    # move forward to drop anna and audience member
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=500, target_angle=-135, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(60)))

async def runThree():
    print("Inside runE")

    # initialize motor pair
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
    # reset yaw to 0
    motion_sensor.set_yaw_face(motion_sensor.TOP)
    motion_sensor.reset_yaw(0)
    # sleep to ensure drop off is complete
    runloop.sleep_ms(1000)
    # move robot forward to start going to craft creator
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=200, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(21)))

    # Turn left to align with craft creator
    await pivot_gyro_turn(-100, 100, -42, True)

    # move robot forward to start going to craft creator
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=200, target_angle=-45, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(6)))

    # lift Izzy
    await motor.run_for_degrees(port.C, 4000, 1000, stop=motor.HOLD)
    
    # move robot forward to latch onto craft creator lid
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=200, target_angle=-45, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(18)))

    
    # move robot backward to lift craft creator lid
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-200, target_angle=-45, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(35)))
    await pivot_gyro_turn(150, -150, 60, True)

async def runOne():
    print("Inside runF")

    # initialize motor pair
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
    # reset yaw to 0
    motion_sensor.set_yaw_face(motion_sensor.TOP)
    motion_sensor.reset_yaw(0)
    # sleep for 1 second
    runloop.sleep_ms(1000)
    # move robot forward to start going to sound mixer
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=200, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(14.5)))

    # Turn right to align with sound mixer
    await pivot_gyro_turn(100, -100, 40, True)

    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=125, target_angle=45, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(30)))

    # Turn right to align with sound mixer
    await pivot_gyro_turn(40, -40, 47, True)

    # sleep for 1 second
    runloop.sleep_ms(1000)

    # come back to base
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-400, target_angle=50, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(43)))

def doPortCheck():
    # check port status
    if device.ready(port.A):
        print("Port A is ready")
    else:
        print("Port A is NOT ready")
        return False

    if device.ready(port.B):
        print("Port B is ready")
    else:
        print("Port B is NOT ready")
        return False

    if device.ready(port.C):
        print("Port C is ready")
    else:
        print("Port C is NOT ready")
        return False

    if device.ready(port.D):
        print("Port D is ready")
    else:
        print("Port D is NOT ready")
        return False

    if device.ready(port.E):
        print("Port E is ready")
    else:
        print("Port E is NOT ready")
        return False

    if device.ready(port.F):
        print("Port F is ready")
    else:
        print("Port F is NOT ready")
        return False

    return True

light.color(light.POWER, color.MAGENTA)
light_matrix.write("3")
runloop.run(runThree())
