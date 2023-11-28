# Imports --------------------
from hub import light_matrix, button, motion_sensor, light, sound, port
#from app import sound, music
import color, color_sensor, device, motor, motor_pair, orientation, runloop
import sys
from math import *
from time import time

# Global contants --------------------
# Circumference of the drive wheel in cm
WHEEL_CIRCUMFERENCE = 27.004

# Utility functions --------------------

# Helper function to return degrees based on distance in cm
def degreesForDistance(distance_cm):
    # Add multiplier for gear ratio if needed
    return int((distance_cm/WHEEL_CIRCUMFERENCE) * 360)

# Helper function to return yaw in degrees
def get_yaw_value():
    return motion_sensor.tilt_angles()[0] * -0.1

# Helper function to return reflect light intensity on port D - left light sensor
def get_left_color_value():
    return color_sensor.reflection(port.D)

# Helper function to return reflected light intensity on port F - right light sensor
def get_right_color_value():
    return color_sensor.reflection(port.F)

##### Common Functions with callback routines #####

# Turn functions
def wait_for_yaw(angle=90):
    if angle > 0:
        while get_yaw_value() <= angle: runloop.sleep_ms(10)
    elif angle < 0:
        while get_yaw_value() >= angle: runloop.sleep_ms(10)

def spin_gyro_turn(steer=100, speed=50, angle=90, stop=False):
    motor_pair.move(motor_pair.PAIR_1, steer, velocity=speed, acceleration=100)
    wait_for_yaw(angle=angle)
    if stop: motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)

async def pivot_gyro_turn(left_speed=0, right_speed=50, angle=90, stop=False):
    motor_pair.move_tank(motor_pair.PAIR_1, left_speed, right_speed)
    wait_for_yaw(angle=angle)
    if stop: motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)

# follow gyro angle functions
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
        # compute steering correction
        steering_value = (error * kp) + (integral * ki) + (derivative * kd)

        if sleep_time:
            runloop.sleep_ms(sleep_time)
        # kp value should be +ve for forward movement (postive speed value), and -ve for backward movement (negative speed value)
        motor_pair.move(motor_pair.PAIR_1, int(steering_value), velocity=speed)

    # stop when follow_for condition is met
    motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)

# Main programs --------------------

# initialize motor and reset yaw
def doInit():
    # initialize motor pair
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
    print("runOnePt1: reset gyro to 0.. waiting to stabilize")
    # reset yaw to 0
    motion_sensor.set_yaw_face(motion_sensor.TOP)
    motion_sensor.reset_yaw(0)
    while motion_sensor.stable() == False:
        runloop.sleep_ms(100)
        print("Gyro NOT stable")
    #print("Gyro stable")

# run 1 part 1 code
async def runOnePt1():
    print("runOnePt1 -- START")
    # # initialize motor pair
    # motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
    # print("runOnePt1: reset gyro to 0.. waiting to stabilize")
    # # reset yaw to 0
    # motion_sensor.set_yaw_face(motion_sensor.TOP)
    # motion_sensor.reset_yaw(0)
    # while motion_sensor.stable() == False:
    #     await runloop.sleep_ms(100)
    #     print("Gyro NOT stable")
    doInit()

    #print("Gyro stable")
    # sleep for 1 second to allow gyro to stabilize
    # runloop.sleep_ms(1000)

    # move robot forward to start going to sound mixer
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=200, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(14.5)))
    # print("runOnePt1: motor A position before approaching sound mixer: " + str(motor.relative_position(port.A)))

    # Turn right to align with sound mixer
    await pivot_gyro_turn(100, -100, 43, True)

    # move forward and approach sound mixer
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=125, target_angle=45, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(27)))
    # print("runOnePt1: motor A position after approaching sound mixer: " + str(motor.relative_position(port.A)))
    # print("runOnePt1: angle at approaching sound mixer: " + str(get_yaw_value()))

    # Turn right to align with sound mixer
    await pivot_gyro_turn(50, -50, 47, True)
    # print("runOnePt1: angle at commpleting sound mixer: " + str(get_yaw_value()))

    # sleep for 0.5 second
    await runloop.sleep_ms(500)

    # Turn right to get into Noah's hoop
    await pivot_gyro_turn(100, -100, 55, True)
    # print("runOnePt1: angle at picking Noah: " + str(get_yaw_value()))

    # come back to base
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-400, target_angle=50, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(43)))
    # print("runOnePt1: motor A position after coming to base: " + str(motor.relative_position(port.A)))

    # move motor B to get it in position for runonept2
    await motor.run_for_degrees(port.B, 3000, 1100)
    print("runOnePt1 -- END")

# run 1 part 2 code
async def runOnePt2():
    print("runOnePt2 -- START")
    # # initialize motor pair
    # motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
    # # print("runOnePt2: reset gyro to 0.. waiting to stabilize")
    # # reset yaw to 0
    # motion_sensor.set_yaw_face(motion_sensor.TOP)
    # motion_sensor.reset_yaw(0)
    # while motion_sensor.stable() == False:
    #     await runloop.sleep_ms(100)
    #     print("Gyro NOT stable")
    #print("Gyro stable")
    # sleep for 1 second to allow gyro to stabilize
    # runloop.sleep_ms(1000)
    doInit()

    # go straight to get out of base
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=400, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(60)))

    # go forward even more to get a checkpoint with the left color sensor on white and then black to know our position
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=300, target_angle=4, sleep_time=0, follow_for=follow_for_left_white)
    #await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=300, target_angle=4, sleep_time=0, follow_for=follow_for_left_black)

    # spin turn to right at 45 degrees
    await pivot_gyro_turn(100, -100, 45, True)

    # go forward for 4 cm
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=400, target_angle=45, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(14)))

    # go backward to drop audience memebrs
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-400, target_angle=45, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(2)))

    # move motor B to get it in position to pick up Sam later in the run
    await motor.run_for_degrees(port.B, 725, -1100)

    # spin turn to position with theatre scene change
    await pivot_gyro_turn(-100, 0, -40, True)

    # push forward once to complete theatre scene change
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=150, target_angle=-45, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(10)))

    # move motor B to get it in position to pick up Sam
    await motor.run_for_degrees(port.B, 2500, -1100)

    # reset motor for the relative position
    motor.reset_relative_position(port.A, 0)
    # come back 5 degrees
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-400, target_angle=-45, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(3)))

    # pivot turn
    await pivot_gyro_turn(0,-200, 25, True)

    # go back to base
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-400, target_angle=25, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(65)))

    print("runOnePt2 -- END")

# run 2 code
async def runTwo():
    print("runTwo -- START")
    # # initialize motor pair
    # motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
    # # print("runOnePt2: reset gyro to 0.. waiting to stabilize")
    # # reset yaw to 0
    # motion_sensor.set_yaw_face(motion_sensor.TOP)
    # motion_sensor.reset_yaw(0)
    # while motion_sensor.stable() == False:
    #     await runloop.sleep_ms(100)
    #     print("Gyro NOT stable")
    # #print("Gyro stable")
    doInit()

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

    print("run2 -- END")

# run 3 code
async def runThree():
    print("runThree -- START")
    # initialize motor pair
    # motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
    # # reset yaw to 0
    # motion_sensor.set_yaw_face(motion_sensor.TOP)
    # motion_sensor.reset_yaw(0)
    # while motion_sensor.stable() == False:
    #     await runloop.sleep_ms(100)
    #     print("Gyro NOT stable")
    #print("Gyro stable")
    # sleep to ensure drop off is complete
    #runloop.sleep_ms(1000)
    doInit()

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
    print("runThree -- END")

async def runFour():
    print("runFour -- START")
    # initialize motor pair
    # motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
    # # reset yaw to 0
    # motion_sensor.set_yaw_face(motion_sensor.TOP)
    # motion_sensor.reset_yaw(0)
    # while motion_sensor.stable() == False:
    #     await runloop.sleep_ms(100)
    #     print("Gyro NOT stable")
    #print("Gyro stable")
    # sleep to ensure drop off is complete
    # runloop.sleep_ms(1000)
    doInit()

    # move horizontal rack to left to avoid collision with lights and sound
    motor.run_for_degrees(port.B, 500, -500) # move rack left
    
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
    await motor.run_for_degrees(port.C, -125, 200)

    # move robot backward to align with lights
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-150, target_angle=42, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(1.5))

    # Bring lever back to avoid hitting Noah and audience member
    motor.run_for_degrees(port.C, 100, 200)

    # turn left to complete alignment with lights lever
    await pivot_gyro_turn(-150, 0, 30, True)

    # move robot backward to pull light lever
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-150, target_angle=30, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(10))

    # align with hologram performer
    await pivot_gyro_turn(-300, 0, -20, True)

    # go back to base
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-500, target_angle=-20, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(58)))

    print("runFour -- END")

# run 5 code 
async def runFive():
    print("runFive -- START")
    # initialize motor pair
    # motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
    # # reset yaw to 0
    # motion_sensor.set_yaw_face(motion_sensor.TOP)
    # motion_sensor.reset_yaw(0)
    # while motion_sensor.stable() == False:
    #     await runloop.sleep_ms(100)
    #     print("Gyro NOT stable")
    #print("Gyro stable")
    # sleep to ensure drop off is complete
    # runloop.sleep_ms(1000)
    doInit()

    # move horizontal rack to left to avoid hitting holagram performer
    motor.run_for_degrees(port.C, 700, -500) # move rack left

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
    print("runFive -- END")

# run 6 code
async def runSix():
    print("runSix -- START")
    # # initialize motor pair
    # motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
    # # reset yaw to 0
    # motion_sensor.set_yaw_face(motion_sensor.TOP)
    # motion_sensor.reset_yaw(0)
    # while motion_sensor.stable() == False:
    #     await runloop.sleep_ms(100)
    #     print("Gyro NOT stable")
    #print("Gyro stable")
    # sleep to ensure drop off is complete
    #runloop.sleep_ms(1000)
    doInit()

    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=400, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(25))
    print("runSix -- END")

# do port check code
async def doPortCheck():
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
    
light.color(light.POWER, color.MAGENTA)
light_matrix.write("0")
start = time()
runloop.run(runOnePt1())
end = time()
print("Total run 1 pt 1 time =" + str(end-start))
