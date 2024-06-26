# Imports --------------------
from hub import light_matrix, button, motion_sensor, light, sound, port
import color, color_sensor, device, motor, motor_pair, orientation, runloop
import hub
import sys
import time

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
def wait_for_yaw_abs(angle=0):
    abs_angle = abs(angle)
    abs_current_yaw = abs(get_yaw_value())
    if angle == 0:
        if get_yaw_value() > 0:
            while get_yaw_value() >= angle: runloop.sleep_ms(10)
        elif get_yaw_value() < 0:
            while get_yaw_value() <= angle: runloop.sleep_ms(10)
    elif abs_current_yaw > abs_angle:
        while abs(get_yaw_value()) >= abs_angle: runloop.sleep_ms(10)
    elif abs_current_yaw < abs_angle:
        while abs(get_yaw_value()) <= abs_angle: runloop.sleep_ms(10)

def spin_gyro_turn_abs(steer=100, speed=50, angle=90, stop=False):
    motor_pair.move(motor_pair.PAIR_1, steer, velocity=speed, acceleration=100)
    wait_for_yaw_abs(angle=angle)
    if stop: motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)

async def pivot_gyro_turn_abs(left_speed=0, right_speed=50, angle=90, stop=False):
    motor_pair.move_tank(motor_pair.PAIR_1, left_speed, right_speed)
    # print("pivot_gyro_turn - " + "target angle=" + str(angle) + "current angle ="+ str(get_yaw_value()))
    wait_for_yaw_abs(angle=angle)
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

def follow_for_both_white():
    if (get_right_color_value() >= 90 and get_left_color_value() >= 90):
        return False
    else:
        return True

def follow_for_both_black():
    if (get_right_color_value() <= 20 and get_left_color_value() <= 20):
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
    # reset yaw to 0
    motion_sensor.set_yaw_face(motion_sensor.TOP)
    motion_sensor.reset_yaw(0)
    runloop.sleep_ms(1000)
    # while (hub.motion_sensor.stable() == False):
    #    runloop.sleep_ms(100)

# run one code
async def runOne():
    print("runOne -- START")
    # reset gyro
    doInit()

    # go forward to get out of base
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-350, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(30))

    # turn to align with masterpiece
    await pivot_gyro_turn_abs(-100, 0, -55, True)

    # go forward to drop masterpice and Anna into museum
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-350, target_angle=-60, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(85))

    # go forward to align with immersive experience
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=350, target_angle=-65, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(43.5))

    # turn to align with immersive experience
    await pivot_gyro_turn_abs(-100, 0, -87, True)

    # go forward to do immersive experience
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-250, target_angle=-87, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(25))

    # go forward to do immersive experience p2
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-350, target_angle=-87, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(20))

    # stabilize the robot
    await runloop.sleep_ms(500)

    # come back to turn for theatre scene change
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=200, target_angle=-90, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(12))

    # turn to align to theater scene change
    await pivot_gyro_turn_abs(0, -100, 0, True)

    # go forward until right black
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=300, target_angle=0, sleep_time=0, follow_for=follow_for_right_black)

    # bring rack down to push theater scene change
    await motor.run_for_degrees(port.B, 2000, 1000)

    # bring rack up to push theater scene change
    await motor.run_for_degrees(port.B, -500, 1000)

    # bring rack up to push theater scene change
    await motor.run_for_degrees(port.B, 500, 1000)

    # go back to get to base
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-350, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(19))

    # go forward to substitute for the 3 cm
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=250, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(6.5))

    # turn around to align to go back to base pt1
    await pivot_gyro_turn_abs(-100, 100, -45, True)

    # go forward to align with base
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degreesForDistance(10), 0, velocity=600)

    # turn to not crash into 3D cinema pt2
    await pivot_gyro_turn_abs(-100, 100, -70, True)

    # go back to get into base pt2
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=600, target_angle=-70, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(40))

    # turn again
    await pivot_gyro_turn_abs(100, -100, -20, True)

    # get completely into base
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=600, target_angle=-20, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(19))

    print("runOne -- End")

async def runTwo():
    print("runTwo -- Start")
    # initialize gyro
    doInit()

    # get out of base to approach 3D cinema
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=350, target_angle=-5, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(23))

    # turn to flick 3D cinema lever
    await pivot_gyro_turn_abs(300, -300, 15, True)

    # turn back to 0
    await pivot_gyro_turn_abs(-200, 200, 0, True)

    # raise 3D cinema attachment
    motor.run_for_degrees(port.C, 1500, 1000)

    # come back to base and align with sound mixer
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-400, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(25))

    # sleep for 2 seconds to quickly align robot for reliability
    await runloop.sleep_ms(2000)

    # turn to align to sound mixer
    await pivot_gyro_turn_abs(100, 0, 41, True)

    # approach sound mixer
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=250, target_angle=41, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(44))

    # bring Noah L hook down to pick him up
    await motor.run_for_degrees(port.B, -1000, 800)

    # come back to release sliders on left and right
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-250, target_angle=41, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(8))

    # turn to align to last two sliders to raise middle one
    await pivot_gyro_turn_abs(25, 0, 43, True)

    # go forward and lift last two sliders
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degreesForDistance(8), 0, velocity=150)

    # await sound.beep()
    # print("yaw value after:" + str(get_yaw_value()))

    # come back to base
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-600, target_angle=46, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(50))
    print("runTwo -- End")

# run three code
async def runThree():
    print("runThree -- START")
    # initialize motor pair and reset yaw
    doInit()

    # go forward to align with rolling camera lever
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=300, target_angle=1, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(37))

    # bring movie set attachment down to latch to orange hoop
    motor.run_for_degrees(port.C, 500, -1000)

    # bring rolling camera attachment down
    await motor.run_for_degrees(port.B, 125, -1000)

    # turn to flick rolling camera lever
    await pivot_gyro_turn_abs(0, 100, -5, True)

    # lift rolling camera lever attachment to get out of the way of other missions
    motor.run_for_degrees(port.B, 2200, 1050)

    # bring robot back to turn to align to camera target area
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-250, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(21))

    # turn to align to camera target area
    await pivot_gyro_turn_abs(0, 100, -15, True)

    # go forward to drop camera into target area
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=250, target_angle=-15, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(5))

    # lift movie set lever attachment to get out of the way of other missions
    await motor.run_for_degrees(port.C, 250, 1050)

    # lift movie set lever attachment to get out of the way of other missions
    motor.run_for_degrees(port.C, 2250, 1050)

    # come back to put buddha and take off rolling camera attachment
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-250, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(20))

    # sleep to put buddha and take off rolling camera attachment
    await runloop.sleep_ms(5000)

    # turn robot to launch next mission
    await pivot_gyro_turn_abs(0, 100, -35, True)

    # move forward to go to light show to drop of audience member and do light show
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=400, target_angle=-35, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(40))

    # turn robot to launch next mission
    await pivot_gyro_turn_abs(50, -50, -18, True)

    # move forward beyond the camera and head towards the light show
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=400, target_angle=-18, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(20))

    # move forward for light show until white, then black
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=400, target_angle=-18, sleep_time=0, follow_for=follow_for_left_white)
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=400, target_angle=-18, sleep_time=0, follow_for=follow_for_left_black)

    await sound.beep()

    # do light show
    await motor.run_for_degrees(port.B, 1730, -1000)

    motor.reset_relative_position(port.A, 0)
    motor.reset_relative_position(port.E, 0)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degreesForDistance(6), int(0), velocity=-100)

    # turn robot to align with right base
    await pivot_gyro_turn_abs(100, -100, 15, True)

    # move robot forward to pickup emily and go to base
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=500, target_angle=15, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(20))

    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=900, target_angle=15, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(85))

    print("runThree -- END")

# run Five code
async def runFive():
    print("runFour -- START")
    # initialize motor pair
    doInit()

    # move robot forward to go towards rolling camera
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=750, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(31.4)))

    # move robot backward to get back to base
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-250, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(29.8)))

    print("runFive -- END")

# run Four code
async def runFour():
    print("runFive -- START")
    # initialize motor pair
    doInit()

    # move robot forward to start going to craft creator
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=300, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(35.5)))

    # lift Izzy
    await motor.run_for_degrees(port.B, 2500, 1100, stop=motor.HOLD)
    motor.run_for_degrees(port.B, 500, 1100, stop=motor.HOLD)

    # move robot forward to latch onto craft creator lid
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=200, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(8)))

    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degreesForDistance(1.5), 0)

    # complete virtual reality
    await motor.run_for_degrees(port.C, 1500, 500, stop=motor.BRAKE)

    # move robot backward to lift craft creator lid
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-300, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(30)))

    # move robot backward to lift craft creator lid
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-600, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(20)))

    # Turn left to align with craft creator
    # await pivot_gyro_turn_abs(-100, 0, -35, True)

    print("runFour -- END")

async def runSix():
    print("runSix -- START")
    # initialize motor pair
    doInit()

    # move forward to get out of base
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=200, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(26)))

    # Turn to align for audience drop of
    await pivot_gyro_turn_abs(100, -100, 44, True)

    # move horizontal rack to left to avoid collision with lights and sound
    motor.run_for_degrees(port.B, -900, 720) # move rack left

    # move robot to approach audience drop off with checkpoints along the way
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=400, target_angle=44, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(14)))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=400, target_angle=44, sleep_time=0, follow_for=follow_for_right_white)
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=300, target_angle=44, sleep_time=0, follow_for=follow_for_right_black)

    # move forward to complete audience drop off
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=200, target_angle=44, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(15.5)))

    # move robot backward to align with hologram performer
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-200, target_angle=44, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(16)))

    # align with hologram performer
    await pivot_gyro_turn_abs(80, 0, 87, True)

    # # move forward to push hologram performer lever and complete the mission
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degreesForDistance(16),0, velocity=80)

    # Reset gyro to 0
    doInit()

    # Rotate left motor anti clockwise to flick the lever of sound speaker
    await motor.run_for_degrees(port.C, -120, 400)

    # bring lever back to avoid hitting Noah and audience member
    motor.run_for_degrees(port.C, 100, 300)

    # go backward to pull lights lever
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degreesForDistance(16), 0, velocity=-160)

    # turn left start alignment with augmented reality
    await pivot_gyro_turn_abs(-100, 100, -90, True)

    # move robot forward to align with augmented reality
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=250, target_angle=-90, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(3.5))

    # move robot backward to checkpoint
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-200, target_angle=-90, sleep_time=0, follow_for=follow_for_right_white)
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-200, target_angle=-90, sleep_time=0, follow_for=follow_for_right_black)

    # move robot forward to align with augmented reality
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=200, target_angle=-90, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(13.5))

    # turn left to complete alignment with augmented reality
    await pivot_gyro_turn_abs(-100, 100, -132, True)

    # move robot forward to align with augmented reality lever
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=400, target_angle=-132, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(18))

    # move horizontal rack to engage with augmented reality lever
    await motor.run_for_degrees(port.B, 720, 1050) # move rack right

    # move robot backward to engage with augmented reality lever
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-200, target_angle=-132, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(5.2))

    # move horizontal rack to left to pull augmented reality lever back
    await motor.run_for_degrees(port.B, 600, -1050) # move rack left

    # move robot backward to pull augmented reality lever
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-400, target_angle=-132, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(15))

    # move robot forward to go to audience drop off
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=550, target_angle=-132, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(50))

    # move horizontal rack to ensure Emily and audience member are in the popcorn
    await motor.run_for_degrees(port.B, 120, -1050) # move rack left

    # move robot forward to go to audience drop off
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=550, target_angle=-132, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(25))

    # move robot forward to go to audience drop off
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=550, target_angle=-136, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(18))

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

# Test for gyro tuning
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

# Return true if LEFT button is pressed
def is_left_button_pressed():
    return button.pressed(button.LEFT) > 0

# Return true if RIGHT button is pressed
def is_right_button_pressed():
    return button.pressed(button.RIGHT) > 0

# Main program which drives all the runs. LEFT button is used to trigger runs sequentially, starting with 0 and going till 6
async def mainProgram():
    start1 = time.ticks_ms()
    start2 = time.ticks_ms()
    start3 = time.ticks_ms()
    start4 = time.ticks_ms()
    start5 = time.ticks_ms()
    start6 = time.ticks_ms()
    end1 = time.ticks_ms()
    end2 = time.ticks_ms()
    end3 = time.ticks_ms()
    end4 = time.ticks_ms()
    end5 = time.ticks_ms()
    end6 = time.ticks_ms()

    print("mainProgram -- START")
    # initialize motor pair
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)

    light_matrix.write("0")
    light.color(light.POWER, color.RED)

    # reset yaw to 0
    motion_sensor.set_yaw_face(motion_sensor.TOP)
    motion_sensor.reset_yaw(0)
    i = 0
    while (hub.motion_sensor.stable() == False):
        i = i+1
        await runloop.sleep_ms(100)
        hub.light_matrix.write(str(i))

    # set initial program
    program = 1
    light_matrix.write("1")
    light.color(light.POWER, color.WHITE)

    while (True):
        print("while true....")

        await runloop.until(is_left_button_pressed)
        print("starting runs....")
        if (program == 1):
            print("entering program 1")
            light.color(light.POWER, color.MAGENTA)
            light_matrix.show_image(light_matrix.IMAGE_BUTTERFLY)
            start1 = time.ticks_ms()
            runloop.run(runOne())
            end1 = time.ticks_ms()
            light.color(light.POWER, color.YELLOW)
            print("done program 1")
            program = 2
            light_matrix.write("2")
        elif (program == 2):
            print("entering program 2")
            light.color(light.POWER, color.MAGENTA)
            light_matrix.show_image(light_matrix.IMAGE_BUTTERFLY)
            start2 = time.ticks_ms()
            runloop.run(runTwo())
            end2 = time.ticks_ms()
            light.color(light.POWER, color.YELLOW)
            print("done program 2")
            program = 3
            light_matrix.write("3")
        elif (program == 3):
            print("entering program 3")
            light.color(light.POWER, color.MAGENTA)
            light_matrix.show_image(light_matrix.IMAGE_BUTTERFLY)
            start3 = time.ticks_ms()
            runloop.run(runThree())
            end3 = time.ticks_ms()
            light.color(light.POWER, color.YELLOW)
            print("done program 3")
            program = 4
            light_matrix.write("4")
        elif (program == 4):
            print("entering program 4")
            light.color(light.POWER, color.MAGENTA)
            light_matrix.show_image(light_matrix.IMAGE_BUTTERFLY)
            start4 = time.ticks_ms()
            runloop.run(runFour())
            end4 = time.ticks_ms()
            light.color(light.POWER, color.YELLOW)
            print("done program 4")
            program = 5
            light_matrix.write("5")
        elif (program == 5):
            print("entering program 5")
            light.color(light.POWER, color.MAGENTA)
            light_matrix.show_image(light_matrix.IMAGE_BUTTERFLY)
            start5 = time.ticks_ms()
            runloop.run(runFive())
            end5 = time.ticks_ms()
            light.color(light.POWER, color.YELLOW)
            print("done program 5")
            print("Total run 5 =" + str(time.ticks_diff(end5, start5)/1000))
            program = 6
            light_matrix.write("6")
        elif (program == 6):
            print("entering program 6")
            light.color(light.POWER, color.MAGENTA)
            light_matrix.show_image(light_matrix.IMAGE_BUTTERFLY)
            start6 = time.ticks_ms()
            runloop.run(runSix())
            end6 = time.ticks_ms()
            light.color(light.POWER, color.YELLOW)
            print("done program 6")
            program = 0
            light_matrix.write("0")

            print("Total run 1 =" + str(time.ticks_diff(end1, start1)/1000))
            print("Total run 2 =" + str(time.ticks_diff(end2, start2)/1000))
            print("Total run 3 =" + str(time.ticks_diff(end3, start3)/1000))
            print("Total run 4 =" + str(time.ticks_diff(end4, start4)/1000))
            print("Total run 5 =" + str(time.ticks_diff(end5, start5)/1000))
            print("Total run 6 =" + str(time.ticks_diff(end6, start6)/1000))
            print("Transition 1 to 2 =" + str(time.ticks_diff(start2, end1)/1000))
            print("Transition 2 to 3 =" + str(time.ticks_diff(start3, end2)/1000))
            print("Transition 3 to 4 =" + str(time.ticks_diff(start4, end3)/1000))
            print("Transition 4 to 5 =" + str(time.ticks_diff(start5, end4)/1000))
            print("Transition 5 to 6 =" + str(time.ticks_diff(start6, end5)/1000))
            sys.exit(1)

runloop.run(mainProgram())
