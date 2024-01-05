# Imports --------------------
from hub import light_matrix, button, motion_sensor, light, sound, port
import color, color_sensor, device, motor, motor_pair, orientation, runloop
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
def wait_for_yaw(angle=90):
    if angle > 0:
        while get_yaw_value() <= angle: runloop.sleep_ms(10)
    elif angle < 0:
        while get_yaw_value() >= angle: runloop.sleep_ms(10)
    else:
        while get_yaw_value() != angle: runloop.sleep_ms(10)

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

def spin_gyro_turn(steer=100, speed=50, angle=90, stop=False):
    motor_pair.move(motor_pair.PAIR_1, steer, velocity=speed, acceleration=100)
    wait_for_yaw(angle=angle)
    if stop: motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)

async def pivot_gyro_turn(left_speed=0, right_speed=50, angle=90, stop=False):
    motor_pair.move_tank(motor_pair.PAIR_1, left_speed, right_speed)
    # print("pivot_gyro_turn - " + "target angle=" + str(angle) + "current angle ="+ str(get_yaw_value()))
    wait_for_yaw(angle=angle)
    if stop: motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)

async def pivot_gyro_turn_abs(left_speed=0, right_speed=50, angle=90, stop=False, accel=1000):
    motor_pair.move_tank(motor_pair.PAIR_1, left_speed, right_speed, acceleration=accel)
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

# run onenew code
async def runOne():
    print("runOne -- START")
    # reset gyro
    doInit()

    # pivot turn to 19
    await pivot_gyro_turn_abs(120, -120, 20, True, 5000)

    # go backward for 45 cm to get to theatre scene change
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-400, target_angle=20, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(45))

    # go back till right sensor is on black for a checkpoint towards theatre scene change
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-300, target_angle=20, sleep_time=0, follow_for=follow_for_right_black)

    # turn around to get in position for theatre scene change
    await pivot_gyro_turn_abs(100, 0, 43, True)

    # go forward to look for a checkpoint near theatre scene change
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=200, target_angle=43, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(3))

    # go backward until left sensor is on black for a checkpoint for theatre scene change
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-100, target_angle=43, sleep_time=0, follow_for=follow_for_left_black)

    # go down to complete theatre scene change
    motor.run_for_degrees(port.C, 2200, 1050)

    # bring motor B down to align with Sam
    await motor.run_for_degrees(port.B, 2250, -1000)

    # get rack back up
    motor.run_for_degrees(port.C, 1500, -1000)

    # pivot turn with left wheel going reverse to get the hooks inside Sam
    await pivot_gyro_turn_abs(-100, 0, 20, True)

    # bring up motor B to pick up Sam
    await motor.run_for_degrees(port.B, 1000, 1000)
    motor.run_for_degrees(port.B, 1150, 1000) # run the rest in parallel to next step

    # pivot turn to go towards immersive experience
    await pivot_gyro_turn_abs(0, -200, 122, True)

    # go backwards
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-200, target_angle=122, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(20))

    # turn to get in line for immersive experience
    await pivot_gyro_turn_abs(-100, 100, 90, True)

    # go backwards pt.2
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-300, target_angle=90, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(30))

    # bring the rack down to push the immersive experience
    await motor.run_for_degrees(port.C, 1550, 1050)

    # sleep to make sure immersive experience doesn't bounce back
    runloop.sleep_ms(1000)

    # bring rack up to disengage with immersive experience
    await motor.run_for_degrees(port.C, 1000, -1000)

    # turn around to drop audience member and anna to masterpiece
    await pivot_gyro_turn_abs(-100, 100, 30, True)

    # bring the rack down
    motor.run_for_degrees(port.C, 1740, 1000)

    # go backwards
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-200, target_angle=30, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(4))

    # turn around towards light show completly
    await pivot_gyro_turn_abs(-100, 100, 0, True)

    # go forward
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=200, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(15))

    # bring rack up to complete light show
    await motor.run_for_degrees(port.C, 1555, -1000)

    # go back to get the pieces fully in mastepiece
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-200, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(16))

    # turn to get to base
    await pivot_gyro_turn_abs(100, -100, 34, True)

    # go forward to get Noah and go back to base
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=700, target_angle=34, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(65))

    # come back to base
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=700, target_angle=65, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(45))
    print("runOne -- End")


async def runTwo():
    print("runTwo -- Start")
    # initialize gyro
    doInit()

    # get out of base to approach 3D cinema
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=350, target_angle=-5, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(25))

    # turn to flick 3D cinema lever
    await pivot_gyro_turn_abs(0, -100, 10, True)

    # raise 3D cinema attachment
    motor.run_for_degrees(port.C, 2000, 1000)

    # come back to base and align with sound mixer
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-350, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(25))

    # turn to align to sound mixer
    await pivot_gyro_turn_abs(100, 0, 43, True)

    # approach sound mixer
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=275, target_angle=43, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(47.5))

    # turn right to release sound mixer lever
    await pivot_gyro_turn_abs(0, -50, 60, True)

    # turn to align to movie set
    await pivot_gyro_turn_abs(150, 0, 118, True)

    # go forward to movie set
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=200, target_angle=118, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(14.2))

    # bring movie set attachment down
    await motor.run_for_degrees(port.B, -940, 1000)

    # go backward to pull movie set
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-200, target_angle=115, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(19.2))

    # bring movie set attahcment up
    await motor.run_for_degrees(port.B, 940, 1100)

    # sleep to ensure robot pulls back movie set
    # san # runloop.sleep_ms(250)

    # come back to get out of the way of movie set
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-200, target_angle=115, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(7))

    # turn to align to base
    doInit()
    await pivot_gyro_turn(600, 0, 120, True)

    # go backward to base
    motor.reset_relative_position(port.A, 0)
    motor.reset_relative_position(port.E, 0)
    motor_pair.move_for_degrees(motor_pair.PAIR_1, degreesForDistance(30), int(0), velocity=800)
    print("runTwo -- End")

# run three code
async def runThree():
    print("runThree -- START")
    # initialize motor pair and reset yaw
    doInit()

    # go forward to align with rolling camera lever
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=300, target_angle=1, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(39))

    # bring rolling camera attachment down to push down rolling camera lever
    await motor.run_for_degrees(port.B, 125, -1000)

    # turn to flick rolling camera lever
    await pivot_gyro_turn_abs(0, 100, -5, True)

    # lift rolling camera lever attachment to get out of the way of other missions
    motor.run_for_degrees(port.B, 2900, 1050)

    # bring robot back to base to add audience members and experts
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-250, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(39))

    # sleep to put audiece members and Sam
    await runloop.sleep_ms(1000)

    # turn robot to launch next mission
    await pivot_gyro_turn_abs(0, 100, -35, True)

    # move forward to go to light show to drop of audience member and avoid hitting camera
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=800, target_angle=-35, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(40))

    # turn robot to launch next mission
    await pivot_gyro_turn(0, 100, -20, True)

    # move forward beyond the camera and head towards the light show
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=400, target_angle=-20, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(32.1))

    motor.reset_relative_position(port.A, 0)
    motor.reset_relative_position(port.E, 0)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degreesForDistance(1), int(0), velocity=-100)

    # turn robot to align with clapboard
    await pivot_gyro_turn_abs(150, 0, 50, True)

    # move robot forward to drop sam and audience member
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=200, target_angle=50, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(10))

    # move robot back to complete sam and audience member drop of
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-400, target_angle=50, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(13))

    # turn robot to align with right base
    await pivot_gyro_turn_abs(-80, 80, 15, True)

    # move robot forward to pickup emily and go to base
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=500, target_angle=15, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(20))

    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=900, target_angle=15, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(70))

    print("runThree -- END")

# run Four code
async def runFour():
    print("runFour -- START")
    # initialize motor pair
    doInit()

    # move horizontal rack to left to avoid collision with lights and sound
    motor.run_for_degrees(port.B, 900, -500) # move rack left

    # move robot to approach audience drop off with checkpoints along the way
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=400, target_angle=-3, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(34)))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=400, target_angle=-3, sleep_time=0, follow_for=follow_for_right_white)
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=300, target_angle=-3, sleep_time=0, follow_for=follow_for_right_black)

    # move forward to complete audience drop off
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=200, target_angle=-3, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(15.5)))

    # move robot backward to align with hologram performer
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-200, target_angle=-3, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(16.5)))

    # align with hologram performer
    await pivot_gyro_turn_abs(80, 0, 42, True)

    # move forwward to push hologram performer lever and complete the mission
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=80, target_angle=42, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(15)))

    # Rotate left motor anti clockwise to flick the lever of sound speaker
    await motor.run_for_degrees(port.C, -125, 300)

    # move robot backward to align with lights
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-150, target_angle=42, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(0.8))

    # bring lever back to avoid hitting Noah and audience member
    motor.run_for_degrees(port.C, 100, 200)

    # turn left to complete alignment with lights lever
    await pivot_gyro_turn_abs(-100, 0, 34, True)

    # move robot backward to pull light lever
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-200, target_angle=34, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(15))

    # turn left start alignment with augmented reality
    await pivot_gyro_turn_abs(-100, 100, -35, True)

    # move robot forward to align with augmented reality
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=250, target_angle=-35, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(14))

    # turn left to complete alignment with augmented reality
    await pivot_gyro_turn_abs(-100, 100, -85, True)

    # move robot forward to align with augmented reality
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=400, target_angle=-85, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(15))

    # move horizontal rack to engage with augmented reality lever
    await motor.run_for_degrees(port.B, 850, 1050) # move rack right

    # move robot backward to engage with augmented reality lever
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-200, target_angle=-85, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(5))

    # move horizontal rack to left to pull augmented reality lever back
    await motor.run_for_degrees(port.B, 675, -1050) # move rack left

    # move robot backward to pull augmented reality lever
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-400, target_angle=-85, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(25))

    # turn right to go back to base
    await pivot_gyro_turn_abs(100, -100, -25, True)

    # go back to base
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-800, target_angle=-25, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(78)))
    print("runFour -- END")

# run Five code
async def runFive():
    print("runFive -- START")
    # initialize motor pair
    doInit()

    # move robot forward to go towards rolling camera
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=325, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(34)))

    # move robot backward to get in alignment for craft creator
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-250, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(24.8)))

    # Turn left to align with craft creator
    await pivot_gyro_turn_abs(100, -100, 45, True)

    # move robot forward to start going to craft creator
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=300, target_angle=45, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(29)))

    # lift Izzy
    await motor.run_for_degrees(port.B, 3000, 1000, stop=motor.HOLD)

    # move robot forward to latch onto craft creator lid
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=200, target_angle=45, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(9)))

    # complete virtual reality
    await motor.run_for_degrees(port.C, 1500, 500, stop=motor.BRAKE)

    # move robot backward to lift craft creator lid
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-300, target_angle=45, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(30)))

    # move robot backward to lift craft creator lid
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-600, target_angle=45, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(18)))

    # sleep to take of the attatchment
    await runloop.sleep_ms(3000)

    # start going back to base
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=600, target_angle=45, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(3)))

    # Turn to start going to base
    await pivot_gyro_turn_abs(-150, 150, 13, True)

    # start going back to base
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=600, target_angle=13, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(107)))

    # Turn to go in to base
    await pivot_gyro_turn_abs(-150, 150, -20, True)

    # go to base
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=700, target_angle=-20, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(58)))
    print("runFive -- END")

async def runSix():
    print("runSix -- START")
    # initialize motor pair
    doInit()

    # turn in base to align with target areas
    await pivot_gyro_turn_abs(100, -100, 5, True)

    # go forward to get out of base and approach target area
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=500, target_angle=5, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(45))

    # go back till left sensor is on black for a checkpoint towards theatre scene change
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=300, target_angle=5, sleep_time=0, follow_for=follow_for_left_black)

    # turn around to get in position of the skateboard
    await pivot_gyro_turn_abs(200, 0, 43, True)

    # go forawrd to drop Izzy and audience member into skateboard
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=800, target_angle=43, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(7))

    # come back to position robot so audience members and experts are in popcorn and drama icon
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-800, target_angle=43, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(9))
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
    program = 1
    light_matrix.write("1")
    light.color(light.POWER, color.WHITE)

    # initialize motor pair
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)

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
            sys.exit("sucessfully exited")

runloop.run(mainProgram())
