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
    # while motion_sensor.stable() == False:
    #    runloop.sleep_ms(100)
    #    print("Gyro NOT stable")
    # #print("Gyro stable")

# run onenew code
async def runOne():
    print("runOne -- START")
    # reset gyro
    doInit()

    # go backward for 45 cm to get to thatre scene change
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-400, target_angle=23, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(45))

    # go back till right sensor is on black for a checkpoint towards theatre scene change
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-300, target_angle=23, sleep_time=0, follow_for=follow_for_right_black)

    # turn around to get in position for theatre scene change
    await pivot_gyro_turn(50, 0, 43, True)

    # go forward for 2 cm to get to thatre scene change
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=200, target_angle=43, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(2))

    # go backward until both sensors are on black for a checkpoint for theatre scene change
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-100, target_angle=43, sleep_time=0, follow_for=follow_for_both_black)
    sound.beep()

    # turn around to get in position for theatre scene change
    await pivot_gyro_turn(30, -30, 90, True)

    # go backward to get in position to push theater scene change lever
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-200, target_angle=90, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(1.5))

    # lift up Sam
    motor.run_for_degrees(port.B, 2000, 500)

    # turn around to get in position for theatre scene change
    await pivot_gyro_turn(50, -50, 95, True)

    # go bacward to get in postion for theatre scene change lever
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-200, target_angle=95, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(1.5))
    
    # bring rack down for theatre scene change
    await motor.run_for_degrees(port.C, 2000, 500)

    # raise rack up
    motor.run_for_degrees(port.C, 900, -500)

    # turn
    await pivot_gyro_turn(-50, 50, 85, True)

    # go backward away from theatre scene change and towards immersive experience
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-200, target_angle=85, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(1))

    # go backwards
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-200, target_angle=90, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(52))

    # bring the rack down to push the immersive experience
    await motor.run_for_degrees(port.C, 1600, 1000)

    # sleep for 0.5 seconds
    runloop.sleep_ms(500)

    # bring rack up to disengage with immersive experience
    await motor.run_for_degrees(port.C, 1000, -1000)

    # go backward away from immersive experience and towards light show
    # motor.reset_relative_position(port.A, 0)
    # position = abs(motor.relative_position(port.A))
    # await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-200, target_angle=90, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(0.5))

    # turn around to get the pieces in masterpiece
    doInit()
    await pivot_gyro_turn(-50, 25, -88, True)

    # bring rack down to align with light show
    await motor.run_for_degrees(port.C, 1000, 1000)

    # go forward 12 cm to align with light show
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=100, target_angle=-88, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(12))

    # bring rack up to complete light show
    await motor.run_for_degrees(port.C, 1700, -1000)

    # go backward to drop pieces in masterpiece
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-100, target_angle=-88, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(17))

    # go forward to leave pieces in masterpiece
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=100, target_angle=-88, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(3))

    # pivot turn 50 degrees to get Noah
    doInit()
    await pivot_gyro_turn(50, -50, 38, True)

    # go forward to get Noah
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=500, target_angle=36, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(65))

    # come back to base
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=700, target_angle=80, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(40))

async def runTwo():
    # initialize gyro
    doInit()

    # get out of base to approach 3D cinema
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=350, target_angle=-5, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(25))

    # turn to flick 3D cinema lever
    await pivot_gyro_turn(0, -65, 10, True)

    # raise 3D cinema attachment
    motor.run_for_degrees(port.C, 2000, 1000)

    # come back to base and align with sound mixer
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-350, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(25))

    # turn to align to sound mixer
    await pivot_gyro_turn(100, 0, 43, True)

    # approach sound mixer
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=275, target_angle=43, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(47.5))

    # turn right to release sound mixer lever
    await pivot_gyro_turn(0, -50, 60, True)

    # turn to align to movie set
    await pivot_gyro_turn(150, 0, 118, True)

    # go forward to movie set
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=200, target_angle=118, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(14.2))

    # bring movie set attachment down
    await motor.run_for_degrees(port.B, -940, 1000)

    # go backward to pull movie set
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-200, target_angle=115, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(14.2))

    # bring movie set attahcment up
    await motor.run_for_degrees(port.B, 940, 1100)

    # sleep to ensure robot pulls back movie set
    runloop.sleep_ms(250)

    # come back to get out of the way of movie set
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-200, target_angle=115, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(5))

    # turn to align to base
    doInit()
    await pivot_gyro_turn(600, 0, 120, True)

    # go forward to base
    motor.reset_relative_position(port.A, 0)
    motor.reset_relative_position(port.E, 0)
    motor_pair.move_for_degrees(motor_pair.PAIR_1, degreesForDistance(17), int(0), velocity=900)

# run three code
async def runThree():
    print("runThree -- START")
    # initialize motor pair and reset yaw
    doInit()

    # go forward to align with rolling camera lever
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=200, target_angle=1, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(39))
    
    # bring rolling camera attachment down to push down rolling camera lever
    await motor.run_for_degrees(port.B, 125, -1000)

    # turn to flick rolling camera lever
    await pivot_gyro_turn(0, 100, -5, True)

    # lift rolling camera lever attachment to get out of the way of other missions
    motor.run_for_degrees(port.B, 3300, 1000)
   
    # bring robot back to base to add audience members and experts
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-200, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(39))

    # sleep to put figures
    await runloop.sleep_ms(2000)

    # turn robot to launch next mission
    await pivot_gyro_turn(0, 100, -30, True)

    # move forward to go to light show to drop of audience member and avoid hitting camera
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=500, target_angle=-30, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(40))

    # turn robot to launch next mission
    await pivot_gyro_turn(0, 100, -20, True)

    # move forward beyond the camera and head towards the light show  
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=500, target_angle=-20, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(30))

    # move robot back to drop of audience member near light show
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-500, target_angle=-20, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(4))

    # turn robot to align with clapboard
    await pivot_gyro_turn(100, 0, 60, True)

    # move robot forward to drop sam and audience member
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=200, target_angle=60, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(2))

    # move robot back to complete sam and audience member drop of
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-200, target_angle=45, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(4))

    # move robot forward to pickup emily and go to base
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=400, target_angle=20, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(20))


    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=800, target_angle=20, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(70))

    print("run3 -- END")

# run four code
async def runFour():
    print("runThree -- START")
    # initialize motor pair
    doInit()

    # move robot forward to start going to craft creator
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=200, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(28)))

    # Turn left to align with craft creator
    await pivot_gyro_turn(-100, 100, -42, True)

    # move robot forward to start going to craft creator
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=200, target_angle=-45, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(6)))

    # lift Izzy
    # await motor.run_for_degrees(port.C, 4000, 1000, stop=motor.HOLD)

    # move robot forward to latch onto craft creator lid
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=200, target_angle=-45, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(18)))

    # move robot backward to lift craft creator lid
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-200, target_angle=-45, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(35)))

    # Turn to bring attachment completely in base
    await pivot_gyro_turn(150, -150, 60, True)
    print("runThree -- END")

# run five code
async def runFive():
    print("runFour -- START")
    # initialize motor pair
    doInit()

    # move horizontal rack to left to avoid collision with lights and sound
    # motor.run_for_degrees(port.B, 500, -500) # move rack left

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
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-200, target_angle=-3, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(10.5)))

    # align with hologram performer
    await pivot_gyro_turn(0, -80, 42, True)

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

    # Bring lever back to avoid hitting Noah and audience member
    motor.run_for_degrees(port.C, 100, 200)

    # turn left to complete alignment with lights lever
    await pivot_gyro_turn(-120, 0, 28, True)

    # move robot backward to pull light lever
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-250, target_angle=28, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=degreesForDistance(12))

    # allow light lever to fall down, hence sleep
    runloop.sleep_ms(500)

    # turn left to return to base
    await pivot_gyro_turn(-300, 0, -20, True)

    # go back to base
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-500, target_angle=-20, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(58)))

    print("runFour -- END")

# run six code
async def runSix():
    print("runFive -- START")
    # initialize motor pair
    doInit()

    # move horizontal rack to left to avoid hitting holagram performer
    motor.run_for_degrees(port.C, 750, -500) # move rack left

    # move robot forward to start going to augmented reality
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=400, target_angle=0, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(34)))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=300, target_angle=0, sleep_time=0, follow_for=follow_for_right_white)
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=300, target_angle=0, sleep_time=0, follow_for=follow_for_right_black)

    # turning to go towards augmented reality
    await pivot_gyro_turn(0, 100, -80, True)

    # move forward to start aligning with augmented reality
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=400, target_angle=-80, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(9)))

    # move forward to start aligning with augmented reality
    # motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=400, target_angle=-90, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(16)))

    # move horizontal rack to right to align with augmented reality hook
    await motor.run_for_degrees(port.C, 750, 500) # move rack right

    # move backward to hook on to augmented reality
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-300, target_angle=-90, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(6)))

    # move horizontal rack to left to pull augmented reality lever
    await motor.run_for_degrees(port.C, 625, -500) # move rack left

    # move backward to complete augmented reality
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-300, target_angle=-90, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(8)))

    # move horizontal rack to left to pull augmented reality lever
    await motor.run_for_degrees(port.C, 75, -500) # move rack left

    # move forward to align with museum
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=400, target_angle=-90, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(25)))

    # turning right to align for anna and audience member drop of
    await pivot_gyro_turn(80, 0, -30, True)

    # move forward to drop anna and audience member
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=400, target_angle=-30, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(23)))

    # move backward to leave anna and audience member
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=1.4, ki=0, kd=0, speed=-400, target_angle=-30, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(14)))

    # turning back so robot can go back to base
    await pivot_gyro_turn(0, 100, -85, True)

    # move horizontal rack to left to avoid hitting light show
    # await motor.run_for_degrees(port.C, 120, -500) # move rack left

    # move forward to drop anna and audience member
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=400, target_angle=-90, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(28)))

    # move horizontal rack to left to avoid hitting 3d cinema
    motor.run_for_degrees(port.C, 350, -500) # move rack left

    # move forward to retun to base
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=400, target_angle=-90, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(34)))

    # turning back so robot can go back to base
    await pivot_gyro_turn(0, 100, -160, True)

    # move forward to drop anna and audience member
    motor.reset_relative_position(port.A, 0)
    position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.4, ki=0, kd=0, speed=500, target_angle=-160, sleep_time=0, follow_for=follow_for_distance, initial_position=position, distance_to_cover=(degreesForDistance(55)))
    print("runFive -- END")

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
    start0 = time.ticks_ms()
    start1 = time.ticks_ms()
    start2 = time.ticks_ms()
    start3 = time.ticks_ms()
    start4 = time.ticks_ms()
    start5 = time.ticks_ms()
    start6 = time.ticks_ms()
    end0 = time.ticks_ms()
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
        if (program == 0):
            print("entering program 0")
            light.color(light.POWER, color.MAGENTA)
            light_matrix.show_image(light_matrix.IMAGE_BUTTERFLY)
            start0 = time.ticks_ms()
            runloop.run(runOne())
            end0 = time.ticks_ms()
            prg0 = time.ticks_diff(end0, start0)
            light.color(light.POWER, color.YELLOW)
            print("done program 0")
            program = 1
            light_matrix.write("1")
        elif (program == 1):
            print("entering program 1")
            light.color(light.POWER, color.MAGENTA)
            light_matrix.show_image(light_matrix.IMAGE_BUTTERFLY)
            start1 = time.ticks_ms()
            runloop.run(runOne())
            end1 = time.ticks_ms()
            prg1 = time.ticks_diff(end1, start1)
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
            prg2 = time.ticks_diff(end2, start2)
            light.color(light.POWER, color.YELLOW)
            print("done program 2")
            print("Total run 2 =" + str(time.ticks_diff(end2, start2)/1000))
            program = 3
            light_matrix.write("3")
        elif (program == 3):
            print("entering program 3")
            light.color(light.POWER, color.MAGENTA)
            light_matrix.show_image(light_matrix.IMAGE_BUTTERFLY)
            start3 = time.ticks_ms()
            prg3 = time.ticks_diff(end3, start3)
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
            prg4 = time.ticks_diff(end4, start4)
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
            prg5 = time.ticks_diff(end5, start5)
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
            prg6 = time.ticks_diff(end6, start6)
            light.color(light.POWER, color.YELLOW)
            print("done program 6")
            program = -1
            light_matrix.write("-1")

            print("Total run 1 pt 1 =" + str(time.ticks_diff(end0, start0)/1000))
            print("Total run 1 pt 2 =" + str(time.ticks_diff(end1, start1)/1000))
            print("Total run 2 =" + str(time.ticks_diff(end2, start2)/1000))
            print("Total run 3 =" + str(time.ticks_diff(end3, start3)/1000))
            print("Total run 4 =" + str(time.ticks_diff(end4, start4)/1000))
            print("Total run 5 =" + str(time.ticks_diff(end5, start5)/1000))
            print("Total run 6 =" + str(time.ticks_diff(end6, start6)/1000))

            print("Transition 1pt1 to 1pt2 =" + str(time.ticks_diff(start1, end0)/1000))
            print("Transition 1pt2 to 2 =" + str(time.ticks_diff(start2, end1)/1000))
            print("Transition 2 to 3 =" + str(time.ticks_diff(start3, end2)/1000))
            print("Transition 3 to 4 =" + str(time.ticks_diff(start4, end3)/1000))
            print("Transition 4 to 5 =" + str(time.ticks_diff(start5, end4)/1000))
            print("Transition 5 to 6 =" + str(time.ticks_diff(start6, end5)/1000))

            sys.exit("sucessfully exited")


runloop.run(mainProgram())
