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

def test_front_left_spin():

    print(motor.relative_position(port.E))
    motor.run_for_degrees(port.E, -90, 100)
    print(motor.relative_position(port.E))
    time.sleep(1)
    motor.run_for_degrees(port.D, 2000, 100)
    print("Done!")

def test_front_right_spin():

    print(motor.relative_position(port.E))
    motor.run_for_degrees(port.E, 360, 100)
    print(motor.relative_position(port.E))
    time.sleep(1)
    motor.run_for_degrees(port.D, 2000, 100)
    print("Done!")

def test_back_left_spin():

    print(motor.relative_position(port.E))
    motor.run_for_degrees(port.E, -90, 100)
    print(motor.relative_position(port.E))
    time.sleep(1)
    motor.run_for_degrees(port.D, 2000, 100)
    print("Done!")

def test_back_right_spin():

    print(motor.relative_position(port.E))
    motor.run_for_degrees(port.E, 90, 100)
    print(motor.relative_position(port.E))
    time.sleep(1)
    motor.run_for_degrees(port.D, 2000, 100)
    print("Done!")

def test_gears_spin():
    motor.run_to_absolute_position(port.E, 0, 100)
    time.sleep(1)
    print("Press 0, 1 or 2 buttons....waiting for 5 seconds")
    time.sleep(5)
    
    if is_right_button_pressed():
        print("Spinning Front right Gear")
    elif is_left_button_pressed():
        print("Spinning Front left Gear")
        motor.run_to_absolute_position(port.E, 180, 100)
    elif is_right_button_pressed() and is_left_button_pressed():
        print("Spinning Back left Gear")
        motor.run_to_absolute_position(port.E, 180, 100)
    else: 
        print("Spinning Back right Gear")
        motor.run_to_absolute_position(port.E, 270, 100)
    
    motor.run_for_degrees(port.D, 2000, 100)


# Main program that runs all tests
async def mainProgram():

    test_gears_spin()
    # motor.run_to_absolute_position(port.E, 0, 100)
    # time.sleep(1)
    # test_front_right_spin()

    # motor.run_to_absolute_position(port.E, 90, 100)
    # time.sleep(1)
    # test_front_left_spin()

    # motor.run_to_absolute_position(port.E, 180, 100)
    # time.sleep(1)
    # test_back_right_spin()

runloop.run(mainProgram())
