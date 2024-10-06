#!/usr/bin/env python3

# This file contains all utility functions
# TODO: Populate from the other(?)

import color, color_sensor, device, motor, motor_pair, orientation, runloop
import hub
import sys

from hub import light_matrix, button, motion_sensor, light, sound, port

WHEEL_CIRCUMFERENCE = 17.584

WHITE_COLOR_INTENSITY_MIN = 92
BLACK_COLOR_INTENSITY_MAX = 18


def follow_for_distance(initial_position=0,
                        distance_to_cover=0):
    current_position = abs(motor.relative_position(port.A))
    distance_covered = current_position - initial_position
    if distance_covered < 0 : distance_covered = distance_covered * -1
    if (distance_covered >= abs(distance_to_cover)):
        return False
    else:
        return True

def get_color_value():
    return color_sensor.reflection(port.C)

def follow_for_color_white():
    return get_color_value() <= WHITE_COLOR_INTENSITY_MIN

def follow_for_color_black():
    return get_color_value() >= BLACK_COLOR_INTENSITY_MAX

async def follow_for_black_followed_by_white(last_color, time_on_black=0.25, time_on_white=0.10):
    if last_color == "black":
        runloop.sleep_ms(time_on_black * 1000)
        if follow_for_color_white():
            runloop.sleep_ms(time_on_white * 1000)
            if follow_for_color_white():
                return True
    return False

async def follow_for_white_followed_by_black(last_color, time_on_white=0.25, time_on_black=0.10):
    if last_color == "white":
        runloop.sleep_ms(time_on_white * 1000)
        if follow_for_color_black():
            runloop.sleep_ms(time_on_black * 1000)
            if follow_for_color_black():
                return True
    return False


def get_yaw_value():
    return motion_sensor.tilt_angles()[0] * -0.1

def degreesForDistance(distance_cm):
    # Add multiplier for gear ratio if needed
    return int((distance_cm/WHEEL_CIRCUMFERENCE) * 360)

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
        # kp value should be +ve for forward movement (positive speed value), and -ve for backward movement (negative speed value)
        motor_pair.move(motor_pair.PAIR_1, int(steering_value), velocity=speed)

    # stop when follow_for condition is met
    motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)

async def pivot_gyro_turn_abs(left_speed=0, right_speed=50, angle=90, stop=False):
    motor_pair.move_tank(motor_pair.PAIR_1, left_speed, right_speed)
    # print("pivot_gyro_turn - " + "target angle=" + str(angle) + "current angle ="+ str(get_yaw_value()))
    wait_for_yaw_abs(angle=angle)
    if stop: motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)

async def turn_left(speed=50, angle=90, stop=True):
    await pivot_gyro_turn_abs(left_speed=0, right_speed=speed, angle=angle, stop=stop)

async def turn_right(speed=-50, angle=90, stop=True):
    await pivot_gyro_turn_abs(left_speed=speed, right_speed=0, angle=angle, stop=stop)

async def test_follow_gyro_angle_for_distance(distance):
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    print("degreesForDistance = {}".format(str(degreesForDistance(distance))))
    await follow_gyro_angle(kp=-1.45*(int(distance/abs(distance))), ki=0, kd=0, speed=250*(int(distance/abs(distance))), target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(distance)))

async def test_turn_left(angle=90):
    await turn_left(speed=350, angle=angle, stop=True)

async def test_turn_right(angle=0):
    await turn_right(speed=350, angle=0, stop=True)

async def test_go_to_black(reverse=False):
    await follow_gyro_angle(kp=-1.45*(1 if reverse else -1), ki=0, kd=0,
                            speed=250*(-1 if reverse else 1), target_angle=0, sleep_time=0, follow_for=follow_for_color_black)

async def test_go_to_white(reverse=False):
    await follow_gyro_angle(kp=-1.45*(1 if reverse else -1), ki=0, kd=0,
                            speed=250*(-1 if reverse else 1), target_angle=0, sleep_time=0, follow_for=follow_for_color_white)

async def test_fake_missions():
    # Go forward 20 cm
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    distance = 20
    await follow_gyro_angle(kp=-1.45*(int(distance/abs(distance))), ki=0, kd=0, speed=250*(int(distance/abs(distance))), target_angle=0, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(distance)))

    # turn left 45 degrees
    await turn_left(speed=100, angle=45, stop=True)

    # go forward 12 cm 
    distance = 12
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45*(int(distance/abs(distance))), ki=0, kd=0, speed=250*(int(distance/abs(distance))), target_angle=-45, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(distance)))

    # go back 12 cm
    distance = -12
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    await follow_gyro_angle(kp=-1.45*(int(distance/abs(distance))), ki=0, kd=0, speed=250*(int(distance/abs(distance))), target_angle=-45, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(distance)))
    

    # Turn right to 45
    await turn_right(speed=150, angle=45, stop=True)

    # Go forward 25 cm
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    distance = 25
    await follow_gyro_angle(kp=-1.45*(int(distance/abs(distance))), ki=0, kd=0, speed=250*(int(distance/abs(distance))), target_angle=45, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(distance)))
    
    # Go back 25 cm
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    distance = -25
    await follow_gyro_angle(kp=-1.45*(int(distance/abs(distance))), ki=0, kd=0, speed=250*(int(distance/abs(distance))), target_angle=45, sleep_time=0, follow_for=follow_for_distance,
                    initial_position=initial_position, distance_to_cover=(degreesForDistance(distance)))


    # Turn right to 180 (facing to the pit)
    await turn_right(speed=150, angle=179, stop=True)

    # Go to pit
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    distance = 20
    await follow_gyro_angle(kp=-1.45*(int(distance/abs(distance))), ki=0, kd=0, speed=250*(int(distance/abs(distance))), target_angle=179, sleep_time=0, follow_for=follow_for_distance,
                            initial_position=initial_position, distance_to_cover=(degreesForDistance(distance)))


async def mainProgram():
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
    print("mainProgram -- START")

    light_matrix.write("0")
    light.color(light.POWER, color.RED)

    # reset yaw to 0
    motion_sensor.set_yaw_face(motion_sensor.TOP)
    motion_sensor.reset_yaw(0)
    await runloop.sleep_ms(1000)

    # i = 0
    # while (hub.motion_sensor.stable() == False):
    #    i = i+1
    #    await runloop.sleep_ms(100)
    #    hub.light_matrix.write(str(i))


    # await test_follow_gyro_angle_for_distance(180)
    # await runloop.sleep_ms(1000)
    # await test_follow_gyro_angle_for_distance(-180)

    # await test_turn_left(90)
    # await runloop.sleep_ms(1500)
    # await test_turn_right(90)

    await test_fake_missions()
    
    # await test_go_to_black()

    # await test_go_to_white()



runloop.run(mainProgram())
