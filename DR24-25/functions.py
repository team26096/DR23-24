#!/usr/bin/env python3

# This file contains all utility functions
# TODO: Populate from the other(?)

import color, color_sensor, device, motor, motor_pair, orientation, runloop
import hub

from hub import light_matrix, button, motion_sensor, light, sound, port

WHEEL_CIRCUMFERENCE = 17.584

def follow_for_distance(initial_position=0,
                        distance_to_cover=0):
    current_position = abs(motor.relative_position(port.A))
    distance_covered = current_position - initial_position
    if distance_covered < 0 : distance_covered = distance_covered * -1
    if (distance_covered >= distance_to_cover):
        return False
    else:
        return True


def get_yaw_value():
    return motion_sensor.tilt_angles()[0] * -0.1

def degreesForDistance(distance_cm):
    # Add multiplier for gear ratio if needed
    return int((distance_cm/WHEEL_CIRCUMFERENCE) * 360)


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


async def test_follow_gyro_angle_for_distance(distance):
    motor.reset_relative_position(port.A, 0)
    initial_position = abs(motor.relative_position(port.A))
    print("degreesForDistance = {}"+ str(degreesForDistance(distance)))
    await follow_gyro_angle(kp=0, ki=0, kd=0, speed=250, target_angle=0, sleep_time=0, follow_for=follow_for_distance,
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
    #     i = i+1
    #     await runloop.sleep_ms(100)
    #     hub.light_matrix.write(str(i))


    await test_follow_gyro_angle_for_distance(60)

runloop.run(mainProgram())

