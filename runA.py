from hub import motor_pair, color_sensor, motor, runloop, orientation, force_sensor, distance_sensor, device, color

def gyro_follow_angle_for_distance(angle, distance):
    # divide distance into 10 equal parts
    step = distance/10
    left_velocity = 1000
    right_velocity = 1000
    while (step <= distance):
        # Move straight at default velocity for 360 degrees
        await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1, step, left_velocity, right_velocity)
        # Read the sensor values and adjust angles
        current_sensor_values = motion_sensor.tilt_angles()
        # Say yaw is +10, this means the robot has deviated 10 degrees. To get it back on straight line,
        # we will have to double the skew
        adjustment = motion_sensor.tilt_angles()[0]
        left_velocity+=adjustment
        right_velocity-=adjustment
    pass

aync def main():
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
    await gyro_follow_angle_for_distance(0,300)

runloop.run(main())

