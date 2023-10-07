from hub import port
import runloop
import motor_pair
from hub import motion_sensor


async def gyro_follow_angle_for_distance(angle, distance):
    # divide distance into 10 equal parts
    motion_sensor.reset_yaw(0)
    step = int(distance/10)

    left_velocity = 300
    right_velocity = 300
    steering_angle:int = angle
    while (step <= distance):
        
        # Move straight at default velocity for 360 degrees
        motor_pair.move_for_degrees(motor_pair.PAIR_1, step, steering_angle)
        # Read the sensor values and adjust angles
        # current_sensor_values = motion_sensor.tilt_angles()
        # Say yaw is +10, this means the robot has deviated 10 degrees. To get it back on straight line,
        # we will have to double the skew
        steering_angle = (motion_sensor.tilt_angles()[0] * -1) + angle
        step += int(distance/10)
        
        print("steering_angle=", steering_angle)
    print("DONE")

async def main():
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)    
    await gyro_follow_angle_for_distance(0, 360)


runloop.run(main())
