from hub import port
import runloop
import motor_pair
from hub import motion_sensor

'''
arg1: angle (in degrees)
arg2: distance_to_cover (in degrees - 360 degrees is one full rotation of robot wheel)
async def gyro_follow_angle_for_distance(angle, distance_to_cover):
    # divide distance into 10 equal parts
    motion_sensor.reset_yaw(0)
    step = int(distance_to_cover/10)
    distance_covered = int(0)

    left_velocity = 300
    right_velocity = 300
    steering_angle:int = angle
    while (distance_covered < distance_to_cover):
        # Move straight at default velocity for 360 degrees
        await motor_pair.move_for_degrees(motor_pair.PAIR_1, step, steering_angle)
        # Read the sensor values and adjust angles
        steering_angle = (motion_sensor.tilt_angles()[0] * -1) + angle
        distance_covered += step
        print("steering_angle=", steering_angle)
    print("DONE")

async def main():
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)    
    await gyro_follow_angle_for_distance(0, 360)


runloop.run(main())
