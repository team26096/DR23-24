from hub import port
import motor_pair
from hub import motion_sensor
import time

def gyro_steer(pair:int, angle: int):
    current_yaw = motion_sensor.tilt_angles()[0]
    target_yaw = abs(current_yaw) + abs(angle * 10)
    print("current_yaw = {}".format(current_yaw))

    # Turn to angle
    while(abs(current_yaw) < target_yaw):
        motor_pair.move_for_time(pair, 10, angle * 10)
        current_yaw = motion_sensor.tilt_angles()[0]
        print("current_yaw = {}".format(current_yaw))


def gyro_follow_angle(pair:int, angle: int, move_until=None):
    gyro_steer(pair, angle)

motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
gyro_follow_angle(motor_pair.PAIR_1, 90)
