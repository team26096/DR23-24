from hub import port
import motor
import runloop
import motor_pair
from hub import motion_sensor

STEERING_CORRECTION=1

"""
This function turns the gyro in place to specified deci degrees (-1800 to 1800)
This is achieved by setting a steering value of -100 or + 100 based on which
side we want to turn.
"""
async def gyro_in_place_turn_for_decidegrees(decidegrees):
    initial_yaw = motion_sensor.tilt_angles()[0]
    target_yaw = initial_yaw + decidegrees
    if decidegrees > 0:
        steering = 100
        # since steering is 100, the yaw value will increase.
        # so 'move' until the current value is less than target_yaw
        while motion_sensor.tilt_angles()[0] < target_yaw:
            motor_pair.move(motor_pair.PAIR_1,steering)
    elif decidegrees < 0:
        steering = -100
        # since steering is -100, the yaw value will decrease.
        # so 'move' until the current value is greater than target_yaw
        while motion_sensor.tilt_angles()[0] > target_yaw:
            motor_pair.move(motor_pair.PAIR_1,steering)
    else:
        # if decidegrees is 0, there is nothing to do!
        return
    motor_pair.stop(motor_pair.PAIR_1)

def follow_forever():
    return True

def follow_angle_for_distance(angle, distance_to_cover):
    # get initial reading from left motor
    initial_position = abs(motor.relative_position(port.A))
    # print(initial_position)
    distance_covered = 0
    motor_pair.move(motor_pair.PAIR_1,0)
    while (distance_covered < distance_to_cover):
        current_position = abs(motor.relative_position(port.A))
        distance_covered = current_position - initial_position
        current_yaw = motion_sensor.tilt_angles()[0]
        if current_yaw < angle:
            # steer slightly to the right
            motor_pair.move(motor_pair.PAIR_1,STEERING_CORRECTION)
        elif current_yaw > angle:
            motor_pair.move(motor_pair.PAIR_1,-STEERING_CORRECTION)
    motor_pair.stop(motor_pair.PAIR_1)
    print("Total distance travelled = %d", distance_covered)
    print("Gyro follow angle for distance DONE")
    
"""
The function moves the robot for distance requested, and uses the gyro sensor to keep a
straight line. It uses the yaw value to adjust steering (if needed) to correct direction.
Usually a low steering correction is sufficient.
#arg1: distance_to_cover (in degrees - 360 degrees is one full rotation of robot wheel)
#arg2: how much steering correction to do at a time to adjust direction. Note this has to
    be a low value, otherwise the movement could be erratic. recommended values are 1
    to 10.
"""
async def gyro_assisted_move(angle, stop_function, **kwargs):
    stop_function(angle, **kwargs)
     
async def main():
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
    motion_sensor.set_yaw_face(motion_sensor.FRONT)
    motion_sensor.reset_yaw(0)
    # get current yaw_value
    yaw_value = motion_sensor.tilt_angles()[0]
    # Turn the robot 45 degrees (450 decidegrees) in place
    await gyro_in_place_turn_for_decidegrees(450)
    # New yaw value should be
    yaw_value += 450
    await gyro_assisted_move(yaw_value, follow_angle_for_distance, distance_to_cover=720)

runloop.run(main())

# Need the following...
# 1. do not reset yaw inside gyro angle follow. the angle value passed in is the reference yaw. just maintain it.
# have a function pointer as argument, which will be the stop condition. follow_angle_forever, follow_angle_for_time, follow_angle_for_distance,
# follow_angle_until_left_black, fllow_angle_until_left_white, follow_angle_until_right_black, follow_angle_until_right_white
