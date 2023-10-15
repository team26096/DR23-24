from hub import port
import motor
import runloop
import motor_pair
from hub import motion_sensor
"""
This function turns the gyro in place to specified deci degrees (-1800 to 1800)
This is achieved by setting a steering value of -100 or + 100 based on which
side we want to turn.
"""
async def gyro_in_place_turn_for_decidegrees(decidegrees):
    # set yaw face and reset yaw
    motion_sensor.set_yaw_face(motion_sensor.FRONT)
    motion_sensor.reset_yaw(0)
    initial_yaw = 0
    if decidegrees > 0:
        steering = 100
    else if decidegrees < 0:
        steering = -100
    else:
        # if decidegrees is 0, there is nothing to do!
        pass
    while abs(motion_sensor.tilt_angles()[0]) < angle:
        motor_pair.move(motor_pair.PAIR_1,steering)
    motor_pair.stop()
    # reset the yaw again, so where we are facing is the new 0!
    motion_sensor.reset_yaw(0)


"""
The function moves the robot for distance requested, and uses the gyro sensor to keep a
straight line. It uses the yaw value to adjust steering (if needed) to correct direction.
Usually a low steering correction is sufficient.
#arg1: distance_to_cover (in degrees - 360 degrees is one full rotation of robot wheel)
#arg2: how much steering correction to do at a time to adjust direction. Note this has to
       be a low value, otherwise the movement could be erratic. recommended values are 1
       to 10. 
"""
async def gyro_assisted_move_for_distance(distance_to_cover, steering_correction):
    # set yaw face and reset yaw
    motion_sensor.set_yaw_face(motion_sensor.FRONT)
    motion_sensor.reset_yaw(0)
    # get initial reading from left motor
    initial_position = abs(motor.relative_position(port.A))
    print(initial_position)
    distance_covered = 0;
    motor_pair.move(motor_pair.PAIR_1,0)

    while (distance_covered < distance_to_cover):
        current_position = abs(motor.relative_position(port.A))
        distance_covered = current_position - initial_position
        print(current_position)
        current_yaw = motion_sensor.tilt_angles()[0]
        if current_yaw < 0:
            # steer slightly to the right
            motor_pair.move(motor_pair.PAIR_1,steering_correction)
        else if current_yaw > 0:
            motor_pair.move(motor_pair.PAIR_1,-steering_correction)
        #await runloop.sleep_ms(20)

    motor_pair.stop(motor_pair.PAIR_1)
    print("Total distance travelled = %d", distance_covered)
    print("Gyro move for distance DONE")

async def do_3d_cinema(angle, distance_to_cover):
    # Move straight at default velocity for 360 degrees
    motor_pair.move_for_degrees(motor_pair.PAIR_1, distance_to_cover, angle)
    print(distance_to_cover)
    print("plain move_for_degrees DONE")

async def main():
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
    # Turn the robot 45 degrees (450 decidegrees) in place
    await gyro_in_pace_turn_for_decidegrees(450)
    await gyro_follow_angle_for_distance(720,1)
    #await do_3d_cinema(0, 360)

runloop.run(main())
