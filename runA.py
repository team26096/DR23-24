from hub import port
import motor
import runloop
import motor_pair
from hub import motion_sensor

"""
#arg1: angle (in degrees)
#arg2: distance_to_cover (in degrees - 360 degrees is one full rotation of robot wheel)
"""
async def gyro_follow_angle_for_distance(angle, distance_to_cover):
    # set yaw face and reset yaw
    motion_sensor.set_yaw_face(motion_sensor.FRONT)
    motion_sensor.reset_yaw(0)
    # get initial reading from left motor
    initial_position = abs(motor.relative_position(port.A))
    print(initial_position)
    distance_covered = 0;
    # Move motor pair at constant velocity util further command is given at angle provided 
    # angle can be -100-100. A value of -100 will give 0 power to left motor and 100 percent
    # power to right motor.
    motor_pair.move(motor_pair.PAIR_1,angle)

    while (distance_covered < distance_to_cover):
        current_position = abs(motor.relative_position(port.A))
        distance_covered = current_position - initial_position
        print(current_position)
        #await runloop.sleep_ms(20)

    motor_pair.stop(motor_pair.PAIR_1)
    print("Total distance travelled = %d", distance_covered)
    print("Gyro angle follow DONE")

async def do_3d_cinema(angle, distance_to_cover):
    # Move straight at default velocity for 360 degrees
    motor_pair.move_for_degrees(motor_pair.PAIR_1, distance_to_cover, angle)
    print(distance_to_cover)
    print("plain move_for_degrees DONE")

async def main():
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)
    await gyro_follow_angle_for_distance(-10, 720)
    #await do_3d_cinema(0, 360)

runloop.run(main())
