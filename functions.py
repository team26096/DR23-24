#!/usr/bin/env python3

# This file contains all utility functions
# TODO: Populate from the other(?)

# FLL utilities

def gyro_angle_follow(gyro, motor_pair, angle, speed, distance):
    # Follows a gyro angle
    pass

def pivot_gyro_right(gyro, motor_pair, angle, speed):
    # Pivots left at a speed and angle
    pass

def turn_right(speed, angle):
    # Pivots right at a speed and angle
    pass


def robot_runfordegrees(robot, left_speed, right_speed, degrees):
    pass

#this function turns the motor for a certain amount of degrees, 
#normally we can do this with the one line that is in the function 
#but to put it as a thread(that we will use towards the end of the run) 
#we needed to make a new function
def motor_pair_runfordegrees(speed, degrees):
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees=degrees, steering=0, velocity=speed, stop=motor.BRAKE)


def my_follow_for_degrees(tank, degrees, left_motor, right_motor):    
    pass

def follow_until_white(tank, lightSensor):
    pass

def follow_until_black(tank,lightSensor):
    pass

def follow_until_right_white(tank, lightSensor):
    pass

def follow_until_left_white(tank, lightSensor):
    pass

def follow_until_back_white(tank, lightSensor, range=5):
    pass

def follow_until_front_white(tank, lls, rls):
    pass

def follow_until_right_black(tank,lightSensor, range=3):
    pass

def follow_until_left_black(tank,lightSensor):
    pass
    

# pivot_g_right
# pivot_g_left
#spin_g_right
#spin_g_left
