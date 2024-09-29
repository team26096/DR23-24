#!/usr/bin/env python3

# This file contains all utility functions
# TODO: Populate from the other(?)


from initialize import *

class FollowGyroAngleErrorTooFast(Exception):
    """
    Raised when a gyro following robot has been asked to follow
    an angle at an unrealistic speed
    """
    pass

def readAllValues():
    # Read all Files
    pass

#this function moves a motor until it cannot move anymore(stall)
def run_for_motor_stalled(motor, seconds, speed):
    pass

# class EV3DRTires(Wheel):
#     """
#     part number 41897
#     comes in set 45544
#     """
#     def __init__(self):
#         Wheel.__init__(self, 50, 15)

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
    
def follow_until_back_black(tank,lightSensor):
    pass

def follow_until_front_black(tank, lls, rls, range=3):
    pass

def pivot_gyro_turn(left_speed, right_speed, target_angle, 
robot, gyro, bLeftTurn = True):
    pass

def do_calibrate():
    pass

def do_calibrate_back():
    pass

def squareToWhite(speed, left_light, right_light, left_motor, right_motor):
    pass

def squareToBlack(speed, left_light, right_light, left_motor, right_motor):
    pass

def BackwardStall(robot, speed, timeout):
    pass

def p_pivot_gyro_turn(left_speed, right_speed, target_angle,  
robot, gyro, bLeftTurn = True, error_margian = 2, p = 2):
    pass

def myPerfectSquare(left_target_light, right_target_light, left_motor, right_motor, maxSeconds):
  pass

#function to turn and align back sensor to black with forward motion
def backward_turn_until_config_left_white(light_sensor, robot, bLeftTurn=True):
   pass

#function to turn and align back sensor to black with forward motion
def backward_turn_until_config_left_black(light_sensor, robot, bLeftTurn=True):
    pass

def pivot_turn_until_black(left_speed, right_speed, target_light, 
robot, lightSensor):
    pass
