"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020
Lab 4B - LIDAR Wall Following
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
from simple_pid import PID

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
from enum import IntEnum

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here
class State(IntEnum):
    forward = 0
    turn = 1

cur_state = State.forward
speed = 0.0
angle = 0.0
########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()
    global cur_state
    global speed
    global angle
    # Print start message
    
    print(">> Lab 4B - LIDAR Wall Following")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global cur_state
    global speed
    global angle 
    # TODO: Follow the wall to the right of the car without hitting anything.
    scan = rc.lidar.get_samples()
    pid = PID(0.015, 0.1, 0.05, setpoint=0)
    pid.output_limits = (-1.7, 1.7)
    

    forward_distance = rc_utils.get_lidar_average_distance(scan, 0 ,10)
    top_right = rc_utils.get_lidar_average_distance(scan,42, 10 )
    top_left =  rc_utils.get_lidar_average_distance(scan,318, 10 )

    diff_top =  top_right -top_left 
    diff_top2 = top_left - top_right
    control = pid(diff_top2)

    speed = rc_utils.remap_range(forward_distance, 0, 250, 1, 2.1, True)
    
    angle  = rc_utils.clamp (control,-1.55,1.55)
    
    #angle  = rc_utils.clamp (diff_top * 0.02,-1.6,1.6)

    if (abs(angle) < 0.05):
        angle = 0 

    rc.drive.set_speed_angle(speed,angle)    





########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()