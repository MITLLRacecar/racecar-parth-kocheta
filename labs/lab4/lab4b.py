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
    
    
    _, right_distance = rc_utils.get_lidar_closest_point(scan, (80,100))
    _, left_distance = rc_utils.get_lidar_closest_point(scan, (260,280))
    _, forward_distance = rc_utils.get_lidar_closest_point(scan, (-5,5))
    #print(right_distance)

   # rc.display.show_lidar(scan, radius = 256, highlighted_samples = [right_distance, left_distance])
    diff = right_distance - left_distance
    if abs(diff) > 3:
        angle = rc_utils.remap_range(diff, -30, 30, -1, 1, True)
    else:
        angle = 0
    
    speed = rc_utils.remap_range(forward_distance, 0, 150, 0.3, 1, True)

        
    if cur_state == State.forward:
        print("state forward")
        if (forward_distance < 110 ):
            cur_state = State.turn
    if cur_state == State.turn:
        print('State turn')
        top_right = rc_utils.get_lidar_average_distance(scan,45, 10 )
        top_left =  rc_utils.get_lidar_average_distance(scan,315, 10 )
        if (top_left - top_right > 15):
            #angle = -1
            angle = rc_utils.remap_range(top_left, 0, 100, -0.5, -1, True)
        elif (top_right - top_left > 15):
            angle = rc_utils.remap_range(top_right, 0, 100, 0.5, 1, True)
            #angle = 1
        # else:
        #     angle = rc_utils.remap_range(top_left, 0, 80, 1, -1, True)
            
        if forward_distance > 120:
            cur_state = State.forward
   
    print(left_distance)

    rc.drive.set_speed_angle(speed,angle)    
        #if forward_distance < 20 and left_distance < 30: # sharp turn right
        
    # if left_distance or right_distance is < too_close:
    #     #turn right /left by a certain amount
    # else:
    #     #forward
    #     pass


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
