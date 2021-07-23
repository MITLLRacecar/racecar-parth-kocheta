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

    forward_distance = rc_utils.get_lidar_average_distance(scan, 0 ,10)
    top_right = rc_utils.get_lidar_average_distance(scan,45, 10 )
    top_left =  rc_utils.get_lidar_average_distance(scan,315, 10 )
    diff_top =  top_right -top_left 
    #print(right_distance)

   # rc.display.show_lidar(scan, radius = 256, highlighted_samples = [right_distance, left_distance])
    diff = right_distance - left_distance
    
    #angle = rc_utils.remap_range(diff, -20, 20, -0.4, 0.4, True)
    
    #angle = dif * 0.05
    
    speed = rc_utils.remap_range(forward_distance, 0, 250, 1, 2, True)
    
    #angle  = rc_utils.clamp (diff_top * 0.008,-0.3,0.3)
    angle  = rc_utils.clamp (diff_top * 0.03,-1.5,1.5)

    """
    if cur_state == State.forward:
         if (forward_distance < 125 ):
             cur_state = State.turn
    if cur_state == State.turn:
         top_right = rc_utils.get_lidar_average_distance(scan,45, 5 )
         top_left =  rc_utils.get_lidar_average_distance(scan,315, 5 )
         if (top_left - top_right > 30):    
              angle = rc_utils.remap_range(top_left, 0, 100, 0, -1.7, True)
         elif (top_right -  top_left > 30):
              angle = rc_utils.remap_range(top_right, 0, 100, 0, 1.7, True)
         else:
             cur_state = State.forward
    """
    print (forward_distance)
    #print(cur_state)

    rc.drive.set_speed_angle(speed,angle)    






        #   angle = rc_utils.remap_range(top_left, 0, 80, 1, -1, True)
            
        #if forward_distance > 115:
        #    cur_state = State.forward
   
    #print(left_distance)

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