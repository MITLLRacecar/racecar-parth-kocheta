"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 5 - AR Markers
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

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(">> Lab 5 - AR Markers")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    color_image = rc.camera.get_color_image()
    markers = rc_utils.get_ar_markers(color_image)
    scan = rc.lidar.get_samples()
    
    
    _, right_distance = rc_utils.get_lidar_closest_point(scan, (80,100))
    _, left_distance = rc_utils.get_lidar_closest_point(scan, (260,280))
    _, forward_distance = rc_utils.get_lidar_closest_point(scan, (-10,10))
    #print(right_distance)

   # rc.display.show_lidar(scan, radius = 256, highlighted_samples = [right_distance, left_distance])
    diff = right_distance - left_distance
    
    angle = rc_utils.remap_range(diff, -20, 20, -0.4, 0.4, True)
    
    
    speed = rc_utils.remap_range(forward_distance, 0, 250, 0.5, 1.5, True)

        
    if cur_state == State.forward:
        if (forward_distance < 130 ):
            cur_state = State.turn
    if cur_state == State.turn:
        top_right = rc_utils.get_lidar_average_distance(scan,45, 10 )
        top_left =  rc_utils.get_lidar_average_distance(scan,315, 10 )
        if (top_left - top_right > 30):
            angle = rc_utils.remap_range(top_left, 0, 100, 0, -1, True)
        elif (top_right - top_left > 30):
            angle = rc_utils.remap_range(top_right, 0, 100, 0, 1, True)
        else:
            cur_state = State.forward
    #AR Markers
    
    # TODO: Turn left if we see a marker with ID 0 and right for ID 1

    # TODO: If we see a marker with ID 199, turn left if the marker faces left and right
    # if the marker faces right
    if markers is not None: #if a marker is in view
        ori = rc_utils.get_orientation()

    # TODO: If we see a marker with ID 2, follow the color line which matches the color
    # border surrounding the marker (either blue or red). If neither color is found but
    # we see a green line, follow that instead.


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
