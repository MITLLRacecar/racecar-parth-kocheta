"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 3C - Depth Camera Wall Parking
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
from enum import IntEnum

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

class State(IntEnum):
    aligningAngle = 0
    aligningDistance = 1
    aligned = 2
    
curr_state = State.aligningAngle
speed = 0.0  
angle = 0.0  

def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(">> Lab 3C - Depth Camera Wall Parking")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global curState
    global speed
    global angle
    global curr_state
    
    depth_image = rc.camera.get_depth_image()
    depth_image_adjust = (depth_image - 0.01) % 9999
    depth_image_adjust_blur = cv.GaussianBlur(depth_image_adjust, (11,11), 0)
    imgY = rc.camera.get_height()
    imgX = rc.camera.get_width()
    
    left_distance = rc_utils.get_depth_image_left_distance(depth_image, imgX, imgY)
    right_distance = rc_utils.get_depth_image_right_distance(depth_image, imgX, imgY)
    center_distance = rc_utils.get_depth_image_center_distance(depth_image)

    # angleDiff = left_distance - right_distance
    print(left_distance)
    print(right_distance)

    rc.display.show_depth_image(depth_image, points=[(imgY // 2, imgX  // 4 ), (imgY // 2, imgX * 3 //4)])
    # rc.display.show_depth_image(depth_image, points=[(depth_image.shape[1] *1 // 3, depth_image.shape[0] // 4), (depth_image.shape[1] *1 // 3, depth_image.shape[0] * 3 //4)])
    
    if center_distance > 18 and center_distance < 22:
        curr_state = State.aligned
    else:
        curr_state = State.aligningAngle

    if curr_state == State.aligningAngle:
        if left_distance >= 23 and right_distance >= 23:
            if left_distance > right_distance:
                print("turning right")
                speed = 0.2
                angle = 0.8
            elif left_distance < right_distance:
                print("turning left")
                speed = 0.2
                angle = -0.8
            elif abs(left_distance - right_distance) <= 3:
                print("state switched")
                curr_state = State.aligningDistance

        elif left_distance < 21 or right_distance < 21:
            if left_distance > right_distance:
                print("turning left")
                speed = -0.2
                angle = -0.8
            elif left_distance < right_distance:
                print("turning right")
                speed = -0.2
                angle = 0.8
            elif abs(left_distance - right_distance) <= 3:
                print("state switched")
                # (left_distance < (right_distance + 0.5)) or (left_distance < (right_distance - 0.5)):
                curr_state = State.aligningDistance 
        else:
            curr_state = State.aligningDistance
                
    elif curr_state == State.aligningDistance:
        if center_distance < 19:
            angle = 0
            speed = rc_utils.remap_range(center_distance, 0, 20, -0.7, 0)
            print("backing")

        elif center_distance <= 20:
            speed = 0
            angle = 0
            curr_state = State.aligned
            
        elif center_distance > 30 and center_distance < 100:
            speed = rc_utils.remap_range(center_distance, 22, 100, 0, 0.5)
            angle = 0
        elif center_distance > 100:
            speed = 0.5

    elif curr_state == State.aligned:
        speed = 0
        angle = 0   
  

       


    # TODO: Park the car 20 cm away from the closest wall with the car directly facing
    # the wall

    rc.drive.set_speed_angle(speed, angle)
    pass


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
