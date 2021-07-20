"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020
Phase 1 Challenge - Cone Slaloming
"""


#22 was fastest time with max 1 speed

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
from enum import IntEnum
from nptyping import NDArray
from typing import Any, Tuple, List, Optional

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here

MIN_CONTOUR_AREA = 550

# HSV values for color identification.
BLUE = ((100,150,150), (120,255,255),"BLUE") 
RED = ((170,50,50),(10,255,255),"RED")

# Car values.
speed = 0.0
angle = 0.0

time =0.0
# Camera values.
contour_center = None
contour_area = 0

cur_color = None
contour_distance = 0.0

cone_counter = 0
prev_color = None

########################################################################################
# Functions
########################################################################################

class State(IntEnum):
    redReg = 0
    blueReg = 1
    psdBlue = 2
    psdRed = 3
    reverse = 4
    search = 5
    stop = 6

cur_state = State.search

def start():
    """
    This function is run once every time the start button is pressed
    """
    # Global variables.
    global speed
    global angle
    global cur_color
    global time
    # Variables.
    speed = 0.0
    angle = 0.0
    cur_color = None
    
    # Have the car begin at a stop
    rc.drive.set_speed_angle(speed,angle)

    # Print start message
    print(">> Phase 1 Challenge: Cone Slaloming")

def update_contour():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area
    global cur_color
    global contour_distance

    image = rc.camera.get_color_image()
    depth_image = rc.camera.get_depth_image()
    depth_image_adjust = (depth_image - 0.01) % 9999
    depth_image_adjust_blur = cv.GaussianBlur(depth_image_adjust, (11,11), 0)

    contour = None

    if image is None:
        contour_center = None
        contour_area = 0
    else:

        red_contours = rc_utils.find_contours(image, RED[0], RED[1])
        blue_contours = rc_utils.find_contours(image, BLUE[0], BLUE[1])

        largest_red_contour = rc_utils.get_largest_contour(red_contours, MIN_CONTOUR_AREA)
        largest_blue_contour = rc_utils.get_largest_contour(blue_contours, MIN_CONTOUR_AREA)

        if largest_red_contour is not None:
            largest_red_contour_area = rc_utils.get_contour_area(largest_red_contour)
        else:
            largest_red_contour_area = 0

        if largest_blue_contour is not None:
            largest_blue_contour_area = rc_utils.get_contour_area(largest_blue_contour)
        else:
            largest_blue_contour_area = 0
        
        # Select the largest contour
        if largest_red_contour_area > largest_blue_contour_area:
            contour = largest_red_contour
            cur_color = RED[2]

        elif largest_blue_contour_area > largest_red_contour_area:
            contour = largest_blue_contour
            cur_color = BLUE[2]

        if contour is not None:
            # Calculate contour information
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)

            contour_center_x = rc_utils.clamp(contour_center[0], 0, 639)
            contour_center_y = rc_utils.clamp(contour_center[1], 0, 479)

            contour_distance = rc_utils.get_pixel_average_distance(depth_image_adjust_blur, contour_center)

            # Draw contour onto the image
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)

        else:
            contour_center = None
            contour_area = 0
            #contour_distance = 0
        # Display the image to the screen
        rc.display.show_color_image(image)

def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """

    
    update_contour()

    color_img = rc.camera.get_color_image()
    depth_img = rc.camera.get_depth_image()
    
    # Global variables.
    global speed
    global angle
    global cur_state    
    global cur_color
    global contour_distance
    global cone_counter
    global prev_color
    global time

    # Variables.
    speed = 0.0
    angle = 0.0
    time += rc.get_delta_time()
    
    
    color_img_x = rc.camera.get_width()

    if prev_color != cur_color:
        cone_counter += 1
        prev_color = cur_color

    if cone_counter == 12:
        angle = -1
        speed = 0.2
    elif cone_counter == 11 and contour_distance > 130:
        angle = 0.3
        speed = 0.4
    elif cone_counter == 13 and contour_distance > 130 :
        angle = 0.3
        speed = 2

    else:
        if cur_color == 'BLUE':
        
            if contour_center is not None:
                point = rc_utils.remap_range(contour_distance, 10, 300, color_img_x, color_img_x * 3 //4 , True)
                #speed = rc_utils.remap_range(contour_distance,30, 120,0.8,1,True,)
                speed = 1.4
                angle = rc_utils.remap_range(contour_center[1], point, color_img_x // 2 , 0 ,-1 ,True)
                if contour_distance > 145:
                    angle = -0.16
                    speed = 1.4
            else:
                angle = 0.33
                speed = 1.4
               
        
        elif cur_color == 'RED':
            if contour_center is not None:
                point = rc_utils.remap_range(contour_distance, 50, 300, 0, color_img_x // 2, True)
                #speed = rc_utils.remap_range(contour_distance,30, 120,0.7,1,True,)
                speed = 1.4
                angle = rc_utils.remap_range(contour_center[1], point, color_img_x //2 , 0 ,1 ,True)
                if contour_distance > 145:
                    angle = 0.16
                    speed = 1.4
            else:
                angle = -0.33
                speed = 1.4
               
    
    print(angle)
    #print(time)

    rc.drive.set_speed_angle(speed,angle)
########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()