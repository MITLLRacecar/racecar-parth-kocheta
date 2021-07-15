"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020
Phase 1 Challenge - Cone Slaloming
"""

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

MIN_CONTOUR_AREA = 500

# HSV values for color identification.
BLUE = ((100,150,150), (120,255,255),"BLUE") 
RED = ((170,50,50),(10,255,255),"RED")

# Car values.
speed = 0.0
angle = 0.0

# Camera values.
contour_center = None
contour_area = 0

cur_color = None

# Color distances.
red_distance = 0.0
blue_distance = 0.0
contour_distance = 0.0

# Color contour centers.
red_center = None
blue_center = None

# Color contour areas.
red_contour_area = 0.0
blue_contour_area = 0.0

counter = 0
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
    global counter

    # Variables.
    speed = 0.0
    angle = 0.0
    cur_color = None
    counter = 0
    
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

def get_mask(
    image: NDArray[(Any, Any, 3), np.uint8],
    hsv_lower: Tuple[int, int, int],
    hsv_upper: Tuple[int, int, int]
) -> NDArray[Any, Any]:
    """
    Returns a mask containing all of the areas of image which were between hsv_lower and hsv_upper.
    Args:
        image: The image (stored in BGR) from which to create a mask.
        hsv_lower: The lower bound of HSV values to include in the mask.
        hsv_upper: The upper bound of HSV values to include in the mask.
    """
    # Convert hsv_lower and hsv_upper to numpy arrays so they can be used by OpenCV
    hsv_lower = np.array(hsv_lower)
    hsv_upper = np.array(hsv_upper)

    # TODO: Use the cv.cvtColor function to switch our BGR colors to HSV colors
    image = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    # TODO: Use the cv.inRange function to highlight areas in the correct range
    mask = cv.inRange(image, hsv_lower, hsv_upper)

    return mask

def getCones():
    
    global cur_color
    global red_distance
    global blue_distance
    global red_center
    global blue_center
    global red_contour_area
    global blue_contour_area
    
    color_img = rc.camera.get_color_image()
    depth_img = rc.camera.get_depth_image()

    depth_image_adjust = (depth_img - 0.01) % 9999
    depth_image_adjust_blur = cv.GaussianBlur(depth_image_adjust, (11,11), 0)

    red_contours = rc_utils.find_contours(color_img, RED[0], RED[1])
    red_contour = rc_utils.get_largest_contour(red_contours, 100)
    
    blue_contours = rc_utils.find_contours(color_img, BLUE[0], BLUE[1])
    blue_contour = rc_utils.get_largest_contour(blue_contours, 100)  

    red_distance = 0
    blue_distance = 0

    red_contour_area = 0 
    blue_contour_area = 0
    
    mask_red = get_mask(color_img, RED[0], RED[1])
    masked_depth_image_red = cv.bitwise_and(depth_image_adjust_blur, depth_image_adjust_blur, mask=mask_red)

    mask_blue = get_mask(color_img, BLUE[0], BLUE[1])
    masked_depth_image_blue = cv.bitwise_and(depth_image_adjust_blur, depth_image_adjust_blur, mask=mask_blue)


    if red_contour is not None:
        red_contour_area = rc_utils.get_contour_area(red_contour)
        red_center = rc_utils.get_contour_center(red_contour)
        red_center_y = rc_utils.clamp(red_center[1],0,479)
        red_center_x = rc_utils.clamp(red_center[0],0,639)
        #red_distance = rc_utils.get_closest_pixel(masked_depth_image_red)
        red_distance = depth_image_adjust_blur[red_center_y][red_center_x]
        #rc_utils.draw_circle(color_img, red_center, rc_utils.ColorBGR.yellow.value)
        rc.display.show_depth_image(color_img, points=[red_center])
        #print("access red")

    if blue_contour is not None:
        blue_contour_area = rc_utils.get_contour_area(blue_contour)
        blue_center = rc_utils.get_contour_center(blue_contour)
        blue_center_y = rc_utils.clamp(blue_center[1],0,479)
        blue_center_x = rc_utils.clamp(blue_center[0],0,639)
        # blue_distance = rc_utils.get_closest_pixel(masked_depth_image_blue)
        blue_distance = depth_image_adjust_blur[blue_center_y][blue_center_x]
        # rc_utils.draw_circle(color_img, blue_center, rc_utils.ColorBGR.yellow.value)
        rc.display.show_depth_image(color_img, points=[blue_center])
        #print("access blue")

    """  
    if (red_contour_area is not None) and (red_contour_area > blue_contour_area) :
        
        rc_utils.draw_contour(color_img, red_contour)
        rc_utils.draw_circle(color_img, red_center)       
    if (blue_contour_area is not None) and (blue_contour_area > red_contour_area):
        
        rc_utils.draw_contour(color_img, blue_contour)
        rc_utils.draw_circle(color_img, blue_contour) 
    """
   # print("area red:" + str(red_contour_area))
  #  print("area blue:" + str(blue_contour_area))
    
def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # getCones()
    update_contour()

    color_img = rc.camera.get_color_image()
    depth_img = rc.camera.get_depth_image()
    
    # Global variables.
    global speed
    global angle
    global cur_state    
    # global red_distance
    # global blue_distance
    global cur_color
    # global red_contour_area
    # global blue_contour_area
    global contour_distance
    global counter

    # Variables.
    speed = 0.0
    angle = 0.0

    color_img_x = rc.camera.get_width()
    color_img_y = rc.camera.get_height()
    
    #print(cur_state)
    #print(cur_color)
    
    # if red_contour_area > blue_contour_area:
    #     cur_color = 'RED'
    #     cur_state = State.redReg
    #     #cur_state = State.psdBlue
    #     #print ("swtiched to red")
    # elif blue_contour_area > red_contour_area: 
    #     #print (cur_state)
    #     cur_state = State.blueReg
    #     #cur_state = State.psdRed
    #     cur_color = 'BLUE'    
        
    # if blue_distance == 0 and red_distance == 0:
    #     if cur_color == "BLUE":
    #         cur_state = State.psdBlue
    #     elif cur_color == "RED":
    #         cur_state = State.psdRed

    # if  cur_state == State.psdRed :
    #     angle = -0.05
    #     speed = 1
    #     if blue_contour_area  > 900:
    #         cur_state = State.blueReg  


    # elif cur_state == State.psdBlue:
    #     angle = 0.05
    #     speed = 1
    #     if red_contour_area > 900:
    #         cur_state = State.redReg

    
    if cur_color == 'BLUE':
        if contour_center is not None:
            counter = 0
            point = rc_utils.remap_range(contour_distance, 10, 300, color_img_x, color_img_x * 3 //4 , True)
            #point = rc_utils.remap_range(blue_distance, 30, 90, color_img_x, color_img_x , True)

            #speed = 1
            speed = 1
            angle = rc_utils.remap_range(contour_center[1], point, color_img_x // 2 , 0 ,-1 ,True)
            if contour_distance > 150:
                   angle = 0
        else:
            print("lost the blue")
            angle = -1
            speed = 0.5
        """
        speed = contour_distance * 0.01
        speed = rc_utils.clamp(speed, 0,1)
        if contour_center is not None:
            angle = 0.005 * contour_center[1]
            angle = rc_utils.clamp(angle, 0,-0.5)
        else:
            angle = 1
            speed = 0.5
            print("Finding red")
            # counter += rc.get_delta_time()
            # if counter > 1:
            #     angle = 0.8
            #     speed = 0.5
        # if (contour_distance > 700):
        #     angle = 0.4    
        """
        #if (contour_distance)
    elif cur_color == 'RED':
        if contour_center is not None:
            counter = 0
            point = rc_utils.remap_range(contour_distance, 50, 300, 0, color_img_x //2, True)
            #speed = 1
            speed = rc_utils.remap_range(contour_distance,30, 120,0.3,1,True,)
            #speed = contour_distance * 0.01
            #speed = rc_utils.clamp(speed, 0,1)
            angle = rc_utils.remap_range(contour_center[1], point, color_img_x //2 , 0 ,1 ,True)
            if contour_distance > 180:
                   angle = 0
        else:
            print("lost the red")

            angle = 1
            speed = 0.5    
        """
        if contour_center is not None:
            angle = 0.005 * contour_center[1]
            angle = rc_utils.clamp(angle, 0,0.5)
        else:
            print("Finding blue")
            angle = -1
            speed = 0.5
            # counter += rc.get_delta_time()
            # if counter > 1:
            #     angle = -0.8
            #     speed = 0.5
            """
                

        
    
    print (contour_distance)
    rc.drive.set_speed_angle(speed,angle)
########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()