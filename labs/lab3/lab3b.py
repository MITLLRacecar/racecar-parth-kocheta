"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020
Lab 3B - Depth Camera Cone Parking
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
from typing import Any, Tuple, List, Optional
from nptyping import NDArray
from enum import IntEnum

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################
# Sets up the racecar object
rc = racecar_core.create_racecar()

# >> Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 30

# The HSV range for the color orange, stored as (hsv_min, hsv_max)
ORANGE = ((10, 100, 100), (20, 255, 255))

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour

# Add any global variables here
isParked = False # Set to true once the car has stopped around 30cm in front of the cone

########################################################################################
# Functions
########################################################################################

class State(IntEnum):
    search = 0
    approach = 1

curState = State.search

def update_contour():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area

    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # Find all of the orange contours
        contours = rc_utils.find_contours(image, ORANGE[0], ORANGE[1])

        # Select the largest contour
        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

        if contour is not None:
            # Calculate contour information
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)

            # Draw contour onto the image
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)

        else:
            contour_center = None
            contour_area = 0

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

def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(">> Lab 3B - Depth Camera Cone Parking")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # TODO: Park the car 30 cm away from the closest orange cone.
    global speed
    global angle
    global curState


    # Search for contours in the current color image
    update_contour()

    imgX = rc.camera.get_width()

    if contour_center is not None:
        angle = rc_utils.remap_range(contour_center[1],0,imgX,-1,1)
    
    if contour_center is  None:
        curState == State.search

    if curState == State.search:
        angle = 1
        speed = 1
        if contour_center is not None:
            curState = State.approach


    depth_image = rc.camera.get_depth_image()
    depth_image_adjust = (depth_image - 0.01) % 9999
    depth_image_adjust_blur = cv.GaussianBlur(depth_image_adjust, (11,11), 0)
    

    image = rc.camera.get_color_image()
    mask = get_mask(image, ORANGE[0], ORANGE[1])
    masked_depth_image = cv.bitwise_and(depth_image, depth_image, mask=mask)


    top_left_inclusive = (0, 0)
    bottom_right_exclusive = ((rc.camera.get_height() * 4 // 5) , rc.camera.get_width())


    cropped_image = rc_utils.crop(masked_depth_image, top_left_inclusive, bottom_right_exclusive)
    
    closest_pixel = rc_utils.get_closest_pixel(cropped_image)
    distance = cropped_image[closest_pixel[0], closest_pixel[1]]
    rc.display.show_depth_image(cropped_image, points=[closest_pixel])
    
    if curState == State.approach:
        if distance < 29:
            angle = 0
            speed = rc_utils.remap_range(distance, 0, 30, -1, 0)
          

        elif distance < 30:
            speed = 0
            angle = 0

        elif distance > 30 and distance < 100:
            speed = rc_utils.remap_range(distance, 30, 1000, 0, 1)
                
        elif distance > 100:
            speed = 0.5
   
    rc.drive.set_speed_angle(speed, angle)

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.Y):
        isParked = False
        print("not parke")
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the center and area of the largest contour when B is held down
    if rc.controller.is_down(rc.controller.Button.B):
        if contour_center is None:
            print("No contour found")
        else:
            print("Center:", contour_center, "Area:", contour_area)
    pass


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()