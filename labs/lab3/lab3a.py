"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020
Lab 3A - Depth Camera Safety Stop
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

safety = True
########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    global safety
    # Have the car begin at a stop
    rc.drive.stop()
    safety = True
    # Print start message
    print(
        ">> Lab 3A - Depth Camera Safety Stop\n"
        "\n"
        "Controls:\n"
        "    Right trigger = accelerate forward\n"
        "    Right bumper = override safety stop\n"
        "    Left trigger = accelerate backward\n"
        "    Left joystick = turn front wheels\n"
        "    A button = print current speed and angle\n"
        "    B button = print the distance at the center of the depth image"
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global safety

    # Use the triggers to control the car's speed
    rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = rt - lt

    # Calculate the distance of the object directly in front of the car
    depth_image = rc.camera.get_depth_image()
    depth_image_adjust = (depth_image - 0.01) % 9999
    depth_image_adjust_blur = cv.GaussianBlur(depth_image_adjust, (11,11), 0)
    center_distance = rc_utils.get_depth_image_center_distance(depth_image_adjust_blur)

    # TODO (warmup): Prevent forward movement if the car is about to hit something.
    # Allow the user to override safety stop by holding the right bumper.
    

    if center_distance <= 30 and safety is True:
        speed = -1
        angle = 0
        print(center_distance)
    elif center_distance <= 120 and safety is True:
        speed = 0
        angle = 0
        print(center_distance)

    if rc.controller.is_down(rc.controller.Button.RB):
        safety = False
    else:
        safety = True

    # Use the left joystick to control the angle of the front wheels
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]



    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the depth image center distance when the B button is held down
    if rc.controller.is_down(rc.controller.Button.B):
        print("Center distance:", center_distance)

    # Display the current depth image
    rc.display.show_depth_image(depth_image)

    # TODO (stretch goal): Prevent forward movement if the car is about to drive off a
    # ledge.  ONLY TEST THIS IN THE SIMULATION, DO NOT TEST THIS WITH A REAL CAR.

    # top_left_inclusive = (rc.camera.get_height() * 6 // 7, 0)
    # bottom_right_exclusive = (rc.camera.get_height(), rc.camera.get_width())

    # cropped_image = rc_utils.crop(depth_image, top_left_inclusive, bottom_right_exclusive)
    # image_adjust = (cropped_image - 0.01) % 9999
    # image_adjust_blur = cv.GaussianBlur(image_adjust, (21,21), 0)
    
    # minVal, farthest_pixel, minLoc, farthest_loc = cv.minMaxLoc(image_adjust_blur)

    # adj_loc = farthest_loc[::-1]

    # rc_utils.draw_circle(image_adjust_blur, adj_loc)
    # rc.display.show_depth_image(image_adjust_blur)

    # print(farthest_pixel)
    
    # if farthest_pixel < 9000 and farthest_pixel > 230 and safety is True:
    #     print("found edge")
    #     speed = 0
    #     angle = 0

    # TODO (stretch goal): Tune safety stop so that the car is still able to drive up
    # and down gentle ramps.
    # Hint: You may need to check distance at multiple points.

    # top_left_inclusive1 = (0, 0)
    # bottom_right_exclusive1 = (rc.camera.get_height() * 2 // 5, rc.camera.get_width())

    # cropped_image1 = rc_utils.crop(depth_image, top_left_inclusive1, bottom_right_exclusive1)
    # image_adjust1 = (cropped_image1 - 0.01) % 10000
    # image_adjust_blur1 = cv.GaussianBlur(image_adjust1, (21,21), 0)
    
    # minVal1, farthest_pixel1, minLoc1, farthest_loc1 = cv.minMaxLoc(image_adjust_blur1)
    # adj_loc1 = farthest_loc1[::-1]

    # rc_utils.draw_circle(image_adjust_blur1, adj_loc1)

    # if farthest_pixel1 > 200 and center_distance != 0:
    #     safety = False

    rc.drive.set_speed_angle(speed, angle)

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################
# def get_farthest_pixel(
#     depth_image: NDArray[(Any, Any), np.float32],
#     kernel_size: int = 5) -> Tuple[int, int]:
    
#     depth_image = (depth_image - 0.01) % 10000
#     image_blur = cv.GaussianBlur(depth_image, (kernel_size,kernel_size), 0)
#     minVal, maxVal, minLoc, maxLoc = cv.minMaxLoc(image_blur)
#     rtrnVal = maxLoc[::-1]
#     return (rtrnVal)

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()