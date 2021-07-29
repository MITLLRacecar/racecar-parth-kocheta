"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 5 - AR Markers
"""

########################################################################################
# Imports
########################################################################################

#from labs.lab4.lab4b import DRIVE_SPEED, LEFT_WINDOW
#from labs.lab4.lab4b import FRONT_WINDOW
import sys
import cv2 as cv
import numpy as np
from simple_pid import PID
sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
from racecar_utils import ARMarker
from enum import Enum, IntEnum


########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here
global front_dist, finishedTurning
potential_colors = [
    ((90, 50, 50), (120, 255, 255), "BLUE"),
    ((40, 50, 50), (80, 255, 255), "GREEN"),
    ((170, 50, 50), (10, 255, 255), "RED")
]

MIN_CONTOUR_AREA = 30

### LINE FOLLOWING ###
BLUE = ((88,245,199), (108,255,255))
RED = ((0, 50, 50), (20, 255, 255))#(175, 50, 50), (10, 255, 255))#
GREEN = ((40, 60, 60), (90, 255, 255)) #(40, 50, 50), (80, 255, 255)
ORANGE = ((3, 174, 78), (23, 194, 158), "ORANGE")
WHITE = ((90, 20, 200), (115, 60, 255))
CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))

speed = 0
angle = 0

contour_area = 0
contour_center =0 
contour_center = 0
straight = False
########################################################################################
# Functions
########################################################################################

class Orientation(Enum):
    UP = 0
    LEFT = 1
    DOWN = 2
    RIGHT = 3



def start():
    """
    This function is run once every time the start button is pressed
    """
    rc.drive.stop()



def update():
    global contour_center
    global contour_area
    
    global contour_distance
    #update_contour()
    
    #color_image =rc.camera.get_color_image()
    #orangeplate()
    #print(lower.size)
    
    #print()
    #print( (rc.camera.get_width() * 3 // 4))
    #
    #color_image =rc.camera.get_color_image()
    #listmid = color_image[300][rc.camera.get_width() // 2]

    #print(color_image[300][rc.camera.get_width() // 4] )
    update_contour()
    print (rc.camera.get_width() // 2)
    if contour_center_x < rc.camera.get_width() // 2:
        angle = -0.5
    else: 
        angle = -1
    speed = 0.5
    print(contour_center_x)
    rc.drive.set_speed_angle(speed, angle)



def orangeplate():
    global speed, angle 
    depth_image = rc.camera.get_depth_image()
    color_image =rc.camera.get_color_image()
    new_img = np.array(color_image)
    hsv = cv.cvtColor(new_img, cv.COLOR_BGR2HSV)
    rc.display.show_color_image(hsv)
    scan = rc.lidar.get_samples()
    pid = PID(0.01, 0.1, 0.1, setpoint=0)
    pid.output_limits = (-1.5, 1.5)

    top_right = rc_utils.get_lidar_average_distance(scan,42, 10 )
    top_left =  rc_utils.get_lidar_average_distance(scan,318, 10 )
  
    diff_top2 = top_left - top_right
    control = pid(diff_top2)

    angle  = rc_utils.clamp (control,-1.4,1.4)

    left_dist = depth_image[300][rc.camera.get_width() // 4] 
    middle_dist  = depth_image[300][rc.camera.get_width() // 2 ]
    right_dist = depth_image[300 ][ rc.camera.get_width() * 3 //4]


    listleft = tuple(hsv[300][rc.camera.get_width() // 4])
    listmid = tuple(hsv[300][rc.camera.get_width() // 2])
    listright = tuple(hsv[300][rc.camera.get_width() *3 // 4])


    lower = (0, 77, 94)
    upper = (33, 164, 255)
    
    print(listleft)

    if cv.inRange(listleft, lower, upper):
        print("left true")
    if cv.inRange(listmid, lower, upper):
        print("middle true")
    if  cv.inRange(listright, lower, upper):
        print("right middle")

    if right_dist < 50 and middle_dist > 50:
        angle = -0.3
    elif left_dist < 50 and middle_dist > 50 :
        angle = 0.3     
    elif middle_dist < 50 and left_dist > 50 :
        angle = -0.3
    elif middle_dist < 50 and right_dist > 50:
        angle = 0.3    
    
    print("right distance" + str(right_dist))
    print("left distance" + str(left_dist))
    print("middle distance" + str(middle_dist)+ "\n")
    speed = 0.8


def update_contour():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area
    global cur_color
    global contour_distance
    global contour_center_x
    image = rc.camera.get_color_image()
    depth_image = rc.camera.get_depth_image()
    depth_image_adjust = (depth_image - 0.01) % 9999
    depth_image_adjust_blur = cv.GaussianBlur(depth_image_adjust, (11,11), 0)

    contour = None

    if image is None:
        contour_center = None
        contour_area = 0
    else:

        orange_contours = rc_utils.find_contours(image, ORANGE[0], ORANGE[1])

        largest_orange_contour = rc_utils.get_largest_contour(orange_contours, MIN_CONTOUR_AREA)

        if largest_orange_contour is not None:
            largest_orange_contour_area = rc_utils.get_contour_area(largest_orange_contour)
        else:
            largest_orange_contour_area = 0
        
        contour = largest_orange_contour
        
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

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
