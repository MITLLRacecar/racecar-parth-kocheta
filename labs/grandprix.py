"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Grand Prix 2021
"""

########################################################################################
# Imports
########################################################################################

#from labs.lab4.lab4b import DRIVE_SPEED, LEFT_WINDOW
#from labs.lab4.lab4b import FRONT_WINDOW
import sys
import cv2 as cv
import numpy as np
sys.path.insert(0, "../../library")

import racecar_core
import racecar_utils as rc_utils
from racecar_utils import ARMarker
from enum import IntEnum


########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

### LINE FOLLOWING ###
BLUE_CONE = ((100,150,150), (120,255,255),"BLUE") 
RED_CONE = ((170, 150, 150), (10, 255, 255), "RED")
BLUE = ((88, 245, 199), (108, 255, 255), "BLUE")
RED = ((0, 50, 50), (20, 255, 255), "RED")
ORANGE = ((7, 172,  78), (27, 192, 158), "ORANGE")
GREEN = ((40, 60, 60), (90, 255, 255), "GREEN") 
WHITE = ((90, 20, 200), (115, 60, 255), "WHITE")
ORANGEMARKER = ((7, 172, 78), (27, 192, 158), "ORANGE")#
ORANGELINE = ((5, 245, 215), (25, 255, 255), "ORANGE")
PURPLEMARKER =  ((121, 192, 109), (141, 212, 189), "PURPLE")#
PURPLELINE = ((125, 245, 215), (145, 255, 255), "PURPLE")


### LIDAR WINDOW ###
FRONT_WINDOW = (-10, 10)
RIGHT_FRONT_WINDOW = (40, 50)
LEFT_FRONT_WINDOW = (310, 320)

LEFT_WINDOW_LIDAR = (-135, -45)
RIGHT_WINDOW_LIDAR = (45, 135)
FRONT_WINDOW_LIDAR = (-10, 10)
BACK_WINDOW_LIDAR = (170, 190)


### CROPPED IMAGE ###
CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))


MIN_CONTOUR_AREA = 30

potential_colors = [BLUE, RED, GREEN]
potential_colors_markers = [PURPLEMARKER, ORANGEMARKER]
potential_colors_lines = [PURPLELINE, ORANGELINE]

speed = 0
angle = 0

wall_follow_end = False
final_jump_end = False
elevator_end = False
canyon_end = False

rightLine = 0

time = 0.0
# Camera values.
contour_center = None
contour_area = 0

cur_color = None
contour_distance = 0.0

cone_counter = 0
prev_color = None

MIN_CONTOUR_AREA = 650

currentColor = None

counter = 0

finalJump = False

arColor = None
########################################################################################
# Functions
########################################################################################

### State Machine ###
class State(IntEnum):
    greenLine = 0
    wallFollow = 1
    purpleLine = 2
    elevator = 3
    cone = 4
    train = 5
    orangePlate = 6
    jump = 7

curState = State.greenLine

### Initialization ###
def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

### State Update ###
def update():
    global curState, arColorGlobal
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    color_image = rc.camera.get_color_image()
    depth_image = rc.camera.get_depth_image()

    markers = rc_utils.get_ar_markers(color_image)
    ar_marker: ARMarker = None
    

    #Check to see if there are any markers detected and grab the closest marker.
    if len(markers) > 0:
        ar_marker = markers[0]
        ar_marker.detect_colors(color_image, potential_colors_markers)
        arColor = ar_marker.get_color()
        arColorGlobal = arColor
        

    #Gets the car distance from marker.
    if ar_marker is not None:
        corners = ar_marker.get_corners()
        centerX = (corners[0][0] + corners[3][0]) //2
        centerY= (corners[0][1] + corners[3][1]) //2 
        marker_distance = depth_image[centerX][centerY]
        #rc_utils.draw_circle(image, center, rc_utils.ColorBGR.yellow.value)
    else: 
        marker_distance = None

    print(marker_distance)
    
    print(curState)
    if curState == State.greenLine:
        followLine()
    elif curState == State.wallFollow:
        print("wallfollowenter")
        wallFollow()
        if wall_follow_end is True:
            curState = State.greenLine
    elif curState == State.elevator:
        parkInElevator()
        if elevator_end is True:
            curState = State.greenLine
    # elif curState == State.jump:
    #     finalStageLineFollowing()
    #     if final_jump_end is True:
    #         curState = State.greenLine


    if ar_marker is not None and marker_distance != 0.0: 
        if marker_distance < 70:
            if ar_marker.get_id() == 0:
                curState = State.wallFollow
            elif ar_marker.get_id() == 1:
                curState = State.purpleLine
            elif ar_marker.get_id() == 4:
                curState = State.cone
            elif ar_marker.get_id() == 5:
                curState = State.train
            elif ar_marker.get_id() == 6:
                curState = State.orangePlate
            elif ar_marker.get_id() == 8:
                curState = State.jump
        if ar_marker.get_id() == 3:
            curState = State.elevator
            
    rc.drive.set_speed_angle(speed, angle)

def update_contour(color):
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
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

        contours = rc_utils.find_contours(image, color[0], color[1])
        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

        if contour is not None:
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)
        else:
            contour_center = None
            contour_area = 0

### Follows green line - Default state ###
def followLine():
    global speed
    global angle

    update_contour(GREEN)
    imgX = rc.camera.get_width()

    if contour_center is not None:
        angle = rc_utils.remap_range(contour_center[1],0,imgX,-1,1)

    speed = 2

def wallFollow():
    global speed
    global angle
    global wall_follow_end
    global contour_center

    update_contour(GREEN)
    
    scan = rc.lidar.get_samples()
    color_image = rc.camera.get_color_image()
    depth_image = rc.camera.get_depth_image()

    rf_angle, rf_dist = rc_utils.get_lidar_closest_point(scan, RIGHT_FRONT_WINDOW)
    lf_angle, lf_dist = rc_utils.get_lidar_closest_point(scan, LEFT_FRONT_WINDOW)
    front_angle, front_dist = rc_utils.get_lidar_closest_point(scan, FRONT_WINDOW)

    markers = rc_utils.get_ar_markers(color_image)
    ar_marker: ARMarker = None
    
    #Check to see if there are any markers detected and grab the closest marker.
    if len(markers) > 0:
        ar_marker = markers[0]

    if front_dist < 170:
        if rf_dist > lf_dist:
            dif_dist_r = rc_utils.clamp(rf_dist - lf_dist, 0, 50)
            angle = rc_utils.remap_range(dif_dist_r, 0, 50, 0, 1)
        elif lf_dist > rf_dist:
            dif_dist_l = rc_utils.clamp(lf_dist - rf_dist, 0, 50)
            angle = rc_utils.remap_range(dif_dist_l, 0, 50, 0, -1)

        if contour_center is not None and ar_marker is None:
            print("end")
            angle = 0
            wall_follow_end = True
    else:
        angle = 0

    speed = 1.5



def parkInElevator():

    global speed, angle, curState, elevator_end

    blue = ((90, 100, 100), (120, 255, 255), "blue")
    red = ((170, 100, 100), (10, 255, 255), "red")
    orange = ((7, 172, 78), (27, 192, 158), "orange")
    potential_colors = [blue, red, orange]
    
    color_image = rc.camera.get_color_image()
    depth_image = rc.camera.get_depth_image()

    markers = rc_utils.get_ar_markers(color_image)

    angle = 0
    if len(markers) > 0:
    
        marker = markers[0]

        corners = marker.get_corners()
        centerX = (corners[0][0] + corners[3][0]) //2
        centerY = (corners[0][1] + corners[3][1]) //2 
        
        angle = rc_utils.remap_range(centerY, 0, rc.camera.get_width(), -1, 1) 
        angle = rc_utils.clamp(angle, -1, 1)

        marker_distance = depth_image[centerX][centerY]

        marker.detect_colors(color_image, potential_colors)
        print(marker.get_color())
        if marker_distance > 155 or marker.get_color() == "blue":
            speed = 1
        elif marker_distance < 155 or marker.get_color() == "red" or marker.get_color == "orange":
            speed = 0
    else:
        elevator_end = True

# def finalStageLineFollowing():
#     global speed
#     global angle
#     global rightLine
#     global image, finalJump, counter
#     global contour_center
#     # global final_jump_end
#     #print(order)

#     update_contour(GREEN)
#     image = rc.camera.get_color_image()
#     print("RIGHT LINE: ", rightLine)
#     image = rc_utils.crop(image, (0, 3 * rc.camera.get_width() // 4), (rc.camera.get_height(), rc.camera.get_width()))

#     markers = rc_utils.get_ar_markers(image)
#     ar_marker: ARMarker = None
    
#     #Check to see if there are any markers detected and grab the closest marker.
#     if len(markers) > 0:
#         ar_marker = markers[0]

#     # rc.display.show_color_image(image)

#     scan = rc.lidar.get_samples()
#     _, left_dist = rc_utils.get_lidar_closest_point(scan, LEFT_WINDOW_LIDAR)
#     _, right_dist = rc_utils.get_lidar_closest_point(scan, RIGHT_WINDOW_LIDAR)
#     _, front_dist = rc_utils.get_lidar_closest_point(scan, FRONT_WINDOW_LIDAR)
#     _, back_dist = rc_utils.get_lidar_closest_point(scan, BACK_WINDOW_LIDAR)

#     update_contour_alt([BLUE], image)

#     imgWidth = rc.camera.get_width()
#     halfImgWidth = imgWidth // 2

#     quarterImgWidth = rc.camera.get_width() // 4
#     print(f"Front: {front_dist}   Back: {back_dist}")

#     if counter > 18 and ((front_dist < 170 and back_dist > 145) or finalJump == True):
#         print("final ramp")
#         speed = 3
#         angle = 0
#         finalJump = True
#     if finalJump == False:
#         counter += rc.get_delta_time()
#         print("Counter: ", counter)
#         if contour_center is not None:
#             print("Current Color: ", currentColor)

#             centerClamped = rc_utils.clamp(contour_center[1], int(0.75 * quarterImgWidth), quarterImgWidth)
#             angle = rc_utils.remap_range(centerClamped, int(0.75 * quarterImgWidth), quarterImgWidth, -1, 1)
#             print("Angle: ", angle)
#             speed = 1

#         if contour_center is None:
#             angle = 0.3
#             speed = 1.0

#     if contour_center is not None and ar_marker is None:
#         final_jump_end = True
########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
