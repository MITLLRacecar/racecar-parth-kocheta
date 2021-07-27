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
from simple_pid import PID
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
BLUE = ((90, 100, 100), (120, 255, 255), "BLUE")
RED = ((170, 100, 100), (10, 255, 255), "RED")
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

### CROPPED IMAGE ###
CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))


MIN_CONTOUR_AREA = 30

potential_colors = [BLUE, RED, GREEN]
potential_colors_markers = [PURPLEMARKER, ORANGEMARKER]
potential_colors_lines = [PURPLELINE, ORANGELINE]

speed = 0
angle = 0

wall_follow_end = False

rightLine = 0


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
    global curState
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

    if curState == State.greenLine:
        followLine()
    elif curState == State.wallFollow:
        wallFollow()
        if wall_follow_end is True:
            curState = State.greenLine


    if ar_marker is not None and marker_distance < 100:
        if ar_marker.get_id() == 0:
            curState = State.wallFollow
        elif ar_marker.get_id() == 1:
            curState = State.purpleLine
        elif ar_marker.get_id() == 3:
            curState = State.elevator
        elif ar_marker.get_id() == 4:
            curState = State.cone
        elif ar_marker.get_id() == 5:
            curState = State.train
        elif ar_marker.get_id() == 6:
            curState = State.orangePlate
        elif ar_marker.get_id() == 8:
            curState = State.jump
  
    # if curState == State.wallFollow:
    #     wallFollow()
    # elif curState == State.purpleLine:
    #     followLine()
    # elif curState == State.orangePillar:
    #     slalomPillars()
    # elif curState == State.elevator:
    #     parkInElevator()
    # elif curState == State.cone:
    #     coneSlalom()
    # elif curState == State.train:
    #     avoidTrains()
    # elif curState == State.orangePlate:
    #     avoidPlate()
    # elif curState == State.jump:
    #     bigJump()

    rc.drive.set_speed_angle(speed, angle)

### Wall Following ###
def wallFollow():
    global speed
    global angle
    global wall_follow_end
    
    scan = rc.lidar.get_samples()
    
    rf_angle, rf_dist = rc_utils.get_lidar_closest_point(scan, RIGHT_FRONT_WINDOW)
    lf_angle, lf_dist = rc_utils.get_lidar_closest_point(scan, LEFT_FRONT_WINDOW)
    front_angle, front_dist = rc_utils.get_lidar_closest_point(scan, FRONT_WINDOW)

    if rf_dist > lf_dist:
        dif_dist_r = rc_utils.clamp(rf_dist - lf_dist, 0, 50)
        angle = rc_utils.remap_range(dif_dist_r, 0, 50, 0, 1)
    elif lf_dist > rf_dist:
        dif_dist_l = rc_utils.clamp(lf_dist - rf_dist, 0, 50)
        angle = rc_utils.remap_range(dif_dist_l, 0, 50, 0, -1)

    if rf_dist > 200 and lf_dist > 200 and front_dist > 200:
        angle = 0
        wall_follow_end = True

    speed = 1

### Contour Find and Update ###
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

    speed = 1




def update_contour_purpleLine(colorList, image):
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area
    global currentColor
    
    if image is None:
        contour_center = None
        contour_area = 0
    else:
        #print("in update cont")
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        rc.display.show_color_image(image)
        for color in colorList:
            contours = rc_utils.find_contours(image, color[0], color[1])
            if len(contours) != 0:
                break

        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)
        if contour is not None:
            print("found cont")
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)
            currentColor = color[2]
            print(f"Current color: {currentColor}")
            
        else:
            contour_center = None
            contour_area = 0
    
def canyonLineFollowing(arColor):
    global speed
    global angle
    global rightLine
    global image
    #print(order)

    image = rc.camera.get_color_image()
    print("RIGHT LINE: ", rightLine)
    if rightLine % 2 == 0:
        print("in correct image")
        image = rc_utils.crop(image, (0, 3 * rc.camera.get_width() // 4), (rc.camera.get_height(), rc.camera.get_width()))
    elif rightLine % 2 == 1:
        image = rc_utils.crop(image, (0, 0), (rc.camera.get_height(), rc.camera.get_width() // 4))
    

    update_contour_purpleLine([PURPLELINE, ORANGELINE], image)

    imgWidth = rc.camera.get_width()
    # halfImgWidth = imgWidth // 2

    quarterImgWidth = rc.camera.get_width() // 4

    if contour_center is not None:
        print("AR Color:" ,arColor)
        print("Current Color: ", currentColor)

        if arColor == "PURPLE" or arColor == "ORANGE":
            if (currentColor == "PURPLE" and arColor == "PURPLE") or (currentColor == "ORANGE" and arColor == "ORANGE"):
                    print("Contour center: ",contour_center[1])
                    print("RIGHT LINE FOLLOWING")
                #center = rc_utils.clamp(contour_center[1], 3 * imgWidth // 4, imgWidth)
                #centerInv = rc_utils.clamp(contour_center[1], 3 * imgWidth // 4, imgWidth)
                    
            if (currentColor == "PURPLE" and arColor == "ORANGE") or (currentColor == "ORANGE" and arColor == "PURPLE"):
                rightLine += 1
                print("Contour center: ",contour_center[1])
                print("LEFT LINE FOLLOWING")
                
            if rightLine % 2 == 0:
                #RIGHT FOLLOWING
                #print("Changing angle")
                print("Lower bound: ", int(0.75 * quarterImgWidth))
                print("Upper bound: ", quarterImgWidth)

                centerInv = rc_utils.clamp(contour_center[1], int(0.75 * quarterImgWidth), quarterImgWidth)
                angle = rc_utils.remap_range(centerInv, int(0.75 * quarterImgWidth), quarterImgWidth, -1, 1)
                print("Angle: ", angle)
            if rightLine % 2 == 1:
                #LEFT FOLLOWING
                centerInv = rc_utils.clamp(contour_center[1], 0, int(0.25 * quarterImgWidth))
                angle = rc_utils.remap_range(centerInv, 0, int(0.25 * quarterImgWidth), -1, 1)
    speed = 1

# def followWall2():
#     global speed,angle, linefollow, wallfollow, counter, turnright, turnleft, order 
#     scan = rc.lidar.get_samples()
#     pid = PID(0.01, 0.1, 0.1, setpoint=0)
#     pid.output_limits = (-1.5, 1.5)
#     color_image = rc.camera.get_color_image()
#     depth_image = rc.camera.get_depth_image()

#     markers = rc_utils.get_ar_markers(color_image)
    
#     marker: ARMarker = None
    

#     if len(markers) > 0:
#         marker = markers[0]
        
#     #use depth image and corners to find center and the distance to the marker, then make turn right and turn left functions wbhen distance to marker is less than number
#     front_dist = rc_utils.get_lidar_average_distance(scan, 0 ,5)
#     top_right = rc_utils.get_lidar_average_distance(scan,42, 10 )
#     top_left =  rc_utils.get_lidar_average_distance(scan,318, 10 )
#     right = rc_utils.get_lidar_average_distance(scan,90, 10 )
#     left  = rc_utils.get_lidar_average_distance(scan,270, 10 )
#     #print("front dist" + str(front_dist))
#     diff_top =  top_right -top_left 
#     diff_top2 = top_left - top_right
#     control = pid(diff_top2)

#     speed = rc_utils.remap_range(front_dist, 0, 250, 1, 1.4, True)
#     old_angle = angle
#     angle  = rc_utils.clamp (control,-1.4,1.4)

#     if marker is not None:
#         corners = marker.get_corners()
#         centerx = (corners[0][0] + corners[3][0]) //2
#         centery= (corners[0][1] + corners[3][1]) //2 
#         marker_distance = depth_image[centerx][centery]
#         #rc_utils.draw_circle(image, center, rc_utils.ColorBGR.yellow.value)
#     else: 
#         marker_distance = 9999
    

    
########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
