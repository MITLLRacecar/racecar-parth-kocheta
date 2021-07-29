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
# from simple_pid import PID
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
ORANGE = ((7, 172, 78), (27, 192, 158), "ORANGE")
GREEN = ((40, 60, 60), (90, 255, 255), "GREEN") 
WHITE = ((90, 20, 200), (115, 60, 255), "WHITE")
PURPLE = ((125, 245, 215), (145, 255, 255), "PURPLE")
ORANGE = ((5, 245, 215), (25, 255, 255), "ORANGE")

### LIDAR WINDOW ###
FRONT_WINDOW = (-10, 10)
RIGHT_FRONT_WINDOW = (85, 95)
LEFT_FRONT_WINDOW = (265, 275)

### CROPPED IMAGE ###
CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))


MIN_CONTOUR_AREA = 30

potential_colors = [BLUE, RED, GREEN]

speed = 0
angle = 0

wall_follow_end = False

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
    # elif curState == State.elevator:
    #     parkInElevator()

    if ar_marker is not None:
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

    if ar_marker is not None:
        if ar_marker.get_id() == 199 and ar_marker.get_orientation() == ar_marker.LEFT:
            angle = -1
        elif ar_marker.get_id() == 199 and ar_marker.get_orientation() == ar_marker.RIGHT:
            angle = 1
    else:
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

def parkInElevator():
    global speed, angle, curState

    potential_colors = [BLUE, RED, ORANGE]

    color_image = rc.camera.get_color_image()
    depth_image = rc.camera.get_depth_image()

    markers = rc_utils.get_ar_markers(color_image)
    
    marker: ARMarker = None
    
    if len(markers) > 0:
        marker = markers[0]

        corners = marker.get_corners()
        centerX = (corners[0][0] + corners[3][0]) //2
        centerY = (corners[0][1] + corners[3][1]) //2 
        marker_distance = depth_image[centerX][centerY]

        marker.detect_colors(color_image, potential_colors)

        if marker_distance > 155 or marker.get_color() == "BLUE":
            speed = 1
        elif marker_distance < 155 or marker.get_color() == "RED" or marker.get_color == "ORANGE":
            speed = 0
    else:
        update_contour(GREEN)
        if contour_center is not None:
            curState = State.greenLine

# ### Cone Follow ###
# def coneFollow():

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
    
def avoidTrains():
    global speed
    depth_image = rc.camera.get_depth_image()
    color_image = rc.camera.get_color_image()

    # leftDepth = depth_image[rc.camera.get_height() // 2, rc.camera.get_width() // 3] #1/4
    # middleDepth = depth_image[rc.camera.get_height() // 2, rc.camera.get_width() // 2] #1/2
    # rightDepth = depth_image[rc.camera.get_height() // 2, 2 * rc.camera.get_width() // 3] #3/4

    scan = rc.lidar.get_samples_async()
    leftDepth = rc_utils.get_lidar_average_distance(scan, -25, 5)
    middleDepth = rc_utils.get_lidar_average_distance(scan, 0, 5)
    rightDepth = rc_utils.get_lidar_average_distance(scan, 25, 5)
    farLeft = rc_utils.get_lidar_average_distance(scan, -45, 5)
    farRight = rc_utils.get_lidar_average_distance(scan, 45, 5)
    closestAngle, closestDistance = rc_utils.get_lidar_closest_point(scan, (-30, 30))
    
    print(f"{leftDepth}, {middleDepth}, {rightDepth}")
    print(closestDistance)
    # update_contour(((109, 175, 77), (129, 195, 157)), color_image)
    # if closestDistance > 200:
    #     speed = 1
    # if closestDistance < 50:
    #     speed = -0.2
    # else:
    # if farLeft > 40 and farRight > 40:
    #     speed = rc_utils.remap_range(closestDistance, 50, 250, 0, 0.8)
        # speed = -0.5
    # else:
    #     speed = -0.8
    #     if leftDepth >= 80 or rightDepth >= 80:
    #         speed = 0.8
    if closestDistance < 50:
        speed = 0
    if closestDistance < 200 and closestDistance > 50:
        speed = rc_utils.remap_range(closestDistance, 50, 250, 0, 0.8)
    elif closestDistance > 200:
        speed = 1
    elif closestDistance < 50:
        speed = 0
    
########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
