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
BLUE = ((88,245,199), (108,255,255), "BLUE")
RED = ((0, 50, 50), (20, 255, 255), "RED")
GREEN = ((40, 60, 60), (90, 255, 255), "GREEN") 
WHITE = ((90, 20, 200), (115, 60, 255), "WHITE")
ORANGE = ((), (), "ORANGE")
PURPLE = ((), (), "PURPLE")

CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))

MIN_CONTOUR_AREA = 30

potential_colors = [GREEN, PURPLE, ORANGE]

speed = 0
angle = 0

currentColor = None

########################################################################################
# Functions
########################################################################################

class State(IntEnum):
    greenLine = 0
    wallFollow = 1
    purpleLine = 2
    orangePillar = 3
    elevator = 4
    cone = 5
    train = 6
    orangePlate = 7
    jump = 8

curState = State.greenLine

def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    color_image = rc.camera.get_color_image()
    markers = rc_utils.get_ar_markers(color_image)
    ar_marker: ARMarker = None
    

    if len(markers) > 0:
        ar_marker = markers[0]

    if ar_marker.getid() == 9:
        curState = State.wallFollow
    
  
    # if curState == State.wallFollow:
    #     wallFollow()
    # elif curState == State.purpleLine:
    #     followLine()
    # elif currState == State.orangePillar:
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

        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        for color in potential_colors:
            contours = rc_utils.find_contours(image, color[0], color[1])
            if len(contours) != 0:
                break

        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)
        if contour is not None:
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)
            currentColor = color[2]
            print(f"Current color: {currentColor}")
            
        else:
            contour_center = None
            contour_area = 0
    
def followLine():
    global speed
    global angle
    #print(order)
    update_contour()
    imgX = rc.camera.get_width()
    halfImgX = rc.ca
    if contour_center is not None:
        if currentColor == "GREEN":
            angle = rc_utils.remap_range(contour_center[1],0,imgX,-1,1)
        elif currentColor == "PURPLE" or currentColor == "ORANGE":
            angle = rc_utils.remap_range(contour_center[1],0,halfImgX,-1,1)
            
    speed = 1

    
########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
