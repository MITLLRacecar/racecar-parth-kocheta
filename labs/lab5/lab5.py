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

### LINE FOLLOWING ###
BLUE = ((88,245,199), (108,255,255))
RED = ((0, 50, 50), (20, 255, 255))#(175, 50, 50), (10, 255, 255))#
GREEN = ((40, 60, 60), (90, 255, 255)) #(40, 50, 50), (80, 255, 255)
WHITE = ((90, 20, 200), (115, 60, 255))
CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))

speed = 0
angle = 0

MIN_CONTOUR_AREA = 30
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
counter = 0

wallfollow = True
linefollow = False

order = (GREEN, RED, BLUE)
turnright= False
turnleft = False
########################################################################################
# Functions
########################################################################################

class Orientation(Enum):
    UP = 0
    LEFT = 1
    DOWN = 2
    RIGHT = 3

#left_state = Orientation.LEFT
#right_state = Orientation.RIGHT

def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(">> Lab 5 - AR Markers")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """

    global speed, angle, linefollow, wallfollow, counter, turnleft, turnright

    if linefollow == True:
        followLine()
    elif wallfollow == True:
        followWall2()  
    if turnright == True:
        turn_right()
    elif turnleft == True:
        turn_left()
    
    
    #print("angle" + str(angle))
    #print(counter)
    
    rc.drive.set_speed_angle(speed, angle)


def update_contour(order2):
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area
    global order
    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:

        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        color = order2
        for x in color:
            contours = rc_utils.find_contours(image, x[0], x[1])
            if len(contours) != 0:
                break

        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)
        if contour is not None:
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)

        else:
            contour_center = None
            contour_area = 0



    
def followLine():
    global speed
    global angle
    global order
    #print(order)
    update_contour(order)
    imgX = rc.camera.get_width()
    if contour_center is not None:
        angle = rc_utils.remap_range(contour_center[1],0,imgX,-1,1)
    speed = 1

def turn_right():
    global counter, angle, speed, wallfollow

    if counter < 0.7:
        counter += rc.get_delta_time()
        angle = 1
        speed = 1
    else:
        wallfollow = True

def turn_left():
    
    global counter, angle, wallfollow, speed
       
    if counter < 0.7:
        counter += rc.get_delta_time()
        angle = -1
        speed = 1

    else:
        wallfollow = True

    

def followWall2():
    global speed,angle, linefollow, wallfollow, counter, turnright, turnleft, order 
    scan = rc.lidar.get_samples()
    pid = PID(0.01, 0.1, 0.1, setpoint=0)
    pid.output_limits = (-1.5, 1.5)
    color_image = rc.camera.get_color_image()
    depth_image = rc.camera.get_depth_image()

    markers = rc_utils.get_ar_markers(color_image)
    
    marker: ARMarker = None
    

    if len(markers) > 0:
        marker = markers[0]
        
    #use depth image and corners to find center and the distance to the marker, then make turn right and turn left functions wbhen distance to marker is less than number
    front_dist = rc_utils.get_lidar_average_distance(scan, 0 ,5)
    top_right = rc_utils.get_lidar_average_distance(scan,42, 10 )
    top_left =  rc_utils.get_lidar_average_distance(scan,318, 10 )
    right = rc_utils.get_lidar_average_distance(scan,90, 10 )
    left  = rc_utils.get_lidar_average_distance(scan,270, 10 )
    #print("front dist" + str(front_dist))
    diff_top =  top_right -top_left 
    diff_top2 = top_left - top_right
    control = pid(diff_top2)

    speed = rc_utils.remap_range(front_dist, 0, 250, 1, 1.4, True)
    old_angle = angle
    angle  = rc_utils.clamp (control,-1.4,1.4)

    if marker is not None:
        corners = marker.get_corners()
        centerx = (corners[0][0] + corners[3][0]) //2
        centery= (corners[0][1] + corners[3][1]) //2 
        marker_distance = depth_image[centerx][centery]
        #rc_utils.draw_circle(image, center, rc_utils.ColorBGR.yellow.value)
    else: 
        marker_distance = 9999

    #print(marker)
    #angle  = rc_utils.clamp (diff_top * 0.02,-1.6,1.6)
    # if marker is not None:
    #print(marker)
    print(front_dist)
    if (marker is not None and marker.get_id() == 0 ):
        if marker_distance < 80 :
            print("calling left")
            counter=  0
            wallfollow = False
            turnleft = True
            angle = -1
       
    if (marker is not None and marker.get_id() == 1  ):
        if marker_distance < 80 :
            print("calling right")
            counter=  0
            wallfollow = False
            turnright = True
            angle = 1

    if (marker is not None and marker.get_id() == 2 and marker_distance < 80):
        marker.detect_colors(color_image, potential_colors)
        #print(marker.get_color())
        if marker.get_color() == 'RED':
            print("changing order red")
            order = (GREEN, RED, BLUE)
        elif marker.get_color() == 'BLUE':
            print("changing order blue")
            order = (GREEN, BLUE, RED)
        
        wallfollow = False
        linefollow = True
    if (marker is not None and marker.get_id() == 199 ) :
        print(marker.get_orientation().value)
        if  marker_distance < 80:
            if marker.get_orientation().value == 1:
                counter =  0
                wallfollow = False  
                turnleft = True 
            elif marker.get_orientation().value ==3 :
                counter=  0
                wallfollow = False
                turnright = True

    
########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
