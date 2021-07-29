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
ORANGE = ((3, 174, 78), (23, 194, 158), "ORANGE")
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

firstLoop = True
firstTurn = True
cur_side = "START"


straight = False
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

    global speed, angle, wallfollow, counter, turnleft, turnright, firstLoop, firstTurn, cur_side, straight

    
    
    if wallfollow == True:
        orangeColumns()  
    if turnright == True:
         turn_right()
    if turnleft == True:
        turn_left()
    if straight == True:
        straight2()
    
    #slalom()
    #print("angle" + str(angle))
    #print(counter)
    print(angle)
    rc.drive.set_speed_angle(speed, angle)


def turn_right():
    global counter, angle, speed, wallfollow, turnright, cur_side
        
  
    if counter < 1.45:
        counter += rc.get_delta_time()
        angle = 0.55
        speed = 1
    else:

        turnright = False
        wallfollow = True
        cur_side = "RIGHT"

def turn_left():
    
    global counter, angle, wallfollow, speed, cur_side, turnleft

        
     
    if counter < 1.45:
        counter += rc.get_delta_time()
        angle = -0.55
        speed = 1

    else:
        turnleft  = False
        wallfollow = True
        cur_side = "LEFT"

    
def straight2():

    global counter, angle, wallfollow, speed, straight
       
    if counter < 2.4:
        counter += rc.get_delta_time()
        angle = 0
        speed = 1

    else:
        straight = False
        wallfollow = True


def orangeColumns():
    
    global speed,angle, linefollow, wallfollow, counter, turnright, turnleft, order, firstLoop, firstTurn, cur_side, straight, ar_marker
    scan = rc.lidar.get_samples()
    pid = PID(0.01, 0.1, 0.1, setpoint=0)
    pid.output_limits = (-1.5, 1.5)
    color_image = rc.camera.get_color_image()
    depth_image = rc.camera.get_depth_image()

    markers = rc_utils.get_ar_markers(color_image)
    
    marker: ARMarker = None
    

    if len(markers) > 0:
        marker = markers[0]
        
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
    
    

    if (firstLoop):
        if front_dist > 120:
            speed = 1
            angle = 0
        else: 
            firstLoop = False
        
    # elif (marker is not None and marker.get_id() == 199 ) and marker_distance < 70:
    #     #print(marker.get_orientation().value)
  
    #     if marker.get_orientation().value == 1  :
    #         cur_side = "LEFT" 
        

    #     elif marker.get_orientation().value ==3 :
    #         cur_side = "RIGHT" 
       

    if firstTurn == True and front_dist < 100:

        if marker is not None:
            firstTurn = False
            if marker.get_orientation().value == 1  :
                    print("first turn left")
                    counter =  0.6
                    wallfollow = False  
                    turnleft = True
                    firstTurn = False
                    #cur_side = "LEFT"
                    #angle = -1
            elif marker.get_orientation().value ==3 :
                    print("first turn right")
                    counter=  0.6
                    wallfollow = False
                    turnright = True
                    firstTurn = False
                    #cur_side = "RIGHT"
                   

    
    elif  firstTurn == False:
        if marker is not None:
            if cur_side == "LEFT" and marker.get_orientation().value == 1 :
                print("going straight on left")
                counter= 0 
                wallfollow = False
                straight = True
                
            elif cur_side == "LEFT" and marker.get_orientation().value == 3:
                print("wide right turn")
                counter =  0
                wallfollow = False
                turnright = True   
            elif  cur_side == "RIGHT" and marker.get_orientation().value == 3:
                print("going straight on right")
                counter = 0
                wallfollow = False
                straight = True

            elif cur_side == "RIGHT" and marker.get_orientation().value == 1:
                print("wide left turn")
                counter =  0
                wallfollow = False  
                turnleft = True

    print(cur_side)



def slalom():
    #update_contour()

    global speed, angle
    
    color_image = rc.camera.get_color_image()
    depth_image = rc.camera.get_depth_image()

    markers = rc_utils.get_ar_markers(color_image)
    
    ar_marker: ARMarker = None
    
    if len(markers) > 0:
        ar_marker = markers[0]
        corners = ar_marker.get_corners()
        centerx = (corners[0][0] + corners[3][0]) //2
        centery= (corners[0][1] + corners[3][1]) //2 
    
    
    targetPoint = rc.camera.get_width() / 2   
    if ar_marker is not None:
        curr_side = ar_marker.get_orientation().value
        if ar_marker.get_id() == 199: #and contour_center is not None:
            if curr_side == 1: #left
                centerOfMarker = corners[3][1] + corners[0][1] // 2
                targetPoint = centerOfMarker - 225
            elif curr_side == 3:
                centerOfMarker = corners[0][1] + corners[3][1] // 2
                targetPoint = centerOfMarker + 225
            angle = rc_utils.remap_range(targetPoint, 0, rc.camera.get_width(), -1, 1)
            # if contour_distance > 145:
            #     angle = -0.16
            #     speed = 1.4
        else:
            if curr_side == 1:
                angle = 0.33
            elif curr_side == 3:
                angle = -0.33
    speed = 1.4
        
        
        # if contour_center is not None:
        #     if marker.get_orientation().value == 3:
        #         targetPoint = contour_center[1] + 60
        #     point = rc_utils.remap_range(contour_distance, 50, 300, 0, color_img_x // 2, True)
        #     #speed = rc_utils.remap_range(contour_distance,30, 120,0.7,1,True,)
        #     speed = 1.4
        #     angle = rc_utils.remap_range(contour_center[1], point, color_img_x //2 , 0 ,1 ,True)
        #     if contour_distance > 145:
        #         angle = 0.16
        #         speed = 1.4
        # else:
        #     angle = -0.33
        #     speed = 1.4








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
