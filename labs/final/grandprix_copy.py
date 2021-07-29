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




potential_colors = [BLUE, RED, GREEN]
potential_colors_markers = [PURPLEMARKER, ORANGEMARKER]
potential_colors_lines = [PURPLELINE, ORANGELINE]

speed = 0
angle = 0

wall_follow_end = False
final_jump_end = False
elevator_end = False
canyon_end = False
orange_pillar_end = False
cone_end = False

rightLine = 0

time = 0.0
# Camera values.
contour_center = None
contour_area = 0

cur_color = None
contour_distance = 0.0

cone_counter = 0
prev_color = None

MIN_CONTOUR_AREA = 10

currentColor = None

counter = 0

finalJump = False

arColor = None

wallfollow = False
linefollow = False

order = (GREEN, RED, BLUE)
turnright= False
turnleft = False

firstLoop = True
firstTurn = True
cur_side = "START"

foundLines = False
straight = False
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
    orangePillar = 8
    nothing = 9
    greenLineFast = 10

curState = State.greenLineFast

### Initialization ###
def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

### State Update ###
def update():
    global curState, arColorGlobal, counter, wallfollow, orange_pillar_end, speed, ar_marker
    #global speed, angle, linefollow, wallfollow, counter, turnright, turnleft, order, firstLoop, firstTurn, cur_side, straight, ar_marker, 
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    
    color_image = rc.camera.get_color_image()
    depth_image = rc.camera.get_depth_image()

    markers = rc_utils.get_ar_markers(color_image)
    ar_marker = None
    

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

    #print(marker_distance)
    
    #print(curState)

    if ar_marker is not None and marker_distance != 0.0: 
        
        if marker_distance < 70:
            if ar_marker.get_id() == 0:
                if wall_follow_end is False:
                    curState = State.wallFollow
                elif wall_follow_end is True:
                    curState = State.nothing
                    wallfollow = True
           
            
            # elif ar_marker.get_id() == 5:
            #     curState = State.train

            elif ar_marker.get_id() == 6:
                curState = State.orangePlate

            # elif ar_marker.get_id() == 8:
            #     curState = State.jump

        if ar_marker.get_id() == 3:
            curState = State.elevator
        
        if marker_distance < 60 and ar_marker.get_id() == 4:
            curState = State.cone   
        elif ar_marker.get_id() == 1:
                speed = 0.5
                curState = State.purpleLine
                if canyon_end == True:
                    curState = State.greenLine

    if curState == State.greenLineFast:
        followLineFast()

    elif curState == State.wallFollow :
        #print("wallfollowenter")
        wallFollow()
        if wall_follow_end is True:
            curState = State.greenLine
            
    elif curState == State.elevator:
        parkInElevator()
        if elevator_end is True:
            curState = State.greenLine

    elif curState == State.cone:
        #print("doing cones")
        cone()
        if cone_end is True:
            curState = State.greenLine
        
    elif curState == State.purpleLine:
        newCanyon(arColorGlobal)
        speed = 0.75
        if canyon_end == True:
            curState = State.greenLine

    elif curState == State.greenLine:
        followLine()

            
    # elif curState == State.jump:
    #     finalStageLineFollowing()
    #     if final_jump_end is True:
    #         curState = State.greenLine

    if wallfollow == True:
        orangeColumns()
    if turnright == True:
         turn_right()
    if turnleft == True:
        turn_left()
    if straight == True:
        straight2()

    if orange_pillar_end is True:
            curState = State.greenLine
            orange_pillar_end = False

        
    #print(angle)
    rc.drive.set_speed_angle(speed, angle)

def update_contour(color):
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area
    image= rc.camera.get_color_image()
    
    if image is None:
        contour_center = None
        contour_area = 0
    else:
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

        contours = rc_utils.find_contours(image, color[0], color[1])
        contour = rc_utils.get_largest_contour(contours, 1000)

        if contour is not None:
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)
        else:
            contour_center = None
            contour_area = 0
    #rc.display.show_color_image(image)
    
### Follows green line - Default state ###
def update_contour3(color, image):
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area

    
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
    #rc.display.show_color_image(image)

def followLine():
    global speed
    global angle
   
    update_contour(GREEN)
    imgX = rc.camera.get_width()

    if contour_center is not None:
        angle = rc_utils.remap_range(contour_center[1],0,imgX,-1,1)

    speed = 1.0

def followLineFast():
    global speed
    global angle
   
    update_contour(GREEN)
    imgX = rc.camera.get_width()

    if contour_center is not None:
        angle = rc_utils.remap_range(contour_center[1],0,imgX,-1,1)

    speed = 2

def newCanyon3():#arColor):
    global speed
    global angle
    global rightLine
    global canyon_end
    global image
    #print(order)
    speed = 1
    image = rc.camera.get_color_image()
    image = rc_utils.crop(image, (0, 5 * rc.camera.get_width() // 8), (rc.camera.get_height(), rc.camera.get_width()))
    

    update_contour3(ORANGELINE, image)
    if contour_center is None:
        update_contour3(PURPLELINE, image)
        print("Following the purple line")
    else:
        print("Following the orange line")
    
    #update_contour(arColor)

    quarterImgWidth = rc.camera.get_width() // 4

    if contour_center is not None:
        center = rc_utils.clamp(contour_center[1], int(0.05 * quarterImgWidth), quarterImgWidth)
        angle = rc_utils.remap_range(center, int(0.05* quarterImgWidth), quarterImgWidth, -1.4, 1.4)
           
    else: 
        print("Turning, no line found")
        angle = 1

    # update_contour(GREEN)
    # # if contour_center is not None:
    # #     canyon_end = True
    # #     speed = 1.3
        
### Wall Follow ###
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
            speed = 1
            wall_follow_end = True
    else:
        angle = 0

    speed = 1.2

### Elevator Parking ###
def parkInElevator():

    global speed, angle, curState, elevator_end

    blue = ((90, 100, 100), (120, 255, 255), "blue")
    red = ((170, 100, 100), (10, 255, 255), "red")
    orange = ((7, 172, 78), (27, 192, 158), "orange")
    potential_colors = [blue, red, orange]
    
    color_image = rc.camera.get_color_image()
    depth_image = rc.camera.get_depth_image()

    markers = rc_utils.get_ar_markers(color_image)

    update_contour(GREEN)

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
        #print(marker.get_color())

        if marker_distance > 200 or marker.get_color() == "blue" or marker.get_color == "orange":
            speed = 1
     
        elif marker.get_color() == "red":
            speed = 0
            
        if marker.get_id() != 3:
            elevator_end = True


######################################################################################################################################

def turn_right():
    global counter, angle, speed, wallfollow, turnright, cur_side
        
  
    if counter < 1.35:
        counter += rc.get_delta_time()
        angle = 0.55
        speed = 1
    else:

        turnright = False
        wallfollow = True
        cur_side = "RIGHT"

def turn_left():
    
    global counter, angle, wallfollow, speed, cur_side, turnleft

        
     
    if counter < 1.35:
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

#####################################################################################################################################################################
def update_contour_canyon(colorList, image, crop = True):
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area
    global currentColor
    global canyon_end
    if image is None:
        contour_center = None
        contour_area = 0
    else:
        #print("in update cont")
        if crop:
            image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        #rc.display.show_color_image(image)
        for color in colorList:
            contours = rc_utils.find_contours(image, color[0], color[1])
            if len(contours) != 0:
                break

        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)
        if contour is not None:
            #print("found cont")
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)
            currentColor = color[2]
           # print(f"Current color: {currentColor}")
            
        else:
            contour_center = None
            contour_area = 0
    rc.display.show_color_image(image)

def update_contour_canyon2(colorList, image, crop = False):
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area
    global currentColor
    global canyon_end
    if image is None:
        contour_center = None
        contour_area = 0
    else:
        #print("in update cont")
        if crop:
            image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        #rc.display.show_color_image(image)
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
    rc.display.show_color_image(image)   

def newCanyon(arColor):
    global speed
    global angle
    global rightLine
    global image, defaultUncroppedImage, foundLines
    global canyon_end
    global contour_center
    global contour_area
    #print(order)
    speed = -3
    image = rc.camera.get_color_image()
    defaultUncroppedImage = image

    #update_contour_canyon([WHITE], defaultUncroppedImage, False)
    #rc.display.show_color_image(image)
    if ar_marker is not None and not foundLines: #this is true until you are very close to the ar marker
        print("Aligning to AR marker")
        # center = rc_utils.clamp(contour_center[1], 0, rc.camera.get_width())
        # angle = rc_utils.remap_range(center, 0, rc.camera.get_width(), -1.0, 1.0)
        angle = 0.25
        speed = -3

    else:
        speed = 0.6
        foundLines = True
        #print("RIGHT LINE: ", rightLine)
        if rightLine % 2 == 0:
            #print("in correct image")
            image = rc_utils.crop(image, (0, 3 * rc.camera.get_width() // 4), (rc.camera.get_height(), rc.camera.get_width()))
        elif rightLine % 2 == 1:
            image = rc_utils.crop(image, (0, 0), (rc.camera.get_height(), rc.camera.get_width() // 4))

        
        if arColor == "ORANGE":
            update_contour_canyon([PURPLELINE, ORANGELINE], image)
        elif arColor == "PURPLE":
            update_contour_canyon([ORANGELINE, PURPLELINE], image)
        # if arColor == "PURPLE":
        #     update_contour([PURPLE, ORANGE])
        # elif arColor == "ORANGE":
        #     update_contour([ORANGE, PURPLE])

        imgWidth = rc.camera.get_width()
        halfImgWidth = imgWidth // 2

        quarterImgWidth = rc.camera.get_width() // 4

        if contour_center is not None:
            print("AR Color:" ,arColor)
            #print("Current Color: ", currentColor)

            if arColor == "PURPLE" or arColor == "ORANGE":
                if (currentColor == "PURPLE" and arColor == "PURPLE") or (currentColor == "ORANGE" and arColor == "ORANGE"):
                        #print("Contour center: ",contour_center[1])
                        print("RIGHT LINE FOLLOWING")
                    #center = rc_utils.clamp(contour_center[1], 3 * imgWidth // 4, imgWidth)
                    #centerInv = rc_utils.clamp(contour_center[1], 3 * imgWidth // 4, imgWidth)
                        
                if (currentColor == "PURPLE" and arColor == "ORANGE") or (currentColor == "ORANGE" and arColor == "PURPLE"):
                    rightLine += 1
                    #print("Contour center: ",contour_center[1])
                    print("LEFT LINE FOLLOWING")
                    
                if rightLine % 2 == 0:
                    #RIGHT FOLLOWING
                    #print("Changing angle")
                    #print("Lower bound: ", int(0.75 * quarterImgWidth))
                    #print("Upper bound: ", quarterImgWidth)
                        
                    centerInv = rc_utils.clamp(contour_center[1], int(0.75 * quarterImgWidth), quarterImgWidth)
                    angle = rc_utils.remap_range(centerInv, int(0.75 * quarterImgWidth), quarterImgWidth, -1, 1)
                    #print("Angle: ", angle)
                if rightLine % 2 == 1:
                    #LEFT FOLLOWING
                    centerInv = rc_utils.clamp(contour_center[1], 0, int(0.35 * quarterImgWidth))
                    angle = rc_utils.remap_range(centerInv, 0, int(0.35 * quarterImgWidth), -1, 1)
                    if currentColor != arColor and arColor == "PURPLE":
                        speed = 0.5
        

    
    #if no white, orange, or purple follow green
    update_contour(GREEN)   
    print("contour are" + str(contour_area)) 
    print("right line" + str(rightLine))
    if contour_area !=0 and contour_center is not None and rightLine > 0:
        canyon_end = True

#####################################################################################################################################################################
def orangeColumns():
    
    global speed,angle, linefollow, wallfollow, counter, turnright, turnleft, order, firstLoop, firstTurn, cur_side, straight, ar_marker, orange_pillar_end
    scan = rc.lidar.get_samples()
    pid = PID(0.01, 0.1, 0.1, setpoint=0)
    pid.output_limits = (-1.4, 1.4)
    color_image = rc.camera.get_color_image()
    depth_image = rc.camera.get_depth_image()

    markers = rc_utils.get_ar_markers(color_image)
    
    marker: ARMarker = None
    

    if len(markers) > 0:
        marker = markers[0]
    print(marker)
    front_dist = rc_utils.get_lidar_average_distance(scan, 0 ,5)
    top_right = rc_utils.get_lidar_average_distance(scan,42, 10 )
    top_left =  rc_utils.get_lidar_average_distance(scan,318, 10 )
    right = rc_utils.get_lidar_average_distance(scan,90, 10 )
    left  = rc_utils.get_lidar_average_distance(scan,270, 10 )
    #print("front dist" + str(front_dist))
    diff_top =  top_right -top_left 
    diff_top2 = top_left - top_right
    control = pid(diff_top2)

    speed = rc_utils.remap_range(front_dist, 0, 250, 0.8, 1.2, True)
    old_angle = angle
    angle  = rc_utils.clamp (control,-1.3,1.3)

    update_contour(GREEN)

    if marker is not None:
        corners = marker.get_corners()
        centerX = (corners[0][0] + corners[3][0]) //2
        centerY= (corners[0][1] + corners[3][1]) //2 
        marker_distance = depth_image[centerX][centerY]
        #rc_utils.draw_circle(image, center, rc_utils.ColorBGR.yellow.value)
    else: 
        marker_distance = 9999

    #print(marker)
    #angle  = rc_utils.clamp (diff_top * 0.02,-1.6,1.6)
    # if marker is not None:
    #print(marker)
    
    

    if (firstLoop):
        if front_dist > 125:
            speed = 0.5
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
                   

    
    elif firstTurn == False:
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
        
    if contour_center is not None and marker is None and firstTurn == False:
        print("going into end statement")
        wallfollow = False
        turnleft = False
        turn_right = False
        straight = False
        orange_pillar_end = True
        speed = 1

    print(cur_side)

#############################################################################################################

def update_contour_cone():
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

        red_contours = rc_utils.find_contours(image, RED_CONE[0], RED_CONE[1])
        blue_contours = rc_utils.find_contours(image, BLUE_CONE[0], BLUE_CONE[1])

        largest_red_contour = rc_utils.get_largest_contour(red_contours, 80)
        largest_blue_contour = rc_utils.get_largest_contour(blue_contours, 80)

        if largest_red_contour is not None:
            largest_red_contour_area = rc_utils.get_contour_area(largest_red_contour)
            largest_red_contour_distance = rc_utils.get_pixel_average_distance(depth_image_adjust_blur, rc_utils.get_contour_center(largest_red_contour))
        else:
            largest_red_contour_area = 0
            largest_red_contour_distance = 0

        if largest_blue_contour is not None:
            largest_blue_contour_area = rc_utils.get_contour_area(largest_blue_contour)
            largest_blue_contour_distance = rc_utils.get_pixel_average_distance(depth_image_adjust_blur, rc_utils.get_contour_center(largest_blue_contour))
        else:
            largest_blue_contour_area = 0
            largest_blue_contour_distance = 0

        # Select the largest contour
        if largest_red_contour_area > largest_blue_contour_area and largest_red_contour_distance < 500:
            contour = largest_red_contour
            cur_color = RED_CONE[2]
        elif largest_blue_contour_area > largest_red_contour_area and largest_blue_contour_distance < 500:
            contour = largest_blue_contour
            cur_color = BLUE_CONE[2]
        else:
            contour = None

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
    
def cone():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """

    
    update_contour_cone()

    # color_img = rc.camera.get_color_image()
    # depth_img = rc.camera.get_depth_image()
    
    # Global variables.
    global speed
    global angle
    global cur_color
    global contour_distance
    global cone_counter
    global prev_color
    global time

    # Variables.
    speed = 0.0
    angle = 0.0

    color_img_x = rc.camera.get_width()

    if prev_color != cur_color:
        cone_counter += 1
        prev_color = cur_color
    

    if cur_color == 'BLUE':
        if contour_center is not None:
            point = rc_utils.remap_range(contour_distance, 10, 300, color_img_x, color_img_x * 3 // 4 , True)
            #speed = rc_utils.remap_range(contour_distance,30, 120,0.8,1,True,)
            speed = 0.45
            if point ==  color_img_x // 2:
                point = point + 0.001
            angle = rc_utils.remap_range(contour_center[1], point, color_img_x // 2 , 0 ,-0.9 ,True)
            if contour_distance > 145:
                angle = -0.17
           
        # elif cur_color == 'RED':
        #     angle = 0.2 #0.32
        #     speed = 0.7
        # elif cur_color == None:
        #     angle = 0.5
        else:
            angle = 0.3 
            speed = 0.45
    elif cur_color == 'RED':
        if contour_center is not None:
            point = rc_utils.remap_range(contour_distance, 10, 550, 0, color_img_x // 2, True)
            #speed = rc_utils.remap_range(contour_distance,30, 120,0.7,1,True,)
            speed = 0.45
            if point ==  color_img_x // 2:
                point = point + 0.001
            angle = rc_utils.remap_range(contour_center[1], point, color_img_x // 2 , 0 ,0.9 ,True)
            if contour_distance > 145:
                
                angle = 0.17
 
        #     angle = -0.2    #-0.32
        #     speed = 0.7
        # elif cur_color == None:
        #     angle = -0.5
        else:
            angle = -0.3 
            speed = 0.45

    print(angle)   
        # elif cur_color == 'BLUE':
    #print(cone_counter)
    #print(cone_counter)
    #print(contour_distance)

    update_contour(GREEN)
    if contour_center is not None:
        cone_end = True

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
