"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 1 - Driving in Shapes
"""

########################################################################################
# Imports
########################################################################################

import sys

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Put any global variables here
counter = 0
isA = False
isB = False
isX = False
isY = False
########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
 # If we use a global variable in our function, we must list it at
    # the beginning of our function like this
    global counter
    global isA
    global isB
    global isX
    global isY
    # The start function is a great place to give initial values to global variables
    counter = 0
    isA = False
    isB = False
    isX = False
    isY = False
    # This tells the car to begin at a standstill
    rc.drive.stop()

    print(
        ">> Lab 1 - Driving in Shapes\n"
        "\n"
        "Controls:\n"
        "    Right trigger = accelerate forward\n"
        "    Left trigger = accelerate backward\n"
        "    Left joystick = turn front wheels\n"
        "    A button = drive in a circle\n"
        "    B button = drive in a square\n"
        "    X button = drive in a figure eight\n"
    )


def update():


    global counter
    global isA
    global isB
    global isX
    global isY
    # This prints a message every time the A button is pressed on the controller
   
    # Reset the counter and start driving in an L every time the B button is pressed on
    # the controller
    if rc.controller.was_pressed(rc.controller.Button.A):
        print("Driving in a circle...")
        counter = 0
        isA = True

    if isA:
        counter += rc.get_delta_time()
        if counter < 6.4:
            rc.drive.set_speed_angle(1, 1)
        elif counter < 6.5:
            rc.drive.set_speed_angle(0.3, 1)    
        elif counter < 6.6:
            rc.drive.set_speed_angle(-0.1, 1)        
        else:
            rc.drive.stop()
            isA = False


    if rc.controller.was_pressed(rc.controller.Button.B):
        print("Driving in a square...")
        counter = 0
        isB = True

    if isB:
        counter += rc.get_delta_time()
        turntime = 1.38
        straighttime = 1 
        if counter < straighttime:
            rc.drive.set_speed_angle(1, 0)    

        elif counter < straighttime + turntime:
            rc.drive.set_speed_angle(1, 1)
        elif counter < straighttime + turntime + straighttime :
            rc.drive.set_speed_angle(1, 0)    

        elif counter < straighttime + turntime + straighttime + turntime :
            rc.drive.set_speed_angle(1, 1)   
    
        elif counter < straighttime + turntime + straighttime + turntime +straighttime :
            rc.drive.set_speed_angle(1, 0)    
    
        elif counter < straighttime + turntime + straighttime + turntime + straighttime + turntime :
            rc.drive.set_speed_angle(1, 1)    
        elif counter < straighttime + turntime + straighttime + turntime + straighttime + turntime + straighttime :
            rc.drive.set_speed_angle(1, 0)
        elif counter <  straighttime + turntime + straighttime + turntime + straighttime + turntime + straighttime + turntime :
            rc.drive.set_speed_angle(1, 1)    
        elif counter <  straighttime + turntime + straighttime + turntime + straighttime + turntime + straighttime + turntime + straighttime:
            rc.drive.set_speed_angle(1, 0)                  
        else:
            # Otherwise, stop the car
            rc.drive.stop()
            isB = False

    if rc.controller.was_pressed(rc.controller.Button.X ):
        print("Driving in a figure eight...")
        counter = 0
        isX = True

    if isX:
        counter += rc.get_delta_time()
        if counter < 3:
            rc.drive.set_speed_angle(1, 1)
        elif counter < 5:
            rc.drive.set_speed_angle(1, 0)
        elif counter < 8.2:
            rc.drive.set_speed_angle(1, -1)
        elif counter < 10:
            rc.drive.set_speed_angle(1, 0)
        elif counter < 12:
            rc.drive.set_speed_angle(1, 1)    
        else:
            # Otherwise, stop the car
            rc.drive.stop()
            isX = False

    if rc.controller.was_pressed(rc.controller.Button.Y ):
        print("Driving in a myshape...")
        counter = 0
        isY = True

    if isY:
        counter += rc.get_delta_time()
        if counter < 1:
            rc.drive.set_speed_angle(1, 1)
        elif counter < 4:
            rc.drive.set_speed_angle(1, -1)
        elif counter < 5:
            rc.drive.set_speed_angle(0.5, 0.5)    
        elif counter < 8.5:
            rc.drive.set_speed_angle(-1, -1)        
        else:
            # Otherwise, stop the car
            rc.drive.stop()
            isY = False


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
