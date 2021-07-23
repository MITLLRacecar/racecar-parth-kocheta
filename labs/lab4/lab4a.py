"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 4A - LIDAR Safety Stop
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
prev_forward  = 0
real_speed =0
# >> Constants
# The (min, max) degrees to consider when measuring forward and rear distances
FRONT_WINDOW = (-10, 10)
REAR_WINDOW = (170, 190)

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

    global prev_forward
    global real_speed
    # Print start message
    print(
        ">> Lab 4A - LIDAR Safety Stop\n"
        "\n"
        "Controls:\n"
        "    Right trigger = accelerate forward\n"
        "    Right bumper = override forward safety stop\n"
        "    Left trigger = accelerate backward\n"
        "    Left bumper = override rear safety stop\n"
        "    Left joystick = turn front wheels\n"
        "    A button = print current speed and angle\n"
        "    B button = print forward and back distances"
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global prev_forward
    global real_speed
    # Use the triggers to control the car's speed
    rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    
    speed = rt - lt
    # Calculate the distance in front of and behind the car
    scan = rc.lidar.get_samples()
    _, forward_dist = rc_utils.get_lidar_closest_point(scan, FRONT_WINDOW)
    _, back_dist = rc_utils.get_lidar_closest_point(scan, REAR_WINDOW)

    """
    speed: float
    frame_speed = (prev_forward - forward_dist) / rc.get_delta_time()
    real_speed += (frame_speed - real_speed)

    prev_forward = forward_dist
    # TODO (warmup): Prevent the car from hitting things in front or behind it.
    # Allow the user to override safety stop by holding the left or right bumper.
    min_stop = 30 
    stop_point = min_stop + real_speed
    slow_point = 2 * stop_point
    if stop_point < forward_dist < slow_point:
        if not rc.controller.is_down(rc.controller.Button.RB):
                speed = rc_utils.remap_range(forward_dist, stop_point, slow_point, 0, 0.5)
           

    if  back_dist < stop_point:
    """

    if 100 <= forward_dist <= 150 and not rc.controller.is_down(rc.controller.Button.RB) :
        speed = rc_utils.clamp(speed, 0, 0.1)
    elif (forward_dist < 100 ) and not rc.controller.is_down(rc.controller.Button.RB):
        speed = 0

        

    if 100 <= back_dist <= 150 and not rc.controller.is_down(rc.controller.Button.RB) :
        speed = rc_utils.clamp(speed, 0, -0.1)
        
    elif back_dist > 100 and not rc.controller.is_down(rc.controller.Button.RB):
        speed = 0
    
    
    
    # Use the left joystick to control the angle of the front wheels
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]

    rc.drive.set_speed_angle(speed, angle)

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the distance of the closest object in front of and behind the car
    
    print("Forward distance:", forward_dist, "Back distance:", back_dist)

    # Display the current LIDAR scan
    rc.display.show_lidar(scan)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
