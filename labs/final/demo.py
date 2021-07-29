import sys
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from nptyping import NDArray
from typing import Any, Tuple, List, Optional

sys.path.insert(0, "../../library")   
import racecar_core
import racecar_utils as rc_utils
from enum import IntEnum
import random
from IPython.display import clear_output

rc = racecar_core.create_racecar()
file = open("values.csv")
q_table = np.loadtxt(file, delimiter=",")

print(q_table)
MIN_DISTANCE = 50
window_size = 15



speed = 0
angle = 0
state = 0
action = 0


def observation():
    global done
    

    scan = rc.lidar.get_samples()
    first_dist = rc_utils.get_lidar_average_distance(scan, 285, window_size)
    second_dist = rc_utils.get_lidar_average_distance(scan, 315, window_size)
    third_dist = rc_utils.get_lidar_average_distance(scan, 345, window_size)
    fourth_dist = rc_utils.get_lidar_average_distance(scan, 15, window_size)
    fifth_dist = rc_utils.get_lidar_average_distance(scan, 45, window_size)
    sixth_dist = rc_utils.get_lidar_average_distance(scan, 75, window_size)

    lidar_data = [0,0,0,0,0,0]
    if (first_dist < MIN_DISTANCE):
        lidar_data[0] = 1
    if (second_dist < MIN_DISTANCE):
        lidar_data[1] = 1
    if (third_dist < MIN_DISTANCE):
        lidar_data[2] = 1
    if (fourth_dist < MIN_DISTANCE):
        lidar_data[3] = 1
    if (fifth_dist < MIN_DISTANCE):
        lidar_data[4] = 1
    if (sixth_dist < MIN_DISTANCE):
        lidar_data[5] = 1
    
    
    lidar_data_str = "".join(str(data) for data in lidar_data)
    

    state = int(lidar_data_str, 2)
  
    
    

    return (state)
    


def save(table):
    np.savetxt('values.csv', table, delimiter=",", fmt="%d")


def start():
    print("RL Project")
    global speed, angle
    global done
    
    global count
    rc.drive.stop()
    done = True
    count = 0

def update():

   
        
        
    global action, speed, angle

    state = observation() #function to return state, reward, and done/not done
        


    action = np.argmax(q_table[state]) # Exploit learned values

    speed = .5
    if action == 0:
        angle = -0.7
    elif action ==1:
        angle = 0
    elif action == 2:
        angle = 0.7 

    
    rc.drive.set_speed_angle(speed, angle)


if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()

