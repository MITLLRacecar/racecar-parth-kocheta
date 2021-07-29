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
#lookup table, contains reward values initially at zero for every state-action combination
q_table = np.zeros([64, 3])

MIN_DISTANCE = 50
window_size = 15
# Hyperparameters
alpha = 0.1
gamma = 0.6
epsilon = 0.1
count = 0
speed = 0
angle = 0
state = 0
action = 0
# For plotting metrics
all_epochs = []
all_penalties = []

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
    if lidar_data == [0,0,0,0,0,0]:
        
        done = True
    
    

    if rc_utils.get_lidar_closest_point(scan)[1] < 20.5:
        print("CRASHED")
        save(q_table)
        reward = -10
    else:
        reward = rc_utils.remap_range(abs(fifth_dist-second_dist), 0, 100, 4, -4, True)
        done = True
        print(reward)
    #print(rc_utils.get_lidar_closest_point(scan)[1])
    return (state, reward, done)
    


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
    global count
    global speed, angle
    global done
    global state, action
    global count
    global epochs, penalties
        # rc.drive.stop() - something to reset the environment
    if count < 10000:
        if done:
            epochs, penalties, reward, = 0, 0, 0
            state = observation()[0]
            action = 1
            done = False
            count += 1
        
        
        
        if not done:
            next_state, reward, done = observation() #function to return state, reward, and done/not done
                
            old_value = q_table[state, action]
            next_max = np.max(q_table[next_state])
            
            new_value = (1 - alpha) * old_value + alpha * (reward + gamma * next_max)
            q_table[state, action] = new_value

            if reward == -10:
                penalties += 1

            state = next_state

            epochs += 1
            if random.uniform(0, 1) < epsilon:
                action = np.random.randint(0, 5) 
            else:
                action = np.argmax(q_table[state]) # Exploit learned values
            speed = .5
            if action == 0:
                angle = -0.7
            elif action ==1:
                angle = 0
            elif action == 2:
                angle = 0.7 


        if count % 5 == 0:

            clear_output(wait=True)

            print(f"Episode: {count}")
        
    else:
    
        print("Training finished.\n")
        
    rc.drive.set_speed_angle(speed, angle)
if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()

