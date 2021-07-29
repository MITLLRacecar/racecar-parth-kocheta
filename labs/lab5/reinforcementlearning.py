import sys
import cv2 as cv
import numpy as np
sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
from racecar_utils import ARMarker
from enum import Enum, IntEnum
import random

rc = racecar_core.create_racecar()
speed = 0
angle = 0

lidar_arr = []

def start():
    rc.drive.stop()


def update():
    global lidar_arr    
    update_arr()
    print (lidar_arr)


def train_net(model, state):
    epsilon = 1
    train_frames = 100000 
    observe = 1000
    t = 0
    distance = 0
    while t < train_frames:
        t += 1
        distance +=1
        if random.random() < epsilon or t < observe:
            action = np.random.randint(0, 3) 
        else:
            qval = model.predict(state, batch_size=1)
            action = (np.argmax(qval))  

#make it save the model every x frames

def update_arr():

    scan = rc.lidar.get_samples()
    for i in range (270, )
    lidar_arr = rc_utils.get_lidar_average_distance(scan, i , 2.5)














    import sys
import cv2 as cv
import numpy as np
sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
from racecar_utils import ARMarker
from enum import Enum, IntEnum

rc = racecar_core.create_racecar()
speed = 0
angle = 0

lidar_arr = []

def start():
    rc.drive.stop()


def update():
    global lidar_arr    
    update_arr()
    print (lidar_arr)


def update_arr():

    scan = rc.lidar.get_samples()
    for i in range (270, )
    lidar_arr = rc_utils.get_lidar_average_distance(scan, i , 2.5)