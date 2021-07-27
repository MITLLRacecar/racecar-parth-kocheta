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
    
    lidar_arr = rc_utils.get_lidar_average_distance(scan, i , 2.5)