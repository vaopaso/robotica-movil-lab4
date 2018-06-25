#! usr/bin/env python

import rospy
import time
import cv2
from movement.detector_obstaculo_pasillo import ObstacleDetector
# from movement.accion_mover_timers2 import Move
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from localization.localization import Localization
from localization.weighting import mapCoords_to_pixel,isValidPixel
from localization.c_space import print_matrix
import copy
import yaml
import numpy as np
import math
import json

import signal
import sys


def handlerZ(signum, frame):
    print('  -->  Ctrl+Z catch. Killing process...')
    sys.exit()

signal.signal(signal.SIGTSTP, handlerZ)


num_particles = 200
rospy.init_node("localization")


localization = Localization('../../include/map.pgm','../../include/map.yaml',num_particles)
rospy.Subscriber("/scan", LaserScan, localization.callback_scan)
rospy.Subscriber("/odom", Odometry, localization.odom_callback)

# move = Move()
# obst = ObstacleDetector()

# goals_publisher = rospy.Publisher("/lista_goals", String, queue_size=1)
initial_pose_publisher = rospy.Publisher("/initial_pose",String,queue_size=1)
initial_pose = {'x':0.4,'y':2,'theta':0}

rospy.Subscriber('/initial_pose',String,localization.initial_pose_callback)

rospy.sleep(1)
initial_pose_publisher.publish(String(json.dumps(initial_pose)))
# rospy.Subscriber("/obstaculos", String, move.callback_obstaculo)
# rospy.Timer(rospy.Duration(0.03),move.controlled_tick)

rospy.Timer(rospy.Duration(0.3), localization.plotParticles)
rospy.Timer(rospy.Duration(0.3),localization.timer_located)


cond_termino = False
while not cond_termino:
    if not localization.start_locating:
        continue
    localization.locate()
    localization.sampling_process()

# listener que queda esperando y ejecuta los callback cuando corresponda
rospy.spin()
