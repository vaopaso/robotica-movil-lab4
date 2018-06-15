#! usr/bin/env python

import rospy
import time
import cv2
from movement.detector_obstaculo_pasillo import ObstacleDetector
from movement.accion_mover_timers import Move
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import copy
import numpy as np
import math
import json
from path_finding.data_structure import Matrix

# from blur_map import callback_scan, first_generation_particles
# from c_space import c_spce, read_pgm

class PathFinding:

    def __init__(self, cspace_matrix, width_pixels, height_pixels , bottom_left_origin, resolution):
        rospy.Subscriber('/is_located', String, self.isLocatedCallback)
        rospy.Subscriber('/location', String, self.isLocatedCallback)
        self.isLocated = False
        self.location = None
        matrix = Matrix(cspace_matrix, width_pixels, height_pixels, bottom_left_origin, resolution)


    def isLocatedCallback(self, data):
        res = str(data.data)
        if res == 'true':
            self.isLocated = True
        elif res == 'false':
            self.isLocated = False
        else:
            raise Exception('Error de envio o recepcion de is_located. No es ni "true" ni "false"')
    
    def locationCallback(self, data):
        pos = json.loads(str(data.data))
        if pos:
            self.location = {'x': pos['x'], 'y': pos['y'], 'theta': pos['theta']}
        else:
            self.location = None
    
    def findPath(self, initial_pos, goal_pos):
        


move = Move()
path_finding = PathFinding()
obst = ObstacleDetector()

goal_pose = [2.1, 0.5]

time.sleep(1)


# goals_publisher.publish(String(goals_list))
rospy.Subscriber("/obstaculos", String, move.callback_obstaculo)
rospy.Timer(rospy.Duration(0.03),move.controlled_tick)

# goals_publisher = rospy.Publisher("/lista_goals", String, queue_size=1)


rospy.sleep(1)

rospy.Timer(rospy.Duration(0.3), localization.plotParticles)
rospy.Timer(rospy.Duration(0.3),localization.timer_located)



rospy.spin()
