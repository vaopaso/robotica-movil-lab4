#! usr/bin/env python

import rospy
import time
from movement.detector_obstaculo_pasillo import ObstacleDetector
from movement.accion_mover_timers import Move
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import copy
import numpy as np
import math
import cv2
import json
from path_finding.data_structure import Matrix
from path_finding.path_finding import PathFinding
from localization.c_space import isPainted,paintNeighbours,read_pgm,print_matrix


# from blur_map import callback_scan, first_generation_particles
# from c_space import c_spce, read_pgm

        
rospy.init_node("path_finding")

move = Move()
obst = ObstacleDetector()

bottom_left_origin = [0,0]
resolution = 0.1
path_finding = PathFinding('../../include/map.pgm', bottom_left_origin, resolution)

rospy.sleep(1)


rospy.Subscriber("/obstaculos", String, move.callback_obstaculo)
# rospy.Timer(rospy.Duration(0.03),move.controlled_tick)


#-----------test-------------
start_pose = {'x':0.5,'y':1.3,'theta':0}
goal_pose = {'x':2.1,'y':0.5,'theta':0}
nodes = path_finding.findPath(start_pose, goal_pose)
if nodes is not None:
    for i in nodes:
        print(i)
else:
    print('Error. nodes no valido:')

path_finding.plotPathFound(nodes)
#-----------------------------


# rospy.sleep(1)

# rospy.Timer(rospy.Duration(0.3), localization.plotParticles)
# rospy.Timer(rospy.Duration(0.3),localization.timer_located)

cond_termino = False
while True and not cond_termino:
    if not path_finding.isLocated:
        # Moverse hasta localizarse
        # (Activar timer de move.controlled_tick)
        pass
    else:
        # Desactivar timer de move.controlled_tick
        if path_finding.location is not None:
            #Hacer path finding
            start_pose = path_finding.location
            nodes = path_finding.findPath(start_pose, goal_pose)
            if not bool(nodes):
                print('Error. "nodes" no valido:', nodes)
            else:
                node = nodes.pop(0)
                goals = []
                goals.append({'x': node.x, 'y': node.y, 'theta':0 })


rospy.spin()
