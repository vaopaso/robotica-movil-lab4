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
# from path_finding.data_structure import Matrix
# from path_finding.path_finding import PathFinding
from localization.c_space import isPainted,paintNeighbours,read_pgm,print_matrix


# from blur_map import callback_scan, first_generation_particles
# from c_space import c_spce, read_pgm

        
rospy.init_node("movement_test")

move = Move()
obst = ObstacleDetector()

bottom_left_origin = [0,0]
resolution = 0.1
# path_finding = PathFinding('../../include/map.pgm', bottom_left_origin, resolution)

rospy.sleep(1)


# rospy.Subscriber("/obstaculos", String, move.callback_obstaculo)
rospy.Timer(rospy.Duration(0.03),move.controlled_tick)


#-----------test-------------
start_pose = {'x':0.5,'y':1.3,'theta':0}
goal_pose = {'x':2.1,'y':0.5,'theta':0}
# nodes = path_finding.findPath(start_pose, goal_pose)
# if nodes is not None:
#     for i in nodes:
#         print(i)
# else:
#     print('Error. nodes no valido:')

# path_finding.plotPathFound(nodes)
#-----------------------------


#                 goals.append({'x': node.x, 'y': node.y, 'theta':0 })


goals_publisher = rospy.Publisher("/lista_goals", String, queue_size=5)
rospy.sleep(1)
# rospy.Subscriber("/lista_goals", String, move.callback_goal)
rospy.sleep(1)


arista = 1
move.pose_mapa = {'x':0,'y':0,'theta':0}
goals_list = json.dumps([{'x': 0.4, 'y': 0.4, 'theta': 0}])
print("hola")
goals_publisher.publish(String(goals_list))
# print(String(goals_list))



rospy.spin()
