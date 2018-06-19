#! usr/bin/env python

import rospy
import time
from audio.turtlebot_audio import TurtlebotAudio
from movement.detector_obstaculo_pasillo import ObstacleDetector
from movement.accion_mover_timers import Move as Move
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


# rospy.Subscriber("/obstaculos", String, move.callback_obstaculo)
rospy.Timer(rospy.Duration(0.03),move.controlled_tick)


#-----------test-------------
# start_pose = {'x':0.5,'y':1.3,'theta':0}
goal_pose = {'x':2.1,'y':0.5,'theta':0}
# goals = []
# nodes = path_finding.findPath(start_pose, goal_pose)
# if nodes is not None:
#     for node in nodes:
#         goals.append({'x':node.x,'y':node.y,'theta':0})
# else:
#     print('Error. nodes no valido:')

# path_finding.plotPathFound(nodes)
# #-----------------------------
goals_publisher = rospy.Publisher("/lista_goals", String, queue_size=1)
rospy.sleep(1)
# print(goals)
# goals_publisher.publish(String(json.dumps(goals)))
speaker = TurtlebotAudio()



# rospy.sleep(1)

# rospy.Timer(rospy.Duration(0.3), localization.plotParticles)
# rospy.Timer(rospy.Duration(0.3),localization.timer_located)

aux_audio = False
isGoing = False

cond_termino = False
while True and not cond_termino:
    if not path_finding.isLocated:
        print("not located")
        # Moverse hasta localizarse
        if not aux_audio:
            speaker.say('Localizandome')
            aux_audio = True
        obst.canPublish = True
        isGoing = False
    else:
        # print("located")
        if aux_audio:
            speaker.say('Path planning')
            aux_audio = False
        
        obst.canPublish = False
        if path_finding.location is not None:
            #Hacer path finding
            print("location",path_finding.location)
            if not isGoing:
                isGoing = True
                start_pose = path_finding.location
                nodes = path_finding.findPath(start_pose, goal_pose)
                if not bool(nodes):
                    print('Error. "nodes" no valido:', nodes)
                    isGoing = False
                else:
                    goals = []
                    for node in nodes:
                        goals.append({'x': node.x, 'y': node.y, 'theta':0, 'isGoal':False })
                    goals[-1]['isGoal'] = True
                    goals_publisher.publish(String(json.dumps(goals)))
                    print("PUBLICADO",goals)
    rospy.sleep(1)


rospy.spin()
