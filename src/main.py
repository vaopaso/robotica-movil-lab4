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
import json
from path_finding.data_structure import Matrix
from localization.c_space import isPainted,paintNeighbours,read_pgm,print_matrix

# from blur_map import callback_scan, first_generation_particles
# from c_space import c_spce, read_pgm

class PathFinding:

    def __init__(self, path_map, bottom_left_origin, resolution):
        self.mapa = read_pgm(path_map)
        self.width_pixels = self.mapa.shape[1]
        self.height_pixels = self.mapa.shape[0]
        self.bottom_left_origin = bottom_left_origin
        self.resolution = resolution
        #cspace
        radius = 0.2/self.resolution
        self.cspace_matrix = copy.deepcopy(self.mapa)
        self.c_space(self.mapa,self.cspace_matrix, radius=radius)

        rospy.Subscriber('/is_located', String, self.isLocatedCallback)
        rospy.Subscriber('/location', String, self.isLocatedCallback)

        self.isLocated = False
        self.location = None

        self.matrix = Matrix(self.cspace_matrix, self.width_pixels, self.height_pixels, self.bottom_left_origin, self.resolution)

    def c_space(self, matrix_original, matrix, radius):
        #radius: radio del robot (en pixeles de mapa)
        for i in range(len(matrix_original)):
            for j in range(len(matrix_original[i])):
                if isPainted(matrix_original, i, j):
                    paintNeighbours(matrix, i, j, radius, 0)


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
    
    def findPath(self, start_pos, goal_pos):
        start_node = self.matrix.getNode(start_pos['x'], start_pos['y'])
        goal_node = self.matrix.getNode(goal_pos['x'], goal_pos['y'])
        if (start_node is None) or (goal_node is None):
            print('Error PathFinding. start_node o goal_node no es valido')
            return None
        else:
            return self.matrix.findPath(start_node, goal_node)

        


move = Move()
obst = ObstacleDetector()

bottom_left_origin = [0,0]
resolution = 0.1
path_finding = PathFinding('../include/map.pgm', bottom_left_origin, resolution)

rospy.sleep(1)


rospy.Subscriber("/obstaculos", String, move.callback_obstaculo)
rospy.Timer(rospy.Duration(0.03),move.controlled_tick)

initial_pose = {'x':0.5,'y':1.3,'theta':0}
goal_pose = {'x':2.1,'y':0.5,'theta':0}
nodes = path_finding.findPath(initial_pose, goal_pose)
if nodes:
    for i in nodes:
        print(i)
else:
    print('Error. nodes no valido:', nodes)
    
# rospy.sleep(1)

# rospy.Timer(rospy.Duration(0.3), localization.plotParticles)
# rospy.Timer(rospy.Duration(0.3),localization.timer_located)



rospy.spin()
