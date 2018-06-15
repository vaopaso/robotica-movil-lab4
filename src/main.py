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
from localization.c_space import isPainted,paintNeighbours,read_pgm,print_matrix

# from blur_map import callback_scan, first_generation_particles
# from c_space import c_spce, read_pgm

_RED = [0, 0, 255]
_BLUE = [255, 0, 0]
_GREEN = [0, 255, 0]
_BLACK = [0, 0, 0]

class PathFinding:

    def __init__(self, path_map, bottom_left_origin, resolution):
        self.mapa = read_pgm(path_map)
        self.width_pixels = self.mapa.shape[1]
        self.height_pixels = self.mapa.shape[0]
        self.bottom_left_origin = bottom_left_origin
        self.resolution = resolution

        self.img = cv2.imread(path_map)
        #cspace
        radius = 0.2/self.resolution
        self.cspace_matrix = copy.deepcopy(self.mapa)
        self.c_space(self.mapa,self.cspace_matrix, radius=radius)

        rospy.Subscriber('/is_located', String, self.isLocatedCallback)
        rospy.Subscriber('/location', String, self.locationCallback)

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
        if pos is not None:
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

    def plotPathFound(self, nodes):
        # rad = self.particle_radius*self.display_resolution
        image = copy.deepcopy(self.img)
        
        if nodes is not None:
            for node in nodes:
                cv2.rectangle(image,(node.j,node.i),(node.j,node.i),_BLUE, 1)
        # if nodes is not None:
        #     for node in nodes:
        #         i, j = mapCoords_to_pixel(particle['x'], particle['y'], image.shape[0], image.shape[1] , self.origin, self.display_resolution)
        #         # print(particle['x'],i,particle['y'],j)
        #         if isValidPixel(i, j, image.shape[0], image.shape[1]):
        #             # draw robot circle
        #             cv2.circle(image, (j,i), self.particle_radius, _RED, -1)
        #             # draw direction
        #             y_prima = rad*np.sin(particle['theta']) + particle['y']
        #             x_prima = rad*np.cos(particle['theta']) + particle['x']
        #             # print(x_prima,y_prima)
        #             # x_prima,y_prima = mapCoords_to_pixel(x_prima,y_prima,image.shape[0],image.shape[1],bottom_left_origin,resolution)
        #             # y_prima = int(round(particle_radius*np.sin(particle['theta'])))
        #             # x_prima = int(round(particle_radius*np.cos(particle['theta'])))
        #             x_prima,y_prima = mapCoords_to_pixel(x_prima,y_prima,image.shape[0],image.shape[1],self.origin,self.display_resolution)
        #             cv2.line(image, (j,i), (y_prima,x_prima), _BLACK, 1)

        # Display window
        cv2.namedWindow('Path Finding', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Path Finding', 600,600)
        cv2.startWindowThread()
        cv2.imshow('Path Finding', image)

        
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
