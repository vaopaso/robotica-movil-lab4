import numpy as np
import sys
sys.path.append('../')
from localization.weighting import mapCoords_to_pixel
from localization.c_space import isPaintedOrUnexplored
from astar import AStar
import math

def pixelToCoords(i, j, width_pixels, height_pixels , bottom_left_origin, resolution):
      upper_left_origin = np.empty([2])
      upper_left_origin[0] = bottom_left_origin[0]
      upper_left_origin[1] = bottom_left_origin[1]+(height_pixels-1)*resolution

      y = upper_left_origin[1]-(i*resolution + resolution/2)
      x = upper_left_origin[0]+(j*resolution + resolution/2)
      return x,y

class Node: #Cell
      def __init__(self, i, j, width_pixels, height_pixels , bottom_left_origin, resolution):
            self.i = i
            self.j = j
            self.x, self.y = pixelToCoords(i, j, width_pixels, height_pixels , bottom_left_origin, resolution)
            # self.neighbours = np.zeros([8])
            self.neighbours = []

class Matrix(AStar):
      def __init__(self, cspace_matrix, width_pixels, height_pixels , bottom_left_origin, resolution):
            self.resolution = resolution
            self.width_pixels = width_pixels
            self.height_pixels = height_pixels
            self.bottom_left_origin = bottom_left_origin

            self.nodes = []
            self.nodes_dict = {}

            # generate nodes
            for i in range(cspace_matrix.shape[0]):
                  for j in range(cspace_matrix.shape[1]):
                        if not isPaintedOrUnexplored(cspace_matrix, i, j):
                              # si llega aqui, es una celda libre
                              node = Node(i, j, self.width_pixels, self.height_pixels , self.bottom_left_origin, self.resolution)
                              self.nodes.append(node)
                              self.nodes_dict[(i,j)] = node

            # make connections (fill neighbours)
            keys = self.nodes_dict.keys()
            for node in self.nodes:
                  i0 = node.i
                  j0 = node.j
                  for i in range(-1,2):
                        for j in range(-1,2):
                              if (i,j)!=(0,0):
                                    if (i0+i,j0+j) in keys:
                                          node.neighbours.append(self.nodes_dict[(i0+i,j0+j)])

      def heuristic_cost_estimate(self, n1, n2):
            (x1, y1) = (n1.i, n1.j)
            (x2, y2) = (n2.i, n2.j)
            return math.hypot(x2 - x1, y2 - y1)

      def distance_between(self, n1, n2):
            """this method returns 1, when the neighbors are vertically or horizontally adjacent
            and returns sqrt(2) when the neighbours are diagonal"""
            (x1, y1) = (n1.i, n1.j)
            (x2, y2) = (n2.i, n2.j)
            return math.hypot(x2 - x1, y2 - y1)

      def neighbors(self, node):
            return node.neighbours

      def findPath(self, start_node, goal_node):
            return list(self.astar(start_node, goal_node))

      def getNode(self, x, y):
            i, j = mapCoords_to_pixel(x, y, self.width_pixels, self.height_pixels, self.bottom_left_origin, self.resolution)
            keys = self.nodes_dict.keys()
            if (i,j) in keys:
                  return self.nodes_dict[(i,j)]
            return None # in this case the node does not exist in the cspace

