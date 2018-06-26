import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String

import math
import time
import json

import numpy as np
np.set_printoptions(threshold=np.nan)
import cv2
from cv_bridge import CvBridge, CvBridgeError

class ObstacleDetector(object):

    def __init__(self):
        rospy.on_shutdown(self.shutdown)
        self.bridge = CvBridge()
        self.current_cv_depth_image = np.zeros((1, 1))
        # self.publisher = rospy.Publisher("/walls", String, queue_size=1)
        self.__depth_img = rospy.Subscriber('/camera/depth/image_raw', Image, self.__depth_handler)
        self.goals_publisher = rospy.Publisher("/lista_goals_locate", String, queue_size=1)
        self.obstacle_publisher = rospy.Publisher("/obstacles", String, queue_size=1)
        self.canPublish = False

    def __depth_handler(self,data):
        try:
            self.current_cv_depth_image = np.asarray(self.bridge.imgmsg_to_cv(data, "32FC1"))
            # if self.canPublish:
            #     self.publish_obs()
            self.publish_obs()

        except CvBridgeError, e:
            print("error")
            rospy.logerr(e)

    def publish_obs(self):
        # eliminamos los pixeles que den menos de 40 cm, que es bajo la precision del robot.
        # self.current_cv_depth_image = self.current_cv_depth_image[:, :500]

        new_height = self.current_cv_depth_image.shape[1]*0.65
        self.current_cv_depth_image = self.current_cv_depth_image[:new_height, :]

        mid_width = self.current_cv_depth_image.shape[0]*1.0/2
        mid_height = self.current_cv_depth_image.shape[1]*1.0/2

        # mask = self.current_cv_depth_image < 1
        # self.current_cv_depth_image[mask] = 5000
        # filtered_image = self.current_cv_depth_image

        array_left = self.current_cv_depth_image[mid_height, :mid_width]
        array_right = self.current_cv_depth_image[mid_height, mid_width:]

        minval_left = 10000
        minval_right = 10000

        try:
            minval_left = np.min(array_left[np.nonzero(array_left)])
            minval_right = np.min(array_right[np.nonzero(array_right)])
        except:
            pass

        # values = json.dumps({'minval_left': float(minval_left), 'minval_right': float(minval_right)})

        # self.publisher.publish(String(values))
        self.avoid_walls(minval_left, minval_right)

    def avoid_walls(self, minval_left, minval_right, threshold=650):


        # print(minval_left, minval_right)
        path_finding_obstacle = False # si va a chocar o no
        if not self.canPublish:
            threshold = 570

        # solo se publica un goal en caso que algun lado este a menos del threshold
        if minval_left < threshold or minval_right < threshold:
            path_finding_obstacle = True
            if minval_left < minval_right:
                x = 800
                y = -1000
            else:
                x = 800
                y = 1000
            
            if minval_left < threshold and minval_right < threshold:
                #girar hacia la derecha
                x = 800
                y = -1000

        else:
            x = 1000
            y = 0

        if minval_left == 10000 or minval_right == 10000:
            path_finding_obstacle = True
            x = -850
            y = 0

        if self.canPublish: # en caso que pueda publicar al goals_locate (movimiento de localizacion)
            goal = json.dumps([{'x': x*1.0/1000, 'y': y*1.0/1000, 'theta': 0}])
            self.goals_publisher.publish(String(goal))
        else: # en este caso es para el path finding local (para no chocar accidentalmente en path planning)
            if path_finding_obstacle: # solo publicamos en caso que efectivamente vaya a chocar
                goal = json.dumps([{'x': x*1.0/1000, 'y': y*1.0/1000, 'theta': 0, 'isGoal': False}])
                self.obstacle_publisher.publish(String(goal))


    def shutdown(self):
        rospy.sleep(1)
