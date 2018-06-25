#!/usr/bin/env python
#-*- coding: utf-8 -*-

'''
Script to transform the depth map from Asus Xtion (or Kinect) to a 3D cloud point
attached to the base_link (robot).

'''

import rospy
import rosparam
import tf
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import json

class DeepScan(object):

    def __init__(self):
        rospy.init_node( 'DeepScan', anonymous=True )

        P = rospy.get_param('/navigation/projection_3D_to_kinect', "[[575.8157348632812, 0.0, 314.5, 0.0],[ 0.0, 575.8157348632812, 235.5, 0.0],[ 0.0, 0.0, 1.0, 0.0],[0.0,0.0,0.0,1.0]]")
        self.P = np.array(json.loads(P))
        self.PI = np.array(np.linalg.inv(np.matrix(self.P)))

        # subscribers
        self.__dep_img_sub = rospy.Subscriber( '/camera/depth/image_raw', Image, self.__depth_handler )

        # publishers
        # publish the scan from the depth map

        self.__bridge = CvBridge()
        self.__current_depth_image = Image()


        self.listener = tf.TransformListener()
        self.M = None

        self.alpha_vector = np.linspace(-90,90,181)
        ind_nan = np.where(np.abs(self.alpha_vector)==90)[0]
        self.sec = 1.0/np.cos(self.alpha_vector*np.pi/180.0)
        self.sec[ind_nan] = 1.0
        self.tan = np.tan(self.alpha_vector*np.pi/180)
        self.tan[ind_nan] = 1.0
        self.yfactor = np.ones(self.alpha_vector.shape)
        self.yfactor[ind_nan] = 0.0

        self.sec = self.sec.reshape(-1,1)
        self.tan = self.tan.reshape(-1,1)
        self.yfactor = self.yfactor.reshape(-1,1)

        self.seq = 0

        self.subsample_msg = 10
        self.count_msg = 0

        self.scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
        self.has_transform = False

    def __depth_handler( self, data ):
        self.count_msg += 1
        if self.count_msg%self.subsample_msg != 0:
            return

        if not self.has_transform:
            try:
                rospy.loginfo(rospy.get_namespace())
                self.listener.waitForTransform("/base_link","/kinect_frame", rospy.Time(), rospy.Duration(4.0))
                (trans,rot) = self.listener.lookupTransform('/base_link', '/kinect_frame', rospy.Time(0))
                self.has_transform = True
                if self.M is None:
                    self.M = self.listener.fromTranslationRotation(trans, rot)
                    rospy.loginfo(str(self.M))
            # except Exception as inst:
            #     print(inst.message)
            #     rospy.loginfo(type(inst))


            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("error")
                rospy.loginfo("Not getting kinect to base_link transform ")

        if self.has_transform:

            subsample_factor = 5
            self.__current_depth_image = data
            depth_array = self.__bridge.imgmsg_to_cv2( self.__current_depth_image )[:,:,0]
            # depth_array = self.__bridge.imgmsg_to_cv2( self.__current_depth_image )

            depth_array = depth_array[0::subsample_factor, 0::subsample_factor]
            # Convert from depth image to 3D coordinates in kinect_nav frame
            H,W = depth_array.shape
            p3D_kinect = np.array((H,W,4))
            Y,X = np.where(depth_array)
            YK = H-1 - Y
            D = depth_array[(Y,X)]/1000.0 # depth in meters
            dep_data = np.ones((4, len(X)))
            dep_data[0,:] = X*D*subsample_factor
            dep_data[1,:] = YK*D*subsample_factor
            dep_data[2,:] = D


            # get the 3D point cloud seen from kinect
            pcl_data = np.dot(self.PI, dep_data)
            pcl_data[2,:] *= -1

            # get the 3D point cloud seen from the robot
            base_data = np.dot(self.M, pcl_data)
            ang_kinect = np.arctan2(base_data[1,:], base_data[0,:])

            # remove floor data
            ind = np.where(base_data[2,:] > 0.05)[0]
            # get the points with Z > 0.05
            base_data_filt = base_data[:,ind]

            X = base_data_filt[0,:].reshape(1,-1)
            Y = base_data_filt[1,:].reshape(1,-1)

            # get the distances to rays every 1.0 degrees
            dist_to_rays = np.dot(self.tan, X) - np.dot(self.yfactor, Y)
            dist_to_rays = np.abs(dist_to_rays / self.sec)

            assigned_rays = np.argmin(dist_to_rays, axis=0)

            # rospy.loginfo(str(X.shape) + "   " + str(Y.shape))
            distances = np.sqrt(X*X + Y*Y)[0,:]
            scan_kinect = np.zeros((2, len(self.alpha_vector)))
            scan_kinect[0,:] = self.alpha_vector*np.pi/180
            scan_kinect[1,:] = 20.0
            for i,alpha in enumerate(self.alpha_vector):
                scan_kinect[0,i] = alpha*np.pi/180
                ind = np.where(assigned_rays == i)[0]
                if len(ind):
                    scan_kinect[1,i] = np.min(distances[ind])

            # create LaserScan message
            self.seq += 1
            laserScan = LaserScan()

            laserScan.header.seq = self.seq
            laserScan.header.stamp = rospy.Time.now()
            laserScan.header.frame_id = "base_link"

            laserScan.angle_min = -90.0*np.pi/180
            laserScan.angle_max = 90.0*np.pi/180
            laserScan.angle_increment = 1.0*np.pi/180
            laserScan.time_increment = 0.00001
            laserScan.scan_time = 0.001*181
            laserScan.range_min = 0.0
            laserScan.range_max = 20.0

            laserScan.ranges = scan_kinect[1,:]
            laserScan.intensities = []

            # finally publish!!
            self.scan_pub.publish(laserScan)

if __name__ == "__main__":
    dscan = DeepScan()
    rospy.spin()
