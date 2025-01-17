#!/usr/bin/env python3
import numpy as np
import os
import math
import cv2
from renderClass import Renderer
from dt_apriltags import Detector

import rospy
import yaml
import sys
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

import rospkg 


"""

This is a template that can be used as a starting point for the CRA1 exercise.
You need to project the model file in the 'models' directory on an AprilTag.
To help you with that, we have provided you with the Renderer class that render the obj file.

"""

class ARNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(ARNode, self).__init__(node_name=node_name,node_type=NodeType.GENERIC)
        self.veh = rospy.get_namespace().strip("/")

        # Load calibration files
        self.calib_data = self.readYamlFile('/data/config/calibrations/camera_intrinsic/' + self.veh + '.yaml')
        self.log('Loaded intrinsics calibration file')
        self.extrinsics = self.readYamlFile('/data/config/calibrations/camera_extrinsic/' + self.veh + '.yaml')
        self.log('Loaded extrinsics calibration file') 

        # Retrieve intrinsic info
        self.cam_info = self.setCamInfo(self.calib_data)

        rospack = rospkg.RosPack()
        # Initialize an instance of Renderer giving the model in input.
        self.renderer = Renderer(rospack.get_path('augmented_reality_apriltag') + '/src/models/duckie.obj')
        self.log('Renderer object created')
        # Create AprilTag detector object
        self.at_detector = Detector()

        # Create cv_bridge
        self.bridge = CvBridge()

        # Define subscriber to recieve images
        self.image_sub = rospy.Subscriber('/' + self.veh+ '/camera_node/image/compressed', CompressedImage, self.callback)
        # Publish the rendered image to a new topic
        self.augmented_pub = rospy.Publisher('~image/compressed' , CompressedImage, queue_size=1)

        self.log(node_name + ' initialized and running')

    def callback(self, ros_image):
        '''
            MAIN TASK
            Converse the image from the camera_node to a cv2 image. Then, detects 
            the apriltags position and retrieves the homography. With the homography
            and the intrinsic camera matrix, it computes the projection matrix. Finally,
            the provided Renderer class returns an image with the model rendered on it,
            which is then published to a new topic
        '''
        # Convert to cv2 image
        image = self.readImage(ros_image)
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Extract camera parameters
        K = np.array(self.cam_info.K).reshape((3,3))
        cam_params = (K[0,0], K[1,1], K[0,2], K[1,2])

        # Detect apriltags
        detections = self.at_detector.detect(gray_image, estimate_tag_pose=True, camera_params=cam_params, tag_size=0.065)

        # Render a model in each of the detected Apriltags
        rendered_image = image
        for tag in detections:
            # Extract homography
            H = np.array(tag.homography).reshape((3,3))         

            # Obtain Projection matrix
            projection_matrix = self.projection_matrix(K, H)

            # Render the model
            rendered_image = self.renderer.render(rendered_image, projection_matrix)
        
        # Publish image with models rendered
        augmented_image = self.bridge.cv2_to_compressed_imgmsg(rendered_image)
        self.augmented_pub.publish(augmented_image) 

        


    def setCamInfo(self, calib_data):
        '''
            Introduces the camera information from the dictionary
            obtained reading the yaml file to a CameraInfo object
        ''' 
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = calib_data['camera_matrix']['data']
        cam_info.D = calib_data['distortion_coefficients']['data']
        cam_info.R = calib_data['rectification_matrix']['data']
        cam_info.P = calib_data['projection_matrix']['data']
        cam_info.distortion_model = calib_data['distortion_model']
        return cam_info  


    
    def projection_matrix(self, intrinsic, homography):
        """
            Method that computes a Projection matrix from 3D world to image plane.
            Args:
                intrinsic: (np.array) Camera matrix
                homography: (np.array) 2D Homography obtained by detecting the apriltag position
            Returns:
                P: Projeciton matrix

        """
        # Extract parameters
        K = intrinsic
        K_inv = np.linalg.inv(K)
        H = homography

        # Compute 2D Rt matrix
        Rt = K_inv @ H
        # Normalize so that ||R1|| = 1
        Rt = Rt / np.linalg.norm(Rt[:,0])
        t = np.array(Rt[:,2]).reshape((3,1))
        # Extract rotation vectors
        R1 = np.array(Rt[:,0]).reshape((3,1))
        R2 = np.array(Rt[:,1]).reshape((3,1))
        # Compute third rotation vector to be orthogonal to the other two
        R3 = np.array(np.cross(Rt[:,0],Rt[:,1])).reshape((3,1))
        # Construct 3D rotation matrix
        R = np.concatenate((R1,R2,R3),axis=1)
        # Perform polar decomposition to enforce orthogonality
        U, S, Vt = np.linalg.svd(R)
        R = U @ Vt
        # Compute projection matrix
        P = K @ (np.concatenate((R,t),axis=1))

        return P

    def readImage(self, msg_image):
        """
            Convert images to OpenCV images
            Args:
                msg_image (:obj:`CompressedImage`) the image from the camera node
            Returns:
                OpenCV image
        """
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg_image)
            return cv_image
        except CvBridgeError as e:
            self.log(e)
            return []

    def readYamlFile(self,fname):
        """
            Reads the 'fname' yaml file and returns a dictionary with its input.

            You will find the calibration files you need in:
            `/data/config/calibrations/`
        """
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
                return yaml_dict
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                         %(fname, exc), type='fatal')
                rospy.signal_shutdown()
                return


    def onShutdown(self):
        super(ARNode, self).onShutdown()


if __name__ == '__main__':
    # Initialize the node
    camera_node = ARNode(node_name='augmented_reality_apriltag_node')
    # Keep it spinning to keep the node alive
    rospy.spin()