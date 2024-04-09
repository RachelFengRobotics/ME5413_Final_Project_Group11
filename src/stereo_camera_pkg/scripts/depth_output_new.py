#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Float32

class StereoDepthNode:
    def __init__(self):
        rospy.init_node('stereo_depth_node')

        self.bridge = CvBridge()
        self.left_image_sub = rospy.Subscriber("/front/left/image_raw", Image, self.left_image_callback)  #接收左右两张图片
        self.right_image_sub = rospy.Subscriber("/front/right/image_raw", Image, self.right_image_callback)
        self.point_sub = rospy.Subscriber("/me5413/point", Point, self.point_callback)  #需要计算深度信息的目标点

        self.depth_pub = rospy.Publisher("/stereo/depth", Float32, queue_size=10)

        self.left_image = None
        self.right_image = None
        self.query_points = []

        self.setup_ui()
        self.setup_camera_parameters()

    def setup_ui(self):
        cv2.namedWindow("depth")
        cv2.moveWindow('depth', 600, 0)
        cv2.createTrackbar('num', 'depth', 3, 10, lambda x: None)
        cv2.createTrackbar('blockSize', 'depth', 7, 255, lambda x: None)
        cv2.setMouseCallback("depth", self.callbackFunc, None)

    def setup_camera_parameters(self):
        self.left_camera_matrix = np.array([[554.25469119, 0., 320.5],
                                            [0., 554.25469119, 256.5],
                                            [0., 0., 1.]])
        self.left_distortion = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        self.right_camera_matrix = np.array([[554.25469119, 0., 320.5],
                                             [0., 554.25469119, 256.5],
                                             [0., 0., 1.]])
        self.right_distortion = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        R = np.array([[1.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0],
                      [0.0, 0.0, 1.0]])
        T = np.array([0.320, 0.0, 0.0])
        size = (640, 512)

        _, _, _, _, self.Q, _, _ = cv2.stereoRectify(self.left_camera_matrix, self.left_distortion,
                                                     self.right_camera_matrix, self.right_distortion,
                                                     size, R, T)

    def left_image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
            self.left_image = cv2.pyrDown(cv_image)
        except CvBridgeError as e:
            rospy.logerr(e)

    def right_image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
            self.right_image = cv2.pyrDown(cv_image)
        except CvBridgeError as e:
            rospy.logerr(e)

    def point_callback(self, msg):
        self.query_points = [(int(msg.x), int(msg.y))]

    def callbackFunc(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            rospy.loginfo("Mouse Event at {},{} - Event: {}".format(x, y, event))
            # Additional functionality can be added here


    def process_images(self):
        if self.left_image is not None and self.right_image is not None:
        # Adjust parameters using trackbars
            num = cv2.getTrackbarPos("num", "depth")
            blockSize = cv2.getTrackbarPos("blockSize", "depth")
        if blockSize % 2 == 0:
            blockSize += 1
        if blockSize < 5:
            blockSize = 5

        # Use StereoBM with adjusted parameters
        stereo = cv2.StereoBM_create(numDisparities=16*num, blockSize=blockSize)
        disp = stereo.compute(self.left_image, self.right_image)
        disp = cv2.normalize(disp, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

        # Apply mean filtering to the disparity map
        kernel_size = 40  # Kernel size can be adjusted
        disp_filtered = cv2.blur(disp, (kernel_size, kernel_size))

        # Recalculate 3D point cloud
        threeD = cv2.reprojectImageTo3D(disp_filtered.astype(np.float32) / 16., self.Q)

        # Display depth for queried points
        for (x, y) in self.query_points:
            if 0 <= x < threeD.shape[1] and 0 <= y < threeD.shape[0]:
                depth = threeD[y, x, 2]
                rospy.loginfo("Depth at ({}, {}): {:.2f}".format(x, y, depth))
                self.depth_pub.publish(depth)
            else:
                rospy.loginfo("Point ({}, {}) is out of bounds.".format(x, y))
    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.process_images()
            rate.sleep()

if __name__ == '__main__':
    node = StereoDepthNode()
    node.run()