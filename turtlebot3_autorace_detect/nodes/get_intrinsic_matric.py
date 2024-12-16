#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import CameraInfo
import numpy as np

class CameraInfoSubscriber:
    def __init__(self):
        self.camera_info = None
        rospy.Subscriber('/camera/camera_info', CameraInfo, self.camera_info_callback)

    def camera_info_callback(self, msg):
        self.camera_info = msg
        # 获取内参
        K = np.array(msg.K).reshape(3, 3)  # Camera matrix
        print("Camera matrix K:", K)

if __name__ == "__main__":
    rospy.init_node("camera_info_subscriber")
    camera_info_subscriber = CameraInfoSubscriber()
    rospy.spin()
