#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import cv2
from detect_lane import DetectLane
import rospy
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Image, CompressedImage
import tf
from cv_bridge import CvBridge, CvBridgeError

class LaneDetector:
    def __init__(self):
        rospy.init_node("lane_detector", anonymous=True)

        # 创建 CvBridge 实例
        self.cvBridge = CvBridge()

        # 订阅图像和里程计数据
        self.image_sub = rospy.Subscriber('/detect/image_input', Image, self.image_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odometry_callback)

        # 初始化检测器和其他变量
        self.detect_lane = DetectLane()
        self.detect_lane.sub_image_type = 'raw'
        self.q = [0, 0, 0]  # 机器人位姿初始化

    def image_callback(self, image_msg):
        try:
            # 将 ROS Image 消息转换为 OpenCV 图像
            cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")

            # 图像处理和坐标变换
            self.process_image(cv_image)

        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")

    def odometry_callback(self, msg):
        # 更新机器人位姿
        pose = msg.pose.pose
        orientation_q = pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.q = [pose.position.x, pose.position.y, yaw]

    def process_image(self, cv_image):
        # 相机内参矩阵
        K = np.array(
            [[156.2063, 0, 164.8726], [0, 155.58648, 120.41539], [0, 0, 1.]]
        )

        # 相机外参（假设已知）
        R = np.eye(3)
        t = np.array([0, 0, 0.1]).reshape(-1, 1)

        # 车道线检测
        _, _, yellow_coords = self.detect_lane.maskYellowLane(cv_image)
        _, _, white_coords = self.detect_lane.maskWhiteLane(cv_image)

        # 数据简化
        N = 100
        simplified_white_coords = white_coords[::N]
        simplified_yellow_coords = yellow_coords[::N]

        points_img_white = np.array([[x, y, 1] for x, y in simplified_white_coords])
        points_img_yellow = np.array([[x, y, 1] for x, y in simplified_yellow_coords])

        # 图像坐标转换到地面坐标
        points_real_white = np.linalg.inv(K) @ (R @ points_img_white.T + t)
        points_real_white /= points_real_white[2]  # 齐次坐标归一化
        points_real_yellow = np.linalg.inv(K) @ (R @ points_img_yellow.T + t)
        points_real_yellow /= points_real_yellow[2]

        # 机器人坐标系到世界坐标系的变换
        x_r, y_r, theta_r = self.q
        T_robot_to_world = np.array([
            [math.cos(theta_r), -math.sin(theta_r), x_r],
            [math.sin(theta_r), math.cos(theta_r), y_r],
            [0, 0, 1]
        ])
        T_world_to_robot = np.linalg.inv(T_robot_to_world)

        # 将地面坐标转换到机器人坐标系
        points_robot_white = (T_world_to_robot @ points_real_white.T).T
        points_robot_yellow = (T_world_to_robot @ points_real_yellow.T).T

        # 获取 SLAM 坐标系位姿并转换
        slam_pose = self.get_global_pose()
        if slam_pose is not None:
            x_s, y_s, theta_s = slam_pose
            T_robot_to_map = np.array([
                [math.cos(theta_s), -math.sin(theta_s), x_s],
                [math.sin(theta_s), math.cos(theta_s), y_s],
                [0, 0, 1]
            ])
            points_map_white = (T_robot_to_map @ points_robot_white.T).T
            points_map_yellow = (T_robot_to_map @ points_robot_yellow.T).T

            # 提取 x, y 坐标并打印
            x_map_white, y_map_white = points_map_white[:, 0], points_map_white[:, 1]
            x_map_yellow, y_map_yellow = points_map_yellow[:, 0], points_map_yellow[:, 1]

            print("x_map_white, y_map_white", x_map_white, y_map_white)

    def get_global_pose(self):
        listener = tf.TransformListener()
        try:
            listener.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(3.0))
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            x, y, _ = trans
            roll, pitch, yaw = euler_from_quaternion(rot)
            return x, y, yaw
        except tf.Exception as e:
            rospy.logerr(f"Failed to get transform: {e}")
            return None

def main():
    detector = LaneDetector()
    rospy.spin()

if __name__ == "__main__":
    main()
