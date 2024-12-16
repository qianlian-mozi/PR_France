#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# import numpy as np
# import cv2
# from detect_lane import DetectLane
# import rospy
# from nav_msgs.msg import Odometry
# import math
# from tf.transformations import euler_from_quaternion
# from sensor_msgs.msg import Image, CompressedImage
# import tf
# from cv_bridge import CvBridge

# import rospy
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

# class Detected:
#     def __init__(self):
#         # 创建 CvBridge 实例
#         self.cvBridge = CvBridge()
#         # 订阅 ROS 图像话题
#         self.image_sub = rospy.Subscriber('/detect/image_input', Image, self.image_callback)

#     def image_callback(self, image_msg):
#         try:
#             # 将 ROS Image 消息转换为 OpenCV 图像
#             cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")
#             rospy.loginfo("Image converted successfully.")
#             # 在这里对 cv_image 进行进一步处理
#         except CvBridgeError as e:
#             rospy.logerr(f"CV Bridge Error: {e}")







    
#     Detected = DetectLane()

#     Detected.sub_image_type = 'raw'



# # if DetectLane.sub_image_type == "compressed":
# #     # subscribes compressed image
# #     sub_image_original = rospy.Subscriber('/detect/image_input/compressed', CompressedImage, DetectLane.cbFindLane, queue_size = 1)
# # elif DetectLane.sub_image_type == "raw":
# #     # subscribes raw image
# #     sub_image_original = rospy.Subscriber('/detect/image_input', Image, DetectLane.cbFindLane, queue_size = 1)
                    
# # # get image frame for this file
# # if DetectLane.sub_image_type == "compressed":
# #     #converting compressed image to opencv image
# #     np_arr = np.frombuffer(sub_image_original.data, np.uint8)
# #     cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
# # elif DetectLane.sub_image_type == "raw":
# #     cv_image = CvBridge.imgmsg_to_cv2(sub_image_original, "bgr8")
            
#     image = Detected.sub_image_original

#     if Detected.sub_image_type == "compressed":
#     #converting compressed image to opencv image
#         np_arr = np.frombuffer(image.data, np.uint8)
#         cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
#     elif Detected.sub_image_type == "raw":
#         cv_image = Detected.cvBridge.imgmsg_to_cv2(image, "bgr8")

#     K = np.array(
#         [[156.2063, 0, 164.8726],[0, 155.58648, 120.41539],[0,  0,  1.]]
#     )

#     R = np.eye(3)
#     t = np.array([0,0,0.1]).reshape(-1,1)

#     _,_,yellow_coords = Detected.maskYellowLane(cv_image)
#     _,_,white_coords = Detected.maskWhiteLane(cv_image)

#     N=100
#     simplified_white_coords = white_coords[::N]
#     simplified_yellow_coords = yellow_coords[::N]
#     points_img_white = np.array([[x, y, 1] for x, y in simplified_white_coords]) 
#     points_img_yellow = np.array([[x, y, 1] for x, y in simplified_yellow_coords])

#     # 转换为地面坐标
#     points_real_white = np.linalg.inv(K) @ (R @ points_img_white.T + t)
#     points_real_white /= points_real_white[2]  # 齐次坐标归一化
#     points_real_yellow = np.linalg.inv(K) @ (R @ points_img_yellow.T + t)
#     points_real_yellow /= points_real_yellow[2]  # 齐次坐标归一化

#     def odometry_callback(msg):
#         global q
#         pose = msg.pose.pose
#         orientation_q = pose.orientation
#         _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
#         q = [pose.position.x, pose.position.y, yaw]

#     rospy.Subscriber('/odom', Odometry, odometry_callback)

#     # 机器人在地面上的位姿
#     x_r, y_r, theta_r = q  # 来自 /odom

#     # 变换矩阵
#     T_robot_to_world = np.array([
#         [math.cos(theta_r), -math.sin(theta_r), x_r],
#         [math.sin(theta_r),  math.cos(theta_r), y_r],
#         [0,                 0,                 1]
#     ])

#     # 将世界坐标转换到机器人坐标系
#     T_world_to_robot = np.linalg.inv(T_robot_to_world)
#     points_robot_white = (T_world_to_robot @ points_real_white.T).T
#     points_robot_yellow = (T_world_to_robot @ points_real_yellow.T).T


#     def get_global_pose():
#         listener = tf.TransformListener()

#         try:
#             listener.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(3.0))
#             (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
#             x, y, _ = trans
#             roll, pitch, yaw = euler_from_quaternion(rot)
#             return x, y, yaw
#         except tf.Exception as e:
#             rospy.logerr(f"Failed to get transform: {e}")
#             return None
        
#     slam_pose = get_global_pose()
#     x_s, y_s, theta_s = slam_pose  # 来自 SLAM 系统

#     # SLAM 地图坐标变换矩阵
#     T_robot_to_map = np.array([
#         [math.cos(theta_s), -math.sin(theta_s), x_s],
#         [math.sin(theta_s),  math.cos(theta_s), y_s],
#         [0,                 0,                 1]
#     ])

#     # 转换为地图坐标系
#     points_map_white = (T_robot_to_map @ points_robot_white.T).T
#     points_map_yellow = (T_robot_to_map @ points_robot_yellow.T).T

#     # 提取 x, y 坐标
#     x_map_white, y_map_white = points_map_white[:, 0], points_map_white[:, 1]
#     x_map_yellow, y_map_yellow = points_map_yellow[:, 0], points_map_yellow[:, 1]

#     print("x_map_white, y_map_white",x_map_white,y_map_white)


# if __name__ == "__main__":
#     rospy.init_node("coordinates", anonymous=True)
#     detected = Detected()
#     rospy.spin()





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
import json

class LaneDetector:
    def __init__(self):
        # rospy.init_node("lane_detector", anonymous=True)

        # 创建 CvBridge 实例
        self.cvBridge = CvBridge()

        # 订阅图像和里程计数据
        self.image_sub = rospy.Subscriber('/camera/image_projected_compensated', Image, self.image_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odometry_callback)

        # 初始化检测器和其他变量
        self.detect_lane = DetectLane()
        self.detect_lane.sub_image_type = 'raw'
        self.q = [0, 0, 0]  # 机器人位姿初始化

    def image_callback(self, image_msg):
        try:
            # 将 ROS Image 消息转换为 OpenCV 图像
            print("comes to image_callback")
            cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")
            print("comes to process_image-----------------------")
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

    def process_image(self, cv_image):
    # 相机内参矩阵
        K = np.array(
            [[ -3.3130611,0, 160.5], [0, -3.3130611 , 120.5], [0, 0, 1.]]
        )

        # 相机外参（假设已知）
        R = np.eye(3)
        t = np.array([0, 0, 0.1]).reshape(-1, 1)

        try:
            valid_yellow, _, yellow_coords = self.detect_lane.maskYellowLane(cv_image)
            valid_white, _, white_coords = self.detect_lane.maskWhiteLane(cv_image)

            if not valid_yellow or not valid_white:
                rospy.logwarn("Lane detection returned invalid data.")
                return

            # print("Yellow lane coordinates:", yellow_coords)
            # print("White lane coordinates:", white_coords)

            # 数据简化
            N = 100
            simplified_white_coords = white_coords[::N] if white_coords else []
            simplified_yellow_coords = yellow_coords[::N] if yellow_coords else []

            if not simplified_white_coords or not simplified_yellow_coords:
                rospy.logwarn("Simplified coordinates are empty.")
                return

            # 后续处理...
        except Exception as e:
            rospy.logerr(f"Error during lane processing: {e}")
            return

        points_img_white = np.array([[x, y, 1] for x, y in simplified_white_coords])
        points_img_yellow = np.array([[x, y, 1] for x, y in simplified_yellow_coords])

        # 图像坐标转换到地面坐标
        points_real_white = np.linalg.inv(K) @ (R @ points_img_white.T + t)
        points_real_white /= points_real_white[2, :]  # 齐次坐标归一化
        points_real_white = points_real_white.T  # 转置为 N x 3

        points_real_yellow = np.linalg.inv(K) @ (R @ points_img_yellow.T + t)
        points_real_yellow /= points_real_yellow[2, :]
        points_real_yellow = points_real_yellow.T

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
        
        slam_pose = self.get_global_pose()
        x_s, y_s, theta_s = slam_pose  # 来自 SLAM 系统

    # SLAM 地图坐标变换矩阵
        T_robot_to_map = np.array([
            [math.cos(theta_s), -math.sin(theta_s), x_s],
            [math.sin(theta_s),  math.cos(theta_s), y_s],
            [0,                 0,                 1]
        ])

        # 转换为地图坐标系
        points_map_white = (T_robot_to_map @ points_robot_white.T).T
        points_map_yellow = (T_robot_to_map @ points_robot_yellow.T).T

        # 提取 x, y 坐标
        x_map_white, y_map_white = points_map_white[:, 0], points_map_white[:, 1]
        x_map_yellow, y_map_yellow = points_map_yellow[:, 0], points_map_yellow[:, 1]
        
        

        # 打印调试信息
        print("points_robot_white:", points_map_white)
        
        # save points to file
        self.save_points_to("/home/yirenqiu/turtlebot3_ws/points.txt", zip(x_map_white, y_map_white), zip(x_map_yellow, y_map_yellow))

    
    # def save_points_to(filename, points):
    #     """
    #     将地图点保存为 JSON 文件
    #     :param filename: 保存文件的路径
    #     :param points: 要保存的点，格式为 [(x1, y1), (x2, y2), ...]
    #     """
    #     with open(filename, 'w') as file:
    #         json.dump({'map_points': points}, file, indent=4)
    #     print(f"Points saved to {filename}")
    def save_points_to(self, filename, points_white, points_yellow):
        """
        将地图点保存为文本文件
        :param filename: 保存文件的路径
        :param points_white: 白色车道的点，格式为 [(x1, y1), (x2, y2), ...]
        :param points_yellow: 黄色车道的点，格式为 [(x1, y1), (x2, y2), ...]
        """
        with open(filename, 'w') as file:
            # 写入白色车道点
            file.write("White lane points:\n")
            for point in points_white:
                file.write(f"{point[0]}, {point[1]}\n")
            
            # 写入黄色车道点
            file.write("\nYellow lane points:\n")
            for point in points_yellow:
                file.write(f"{point[0]}, {point[1]}\n")

        print(f"Points saved to {filename}")
    
    def main(self):
        rospy.spin()
        
        
if __name__ == "__main__":
    rospy.init_node("lane_detector", anonymous=True)
    ins = LaneDetector()
    ins.main()


