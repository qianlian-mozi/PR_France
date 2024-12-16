#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from tf.transformations import euler_from_quaternion

class TurtleBotMover:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('move_turtlebot_with_video', anonymous=True)

        # Publisher to control the robot
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Publisher for the manipulated image
        self.image_pub = rospy.Publisher('/cv/image', Image, queue_size=10)

        # Create a subscriber to the video feed (camera)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_projected_compensated', Image, self.image_callback)

        # Create a publisher for the laser scan
        self.laser_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)

        # Create a laser scan message
        self.laser_scan = LaserScan()

        # Create a subscriber for the odometry
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Create a Odometry message
        self.odom = Odometry()

        # Create a subscriber for the map
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # Create a publisher to map
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)

        # Create a OccupancyGrid message
        self.map = OccupancyGrid()

        # Create a Twist message to move the robot
        self.move_cmd = Twist()

        # Set movement speed (forward)
        self.move_cmd.linear.x = 0.1  # meters per second
        self.move_cmd.angular.z = 0   # no turning

        # Define the time to move (1 foot = 0.3048 meters)
        self.distance_to_move = 0.3048  # meters
        self.speed = self.move_cmd.linear.x
        self.duration = self.distance_to_move / self.speed  # time = distance / speed

        # Set up a rate for the loop
        self.rate = rospy.Rate(10)

        # Camera intrinsic matrix, rotation matrix, and translation vector
        self.K = np.array([[156.2063,    0,          164.8726],
                           [0,           155.58648,  120.41539],
                           [0,           0,          1]])
        self.R = np.eye(3)
        self.T = np.array([0, 0, 0.1]).reshape(-1, 1)


    def odom_callback(self, data):
        self.odom = data

    def map_callback(self, data):
        self.map = data


    def image_callback(self, data):
        # Convert the ROS image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            rospy.logerr("Failed to convert image: %s", str(e))
            return

        # Get the height and width of the image
        height, width, _ = cv_image.shape

        # Convert the cropped region to HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define color ranges for yellow and white in HSV space
        
        # Yellow color range in HSV
        lower_yellow = np.array([20, 100, 100])  # Hue, Saturation, Value
        upper_yellow = np.array([40, 255, 255])
        
        # White color range in HSV
        lower_white = np.array([0, 0, 200])  # High value for brightness
        upper_white = np.array([180, 20, 255])  # Low saturation for white

        # Create masks for yellow and white colors in the lower half
        yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
        white_mask = cv2.inRange(hsv_image, lower_white, upper_white)

        # Combine both masks to get yellow and white lines
        mask = cv2.bitwise_or(yellow_mask, white_mask)

        # Find contours in the mask (edges of yellow and white lines)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw the contours on the original image (green color)
        cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 2)

        # find the distance of the detected lines from the center of the image
        distances = []
        for contour in contours:
            # Get the bounding rectangle of the contour
            x, y, w, h = cv2.boundingRect(contour)
            # Calculate the center of the bounding rectangle
            center = x + w // 2
            # Calculate the distance from the center of the image
            distance = center - width // 2
            distances.append(distance)

        # Show the distance of the detected lines from the center
        rospy.loginfo("Distances from the center: %s", str(distances))

        # Edit the map with the detected lines
        

        # Convert the manipulated image back to a ROS image message
        try:
            modified_image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        except Exception as e:
            rospy.logerr("Failed to convert manipulated image to msg: %s", str(e))
            return

        # Publish the manipulated image with detected lines on the lower half
        self.image_pub.publish(modified_image_msg)

        # Log info about the processed image
        rospy.loginfo("Published manipulated image with detected yellow and white lines.")


    def move_turtlebot(self):
        # Start moving the robot forward
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            # Publish the movement command to move forward
            self.pub.publish(self.move_cmd)

            # Check if the robot has moved the desired distance
            if rospy.Time.now() - start_time >= rospy.Duration(self.duration):
                break

            self.rate.sleep()

        # Stop the robot after moving
        self.move_cmd.linear.x = 0
        self.pub.publish(self.move_cmd)
        rospy.loginfo("TurtleBot moved 1 foot ahead!")

    def wait_for_exit(self):
        # Wait for the 'q' key press to exit using input() instead of OpenCV
        rospy.loginfo("Press 'q' and hit enter to exit the node.")
        
        # Loop until the user enters 'q' and presses Enter
        while not rospy.is_shutdown():
            user_input = input("Enter 'q' to quit: ").strip().lower()
            if user_input == 'q':  # If the 'q' key is entered, exit
                rospy.loginfo("Exiting...")
                break

            self.rate.sleep()

if __name__ == '__main__':
    try:
        
        turtlebot_mover = TurtleBotMover()
        # turtlebot_mover.move_turtlebot()
        turtlebot_mover.wait_for_exit()

    except rospy.ROSInterruptException:
        pass