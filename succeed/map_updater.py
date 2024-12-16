import rospy
import tf
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry

class MergedMapPublisher:
    def __init__(self):
        rospy.init_node('merged_map_publisher')

        # Subscribers for /map and /odom
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Publisher for the merged map (published back to /map)
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)

        # Persistent map for lines
        self.persistent_map = None

        # Robot pose
        self.robot_pose = None

    def map_callback(self, data):
        # If no persistent map exists, initialize it as a copy of /map
        if self.persistent_map is None:
            self.persistent_map = OccupancyGrid()
            self.persistent_map.header = data.header
            self.persistent_map.info = data.info
            self.persistent_map.data = list(data.data)  # Make a copy

        # Merge incoming map with persistent modifications
        self.merge_maps(data)

    def odom_callback(self, data):
        # Get robot position and orientation from odometry
        position = data.pose.pose.position
        orientation = data.pose.pose.orientation

        # Convert orientation (quaternion) to yaw
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )

        # Store robot pose as (x, y, yaw)
        self.robot_pose = (position.x, position.y, yaw)

    def add_lines_to_map(self, map_data):
        if self.robot_pose is None:
            return map_data

        # Extract map info
        resolution = self.persistent_map.info.resolution
        width = self.persistent_map.info.width
        height = self.persistent_map.info.height
        origin = self.persistent_map.info.origin

        # Get the robot's pose in the map frame
        robot_x, robot_y, robot_yaw = self.robot_pose

        # Convert robot position to map grid indices
        robot_grid_x = int((robot_x - origin.position.x) / resolution)
        robot_grid_y = int((robot_y - origin.position.y) / resolution)

        # Define line parameters
        line_length = 2  # Length of the line (in cells) above and below the robot
        line_value = 100  # Occupied value for obstacles
        side_offset = 3  # Distance (in cells) to place lines on the left/right of the robot

        # Calculate offsets for the lines
        left_offset_x = side_offset * np.cos(robot_yaw + np.pi / 2)
        left_offset_y = side_offset * np.sin(robot_yaw + np.pi / 2)
        right_offset_x = side_offset * np.cos(robot_yaw - np.pi / 2)
        right_offset_y = side_offset * np.sin(robot_yaw - np.pi / 2)

        # Draw the vertical lines for both sides
        updated_map = np.array(map_data).reshape((height, width))
        for i in range(-line_length, line_length + 1):
            # Vertical offset (aligned with the robot's forward direction)
            vertical_x_offset = i * np.cos(robot_yaw)
            vertical_y_offset = i * np.sin(robot_yaw)

            # Left line position
            left_x = robot_grid_x + int(left_offset_x + vertical_x_offset)
            left_y = robot_grid_y + int(left_offset_y + vertical_y_offset)

            # Right line position
            right_x = robot_grid_x + int(right_offset_x + vertical_x_offset)
            right_y = robot_grid_y + int(right_offset_y + vertical_y_offset)

            # Check bounds and update map data
            if 0 <= left_x < width and 0 <= left_y < height:
                updated_map[left_y, left_x] = line_value
            if 0 <= right_x < width and 0 <= right_y < height:
                updated_map[right_y, right_x] = line_value

        return updated_map.flatten().tolist()

    def merge_maps(self, incoming_map):
        # Convert incoming map and persistent map data to numpy arrays
        incoming_data = np.array(incoming_map.data)
        persistent_data = np.array(self.persistent_map.data)

        # Retain Gmapping updates (where occupancy != 0) and reapply lines
        updated_map_data = np.where(incoming_data != 0, incoming_data, persistent_data)
        updated_map_data = self.add_lines_to_map(updated_map_data)

        # Update persistent map with new data
        self.persistent_map.data = list(updated_map_data)

        # Publish the merged map
        self.persistent_map.header = incoming_map.header
        self.map_pub.publish(self.persistent_map)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz update rate
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    node = MergedMapPublisher()
    node.run()