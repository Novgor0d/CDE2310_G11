import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Bool
import numpy as np
import time
import threading
import random


class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info("Explorer Node Started")

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.fire_sub = self.create_subscription(Bool, '/fire_flare', self.fire_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.visited_frontiers = set()
        self.visited_positions = []

        self.map_data = None
        self.robot_position = (0, 0)

        self.pause_requested = False
        self.auto_nav_started = False

        self.timer = self.create_timer(5.0, self.explore)

    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info("Map received")

    def fire_callback(self, msg):
        if msg.data:
            self.get_logger().info("Received fire flare signal. Pausing for 20 seconds.")
            self.pause_requested = True
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            threading.Thread(target=self.pause_exploration).start()

    def pause_exploration(self):
        time.sleep(20)
        self.get_logger().info("Resuming exploration after pause.")
        self.pause_requested = False

    def navigate_to(self, x, y):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.w = 1.0

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg

        self.get_logger().info(f"Navigating to goal: x={x}, y={y}")

        self.nav_to_pose_client.wait_for_server()
        send_goal_future = self.nav_to_pose_client.send_goal_async(nav_goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Goal rejected!")
            return

        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_complete_callback)

    def navigation_complete_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f"Navigation completed with result: {result}")
        except Exception as e:
            self.get_logger().error(f"Navigation failed: {e}")

    def find_frontiers(self, map_array):
        frontiers = []
        rows, cols = map_array.shape

        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if map_array[r, c] == 0:
                    neighbors = map_array[r-1:r+2, c-1:c+2].flatten()
                    if -1 in neighbors:
                        frontiers.append((r, c))

        self.get_logger().info(f"Found {len(frontiers)} frontiers")
        return frontiers

    def choose_random_point(self, map_array):
        """Choose a random point on the map that is unexplored."""
        rows, cols = map_array.shape
        random_row, random_col = random.randint(0, rows - 1), random.randint(0, cols - 1)

        # Ensure the point is free space (0) and not visited
        while map_array[random_row, random_col] != 0 or (random_row, random_col) in self.visited_frontiers:
            random_row, random_col = random.randint(0, rows - 1), random.randint(0, cols - 1)

        self.visited_frontiers.add((random_row, random_col))
        self.get_logger().info(f"Chosen random point: ({random_row}, {random_col})")
        return random_row, random_col

    def explore(self):
        if self.pause_requested:
            self.get_logger().info("Exploration is paused.")
            return

        if self.map_data is None:
            self.get_logger().warning("No map data available")
            return

        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width))

        # Choose a random point to navigate to
        random_point = self.choose_random_point(map_array)

        if not random_point:
            self.get_logger().warning("No valid point found to explore")
            return

        goal_x = random_point[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x
        goal_y = random_point[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y

        self.navigate_to(goal_x, goal_y)


def main(args=None):
    rclpy.init(args=args)
    explorer_node = ExplorerNode()

    try:
        explorer_node.get_logger().info("Starting exploration...")
        rclpy.spin(explorer_node)
    except KeyboardInterrupt:
        explorer_node.get_logger().info("Exploration stopped by user")
    finally:
        explorer_node.destroy_node()
        rclpy.shutdown()
