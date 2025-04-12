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

    def choose_frontier(self, frontiers):
        robot_row, robot_col = self.robot_position
        frontier_distances = []

        for frontier in frontiers:
            if frontier in self.visited_frontiers:
                continue
            dist = np.sqrt((robot_row - frontier[0])**2 + (robot_col - frontier[1])**2)
            frontier_distances.append((dist, frontier))

        if not frontier_distances:
            self.get_logger().warning("No valid frontier found")
            return None

        frontier_distances.sort()
        closest_frontiers = [f for _, f in frontier_distances[:5]]  # Pick from 5 nearest

        chosen_frontier = random.choice(closest_frontiers)
        self.visited_frontiers.add(chosen_frontier)
        self.visited_positions.append((robot_row, robot_col))
        self.get_logger().info(f"Chosen random frontier among closest 5: {chosen_frontier}")

        return chosen_frontier

    def explore(self):
        if self.pause_requested:
            self.get_logger().info("Exploration is paused.")
            return

        if self.map_data is None:
            self.get_logger().warning("No map data available")
            return

        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width))

        frontiers = self.find_frontiers(map_array)

        if not frontiers:
            self.get_logger().info("No frontiers found. Backtracking...")
            if self.visited_positions:
                last_position = self.visited_positions.pop()
                self.navigate_to(last_position[0], last_position[1])
            else:
                self.get_logger().info("No previous positions to backtrack to.")
            return

        chosen_frontier = self.choose_frontier(frontiers)

        if not chosen_frontier:
            self.get_logger().warning("No frontiers to explore")
            return

        goal_x = chosen_frontier[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x
        goal_y = chosen_frontier[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y

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
