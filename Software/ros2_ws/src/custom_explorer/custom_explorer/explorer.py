import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, GoalResponse, CancelResponse
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

        self.map_data = None
        self.visited_frontiers = set()
        self.pause_requested = False

        self.exploration_mode = 'frontier'
        self.start_time = self.get_clock().now()

        self.timer = self.create_timer(10.0, self.explore)
        self.mode_timer = self.create_timer(1.0, self.update_mode)
        self.active_goal_handle = None

    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info("Map received")

    def fire_callback(self, msg):
        if msg.data and not self.pause_requested:
            self.get_logger().info("Received fire flare pause signal. Pausing for 20 seconds.")
            self.pause_requested = True

            # Cancel current navigation goal if any
            if self.active_goal_handle is not None:
                self.get_logger().info("Cancelling active navigation goal...")
                cancel_future = self.active_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(lambda f: self.get_logger().info("Navigation goal cancelled."))

            # Stop robot movement
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)

            # Start pause thread
            threading.Thread(target=self.pause_exploration).start()

    def pause_exploration(self):
        time.sleep(20)
        self.get_logger().info("Resuming exploration after pause.")
        self.pause_requested = False

    def navigate_to(self, x, y):
        if self.pause_requested:
            self.get_logger().info("Navigation skipped due to pause.")
            return

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
        self.active_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_complete_callback)

    def navigation_complete_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f"Navigation completed with result: {result}")
        except Exception as e:
            self.get_logger().error(f"Navigation failed: {e}")
        finally:
            self.active_goal_handle = None

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
        rows, cols = map_array.shape
        random_row, random_col = random.randint(0, rows - 1), random.randint(0, cols - 1)

        while map_array[random_row, random_col] != 0 or (random_row, random_col) in self.visited_frontiers:
            random_row, random_col = random.randint(0, rows - 1), random.randint(0, cols - 1)

        self.visited_frontiers.add((random_row, random_col))
        self.get_logger().info(f"Chosen random point: ({random_row}, {random_col})")
        return random_row, random_col

    def choose_wall_point(self, map_array):
        rows, cols = map_array.shape
        wall_points = []

        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if map_array[r, c] == 0:
                    neighbors = map_array[r-1:r+2, c-1:c+2].flatten()
                    if 100 in neighbors:
                        wall_points.append((r, c))

        if not wall_points:
            return self.choose_random_point(map_array)

        point = random.choice(wall_points)
        self.visited_frontiers.add(point)
        self.get_logger().info(f"Chosen wall-following point: {point}")
        return point

    def explore(self):
        if self.pause_requested or self.map_data is None:
            return

        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width))

        if self.exploration_mode == 'frontier':
            frontiers = self.find_frontiers(map_array)
            if not frontiers:
                self.get_logger().info("No frontiers found, switching to random.")
                point = self.choose_random_point(map_array)
            else:
                point = random.choice(frontiers)
        else:
            point = self.choose_random_point(map_array)

        goal_x = point[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x
        goal_y = point[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y

        self.navigate_to(goal_x, goal_y)

    def update_mode(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if elapsed < 240:
            self.exploration_mode = 'frontier'
        else:
            # Time since switching phase (after initial 4 min)
            time_since_phase = elapsed - 240
            # Check which 2-minute phase we're in
            phase_number = int(time_since_phase // 120)

            if phase_number % 2 == 0:
                self.exploration_mode = 'random'
            else:
                self.exploration_mode = 'frontier'

        self.get_logger().info(f"Exploration mode: {self.exploration_mode}")



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