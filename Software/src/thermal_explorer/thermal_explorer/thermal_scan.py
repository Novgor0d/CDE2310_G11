import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time
import busio
import board
import adafruit_amg88xx
import RPi.GPIO as GPIO
import math

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

class ThermalDetectorNode(Node):
    def __init__(self):
        super().__init__('thermal_detector')

        self.publisher_ = self.create_publisher(Bool, '/fire_flare', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.THRESHOLD = 27.0  # Celsius
        self.MAX_TEMP_TRIGGER = 29.5  # Celsius
        self.detected_locations = []
        self.detected_count = 0
        self.current_position = (0.0, 0.0)
        self.current_yaw = 0.0
        self.detecting = True

        # Set up I2C and thermal sensor
        i2c_bus = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_amg88xx.AMG88XX(i2c_bus)

        # Set up GPIO and servo motor
        self.servo_pin = 13
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.servo_pin, 50)  # 50 Hz
        self.pwm.start(0)

        # Timer to perform sweep and detection every second
        self.timer = self.create_timer(1.0, self.sweep_and_detect)

    def angle_to_duty_cycle(self, angle):
        return 2.5 + (angle / 180.0) * 10.0

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.current_position = (position.x, position.y)
        _, _, self.current_yaw = euler_from_quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w)

    def is_near_existing_source(self, x, y, threshold=0.5):
        for px, py in self.detected_locations:
            if math.sqrt((x - px) ** 2 + (y - py) ** 2) < threshold:
                return True
        return False

    def move_forward_safely(self, max_duration=5.0):
        twist = Twist()
        speed = 0.03  # slower forward speed
        twist.linear.x = speed
        start_time = self.get_clock().now().seconds_nanoseconds()[0]

        self.cmd_vel_pub.publish(twist)

        while rclpy.ok():
            current_time = self.get_clock().now().seconds_nanoseconds()[0]
            if current_time - start_time >= max_duration:
                break

            pixel_array = self.sensor.pixels
            high_temp_count = sum(1 for row in pixel_array for temp in row if temp > 30.0)

            if high_temp_count > 10:
                self.get_logger().info(f"Too hot ahead: {high_temp_count} pixels > 30°C. Stopping.")
                break

            time.sleep(0.1)

        # Stop the robot
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)

    def sweep_and_detect(self):
        if not self.detecting:
            return

        pixel_array = self.sensor.pixels

        self.get_logger().info("Thermal array:")
        for row in pixel_array:
            self.get_logger().info(str(["{:.1f}".format(temp) for temp in row]))

        count = sum(1 for row in pixel_array for temp in row if temp > self.THRESHOLD)
        max_temp = max(max(row) for row in pixel_array)
        self.get_logger().info(f"Max temperature detected: {max_temp:.1f}°C")

        x, y = self.current_position

        if max_temp > self.MAX_TEMP_TRIGGER and not self.is_near_existing_source(x, y):
            self.get_logger().info(f"Max temp {max_temp:.1f} > {self.MAX_TEMP_TRIGGER}, firing and moving...")

            # Log location to prevent duplicate triggering
            self.detected_locations.append((x, y))
            self.detected_count += 1

            # Fire flare once
            msg = Bool()
            msg.data = True
            self.publisher_.publish(msg)

            # Move forward carefully
            self.move_forward_safely()

            if self.detected_count >= 2:
                self.get_logger().info("Detected 2 distinct sources. Stopping detection.")
                self.cleanup()
                self.detecting = False

        elif count > 2 and not self.is_near_existing_source(x, y):
            self.get_logger().info(f'Heat source detected at ({x:.2f}, {y:.2f})')
            self.detected_locations.append((x, y))
            self.detected_count += 1

            msg = Bool()
            msg.data = True
            self.publisher_.publish(msg)

            if self.detected_count >= 2:
                self.get_logger().info("Detected 2 distinct sources. Stopping detection.")
                self.cleanup()
                self.detecting = False

    def cleanup(self):
        self.pwm.ChangeDutyCycle(0)
        time.sleep(0.5)
        self.pwm.stop()
        GPIO.cleanup()
        self.get_logger().info("GPIO and PWM cleaned up.")

def main(args=None):
    rclpy.init(args=args)
    node = ThermalDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
