import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import time
import busio
import board
import adafruit_amg88xx
import RPi.GPIO as GPIO


class ThermalDetectorNode(Node):
    def __init__(self):
        super().__init__('thermal_detector')

        # Create a publisher with QoS profile
        self.publisher_ = self.create_publisher(Bool, '/fire_flare', 10)

        self.THRESHOLD = 31.0  # Celsius

        # Initialize I2C and AMG88XX
        i2c_bus = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_amg88xx.AMG88XX(i2c_bus)
        time.sleep(0.1)  # Sensor warm-up

        # Set up GPIO for servo motor
        self.servo_pin = 13
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.servo_pin, 50)  # 50 Hz
        self.pwm.start(0)

        # Sweep after slight delay
        self.timer = self.create_timer(1.0, self.sweep_and_detect)

    def angle_to_duty_cycle(self, angle):
        return 2.5 + (angle / 180.0) * 10.0

    def sweep_and_detect(self):
        start_angle = 30
        end_angle = 120
        step = 2

        for angle in range(start_angle, end_angle + 1, step):
            duty = self.angle_to_duty_cycle(angle)
            self.pwm.ChangeDutyCycle(duty)
            time.sleep(0.05)

            count = sum(
                1 for row in self.sensor.pixels for temp in row if temp > self.THRESHOLD
            )

            if count > 40:
                self.get_logger().info('Heat source detected!')
                msg = Bool()
                msg.data = True
                self.publisher_.publish(msg)
                self.cleanup()
                return

        self.get_logger().info("Sweep complete. No significant heat source found.")
        self.cleanup()

    def cleanup(self):
        self.pwm.ChangeDutyCycle(0)
        time.sleep(0.5)
        self.pwm.stop()
        GPIO.cleanup()
        self.get_logger().info("GPIO and PWM cleaned up.")
        self.destroy_node()  # End the node after task


def main(args=None):
    rclpy.init(args=args)
    node = ThermalDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
