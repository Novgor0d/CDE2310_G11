import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import RPi.GPIO as GPIO
import time

# Raspberry Pi GPIO setup
GPIO.setmode(GPIO.BOARD)

FLYWHEEL = 13
ENB = 15
SERVO_PIN = 32

GPIO.setup([FLYWHEEL, ENB, SERVO_PIN], GPIO.OUT)

# Flywheel PWM setup
pwm_flywheel = GPIO.PWM(ENB, 1000)
pwm_flywheel.start(100)

# Servo PWM setup
pwm_servo = GPIO.PWM(SERVO_PIN, 50)
pwm_servo.start(0)

class FlareExplorerNode(Node):
    def __init__(self):
        super().__init__('flare_explorer_node')

        # Subscriber to fire flare signal
        self.fire_sub = self.create_subscription(
            Bool, '/fire_flare', self.fire_callback, 10)

    def fire_callback(self, msg):
        if msg.data:
            self.get_logger().info("Received 'fire!' signal.")
            self._fire_flare()

    def _fire_flare(self):
        """Start flywheel, reload, and fire flare three times."""
        print("Starting Flywheel ...")
        self.start_flywheel()
        time.sleep(3)  # spin-up time

        # Fire the flare 3 times with correct intervals
        for i in range(3):
            print(f"Firing {i+1} ...")
            self.reload()

            if i == 0:  # After the first reload, wait 2.5 seconds
                time.sleep(2.5)
            elif i == 1:  # After the second reload, wait 0.5 seconds
                time.sleep(0.5)

        print("Stopping Flywheel ...")
        self.stop_flywheel()

    def start_flywheel(self):
        GPIO.output(FLYWHEEL, GPIO.HIGH)

    def stop_flywheel(self):
        GPIO.output(FLYWHEEL, GPIO.LOW)

    def reload(self):
        """Reloads by moving the servo."""
        for angle in [0, 180, 0]:
            duty = 2 + (angle / 18)
            pwm_servo.ChangeDutyCycle(duty)
            time.sleep(0.5)  # faster movement
        pwm_servo.ChangeDutyCycle(0)
        time.sleep(0.3)  # small settling time

def main(args=None):
    rclpy.init(args=args)

    flare_explorer_node = FlareExplorerNode()

    rclpy.spin(flare_explorer_node)

    # Shutdown ROS2 after the node is done
    flare_explorer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
