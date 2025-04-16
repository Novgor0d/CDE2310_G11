import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

SERVO_PIN = 32
GPIO.setup(SERVO_PIN, GPIO.OUT)

pwm_servo = GPIO.PWM(SERVO_PIN, 50)
pwm_servo.start(0)

try:
    # Move to 0 degrees
    duty = 2 + (0 / 18)
    pwm_servo.ChangeDutyCycle(duty)
    time.sleep(0.4)  # Let servo reach position

    pwm_servo.ChangeDutyCycle(0)
    time.sleep(0.3)  # Small pause before cleanup

finally:
    pwm_servo.stop()
    GPIO.cleanup()
