import RPi.GPIO as GPIO
import time

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

# Functions
def start_flywheel():
    GPIO.output(FLYWHEEL, GPIO.HIGH)

def stop_flywheel():
    GPIO.output(FLYWHEEL, GPIO.LOW)

def reload():
    for angle in [0, 180, 0]:
        duty = 2 + (angle / 18)
        pwm_servo.ChangeDutyCycle(duty)
        time.sleep(0.5)  # faster movement
    pwm_servo.ChangeDutyCycle(0)
    time.sleep(0.3)  # small settling time

# Main
try:
    print("Starting Flywheel ...")
    start_flywheel()
    time.sleep(3)  # spin-up time

    print("Firing 1 ...")
    reload()
    time.sleep(2.5)

    print("Firing 2 ...")
    reload()
    time.sleep(0.5)

    print("Firing 3 ...")
    reload()
    print("Stopping Flywheel ...")
    stop_flywheel()

finally:
    pwm_flywheel.stop()
    pwm_servo.stop()
    GPIO.cleanup()

