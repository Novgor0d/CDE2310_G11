import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

FLYWHEEL = 11
RELOAD = 13
ENA = 33
ENB = 32

GPIO.setup([FLYWHEEL,RELOAD,ENA,ENB],GPIO.OUT)

pwm_flywheel = GPIO.PWM(ENA,1000)
pwm_reload = GPIO.PWN(ENB, 1000)

pwm_flywheel.start(50)
pwm_reload.start(50)

def start_flywheel():
    GPIO.output(FLYWHEEL, GPIO.HIGH)

def stop_flywheel():
    GPIO.output(FLYWHEEL, GPIO.LOW)

def start_reload():
    GPIO.output(RELOAD, GPIO.HIGH)

def stop_reload():
    GPIO.output(RELOAD, GPIO.LOW)

try:
    print("Starting Flywheel ...")
    start_flywheel()
    time.sleep(3)

    print("Stopping Flywheel ...")
    stop_flywheel()
    
    print("Starting Reload ...")
    start_reload()
    time.sleep(2)

    print("Stopping Reload ...")
    stop_reload()

finally:
    pwm_flywheel.stop()
    pwm_reload.stop()
    GPIO.cleanup()