import time
import busio
import board
import adafruit_amg88xx
import RPi.GPIO as GPIO 
import subprocess


# Initialize I2C bus
i2c_bus = busio.I2C(board.SCL, board.SDA)

# Initialize AMG8833 sensor
sensor = adafruit_amg88xx.AMG88XX(i2c_bus)

# Setting the threshold temperature
THRESHOLD = 31.0

# Allow sensor to initialize
time.sleep(0.1)

# Set pin numbering convention 
GPIO.setmode(GPIO.BCM)  # Using physical pin numbering 

# Choose an appropriate PWM channel to control the servo 
servo_pin = 12  # GPIO pin used for PWM 

# Set the pin as an output 
GPIO.setup(servo_pin, GPIO.OUT)

# Initialize the servo to be controlled by PWM with a 50 Hz frequency 
p = GPIO.PWM(servo_pin, 50) 
 
# Start the servo at 90 degrees (neutral position) 
p.start(7.5) 

#angles for heat sweep
angles = [30, 90, 120]
flag = 0

def set_servo_angle(angles): 
    for angle in angles:
        duty_cycle = 2.5 + (angle / 180) * 10 
        p.ChangeDutyCycle(duty_cycle) 
        time.sleep(2)  # Give the servo time to move 
        count = 0
        # Print sensor readings
        for row in sensor.pixels:
            for temp in row:
                if temp > THRESHOLD:
                    count += 1

           print(["{:.2f}".format(temp) for temp in row])  # Format for better readability
        print("\n" + "-" * 40)  # Separator for clarity
        if count > 40:
            print("Fire Flare")
            flag = 1
            break

try:
      set_servo_angle(angles)
      if flag == 1:
          subprocess.run(["python3", "Motor_Test.py"])    #Test motor activation sequence after heat detection
           print(1)
except KeyboardInterrupt:
    print("Exiting...")
    

finally:
    p.stop()
    GPIO.cleanup()
