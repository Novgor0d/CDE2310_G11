import time
import busio
import board
import adafruit_amg88xx

# Initialize I2C bus
i2c_bus = busio.I2C(board.SCL, board.SDA)

# Initialize AMG8833 sensor
sensor = adafruit_amg88xx.AMG88XX(i2c_bus)

# Allow sensor to initialize
time.sleep(0.1)

while True:
    # Read and print the 8x8 pixel temperature array
    for row in sensor.pixels:
        print(["{:.2f}".format(temp) for temp in row])  # Format for better readability
    print("\n" + "-" * 40)  # Separator for clarity
    time.sleep(1)  # Read every second