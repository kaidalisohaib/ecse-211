#!/usr/bin/env python3

# Ultrasonic sensor example
# Author: Younes Boubekeur

import atexit
import brickpi3
import os
import signal
from time import sleep

# Save process ID of this program so we can force stop it later if needed
os.system(f"echo {os.getpid()} > ~/brickpi3_pid")

# Instantiate BrickPi
BP = brickpi3.BrickPi3()

# Set up ultrasonic sensor on Port 1
US_SENSOR = BP.PORT_1
BP.set_sensor_type(US_SENSOR, BP.SENSOR_TYPE.EV3_ULTRASONIC_CM)

def reset_brick(*args):
    "Reset BrickPi devices when program exits ('at exit')."
    print("Exiting US sensor example program")
    BP.reset_all()

atexit.register(reset_brick)
signal.signal(signal.SIGTERM, reset_brick)
signal.signal(signal.SIGINT, reset_brick)  # Ctrl-C

# Print ultrasonic distance once each second in an infinite loop
while True:
    try:
        print(f"US Distance: {BP.get_sensor(US_SENSOR)} cm")
    except brickpi3.SensorError:
        pass  # sensor has no valid data, so do nothing for this iteration
    sleep(1)  # sleep for 1 second
