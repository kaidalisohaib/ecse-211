import brickpi3
import time

# DPM Lab 1
# Wall Follower using BrickPi
# Author: Katrina Poulin
# Date: March 2021

BP = brickpi3.BrickPi3() # instantiate brickPi

# sensor
US_SENSOR = BP.PORT_1
# Configure for an EV3 ultrasonic sensor.
BP.set_sensor_type(US_SENSOR, BP.SENSOR_TYPE.EV3_ULTRASONIC_CM)

LEFT_MOTOR = BP.PORT_B   # BP.PORT_C
RIGHT_MOTOR = BP.PORT_C  # BP.PORT_B

# speed constants
MOTOR_HIGH = 30
MOTOR_LOW = 20

# distance constants (in cm)
BANDCENTER = 21
BANDWIDTH = 3
HARD_TURN_THRESHOLD = 13

# polling sleep time (in seconds)
SLEEP_TIME = 0.02

# sensor init time (in seconds)
INIT_TIME = 5

# values needed for filter function
filter_count = 0
prev_input = 0
# function to filter out false negatives
def filter(input):
    global filter_count
    global prev_input
    if(input > 100):
        filter_count = filter_count + 1
        if(filter_count == 10):
            filter_count = 0
            prev_input = input
            return input
        else:
            return prev_input
    else:
        prev_input = input
        return input

# general structure: 'try' block with code, 'except' block with hard interrupt (Crtl+C)

try:
    time.sleep(INIT_TIME) # give time to sensors and motors to init
    while True:
        try:
            # get filtered distance
            distance = filter(BP.get_sensor(US_SENSOR))
            print(distance) # for debugging purposes
            if (distance > BANDCENTER + BANDWIDTH): # too far away from wall   
                # get closer to the wall
                BP.set_motor_power(LEFT_MOTOR, MOTOR_LOW)
                BP.set_motor_power(RIGHT_MOTOR, MOTOR_HIGH)

            elif (distance < BANDCENTER - BANDWIDTH): # too close to the wall
                if (distance < HARD_TURN_THRESHOLD): # way too close
                    # back up a little to avoid collisions
                    BP.set_motor_power(LEFT_MOTOR, -1*MOTOR_LOW)
                    BP.set_motor_power(RIGHT_MOTOR, -1*MOTOR_LOW)
                    time.sleep(SLEEP_TIME)
                    BP.set_motor_power(LEFT_MOTOR, MOTOR_LOW)
                    BP.set_motor_power(RIGHT_MOTOR, -1*MOTOR_LOW)
                    time.sleep(SLEEP_TIME)
                else: # get away from the wall
                    BP.set_motor_power(LEFT_MOTOR, MOTOR_HIGH)
                    BP.set_motor_power(RIGHT_MOTOR, MOTOR_LOW)

            else: # distance is fine; go forward
                BP.set_motor_power(LEFT_MOTOR, MOTOR_LOW)
                BP.set_motor_power(RIGHT_MOTOR, MOTOR_LOW)
            
            time.sleep(SLEEP_TIME)

        except brickpi3.SensorError as error:
            print(error)
        

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.
