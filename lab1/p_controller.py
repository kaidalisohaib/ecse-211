import time
from utils.brick import EV3UltrasonicSensor, Motor, reset_brick, wait_ready_sensors

us_sensor = EV3UltrasonicSensor("3")

motorR = Motor("C")
motorL = Motor("B")

def move(forward_speed, turn_bias):
    ## High turn bias, right motor turn faster = turn left
    motorR.set_dps(-forward_speed - turn_bias)
    motorL.set_dps(-forward_speed + turn_bias)
    
KP = 6.0
NOMINAL_SPEED = 150.0 #deg/s
BAND_CENTER = 40
BAND_WIDTH = 0.5
BIG_TURN_THRESHOLD = 70
counter = 0
MOVING_AVG = [0] * 20


if __name__ == "__main__":
    wait_ready_sensors()
    try:
        
        move(90, 0)

        #while False:
        
        while True:
            d = us_sensor.get_cm()
            error = min(max(KP*(d - BAND_CENTER),-90),90)
            
            print(d, error)
            
            move(max(0, NOMINAL_SPEED), error) 

            
            time.sleep(0.01)
    except BaseException as E:
        print(E)
        reset_brick()
        exit()

reset_brick()
print("RESET")