import time
from utils.brick import EV3UltrasonicSensor, Motor, reset_brick, wait_ready_sensors

us_sensor = EV3UltrasonicSensor("3")

motorR = Motor("C")
motorL = Motor("B")

def move(forward_speed, turn_bias):
    ## High turn bias, right motor turn faster = turn left
    motorR.set_dps(-forward_speed - turn_bias)
    motorL.set_dps(-forward_speed + turn_bias)
    
TARGET_DIST = 40.0
KP = 1.0
NOMINAL_SPEED = 100.0 #deg/s
BAND_CENTER = 30
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
            if d < 2:
                continue
            #MOVING_AVG.pop()
            #MOVING_AVG.insert(0, reading)
            #d = sum(MOVING_AVG)/len(MOVING_AVG)
            
            print(d)
            if d > BIG_TURN_THRESHOLD:
                counter += 1
            else:
                counter = 0
            ## Turn Right    
            if d < BAND_CENTER - BAND_WIDTH:
                move(155, -80)
                time.sleep(0.75)
            elif counter > 50:
                ##move(200, 0)
                ##time.sleep(0.1)
                move(360,  75)
                time.sleep(0.7)
            elif d > BAND_CENTER + BAND_WIDTH:
                move(155, 60)
            else:
                move(90,-20)
            

#            error = min(-30, max(KP*(TARGET_DIST - d),30))
            error = 0
            #move(0.0, 0)
            time.sleep(0.01)
    except BaseException as E:
        print(E)
        reset_brick()
        exit()

reset_brick()
print("RESET")