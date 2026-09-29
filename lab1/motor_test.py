import time
from utils.brick import EV3UltrasonicSensor, Motor, reset_brick, wait_ready_sensors

us_sensor = EV3UltrasonicSensor("3")

motorR = Motor("C")
motorL = Motor("B")

def move(forward_speed, turn_bias):
    
    motorR.set_dps(-forward_speed - turn_bias)
    motorL.set_dps(-forward_speed - turn_bias)

TARGET_DIST = 20.0
KP = 1.0
NOMINAL_SPEED = 100.0 #deg/s
  

if __name__ == "__main__":
    wait_ready_sensors()
    try:
        
        move(90, 0)

        #while False:
        while True:

            d = max(us_sensor.get_cm(),40)
            print(d)
            
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