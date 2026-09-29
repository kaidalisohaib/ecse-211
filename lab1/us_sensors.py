from utils.brick import EV3UltrasonicSensor, reset_brick, wait_ready_sensors
import time

us_sensor = EV3UltrasonicSensor("3")

if __name__ == "__main__":
    wait_ready_sensors()
    try:
        while True:
            d = us_sensor.get_cm()
            print(d)
            time.sleep(0.1)
    except BaseException as e:
        print(e)
        reset_brick()
        exit()
    reset_brick()