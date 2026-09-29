# Basic odometer 

import math

THETA = 0
X = 0
Y = 0
RADIUS_WHEEL_M = 
DISTANCE_BETWEEN_WHEELS = 

def float_motors():
        left_motor.float_motor()
        right_motor.float_motor()

        while(True):
            update_odometer()
            time.sleep(0.01)

def update_odometer():
        prev_encoder_l = 
        prev_encoder_r = 

        current_encoder_l = 
        current_encoder_r = 

        # Delta encoders, handle overflow cases
        delta_l = current_encoder_l - prev_encoder_l
        delta_l = delta_l % 360 if delta < -100 else delta_l

        delta_r = current_encoder_r - prev_encoder_r
        delta_r = delta_r % 360 if delta < -100 else delta_r
                
        # Update prev encoders
        prev_encoder_l = current_encoder_l
        prev_encoder_r = current_encoder_r

                    # Distances traveled by encoders
                    Distance_right = (delta_r) * 2*math.pi/360
                            Distance_left  = (delta_l) * 2*math.pi/360

                                Distance_average = .5 * (Distance_right + Distance_left)

                                    # Change in angle 
                                        delta_theta = (Distance_right - Distance_left)/DISTANCE_BETWEEN_WHEELS

                                            # Find changes in X and Y
                                                THETA = (THETA + delta_theta) % 360
                                                    X = X + Distance_average * math.cos(math.rad(THETA))
                                                        Y = Y + Distance_average * math.sin(math.rad(THETA))

                                                                    def line_corrector():
                                                                            # 






