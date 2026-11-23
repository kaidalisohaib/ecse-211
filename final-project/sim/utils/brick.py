"""
Fake utils.brick for the laptop simulator. Same class names and methods the robot code
uses; every call is forwarded to the simulated world in sim/world.py.
"""
WORLD = None  # set by run_sim.py before the robot code is imported


class Motor:
    def __init__(self, port, bp=None):
        self.port = str(port).upper()

    def set_power(self, power):
        WORLD.motor_set_power(self.port, power)

    def float_motor(self):
        WORLD.motor_set_power(self.port, 0)

    def set_dps(self, dps):
        WORLD.motor_set_dps(self.port, dps)

    def set_limits(self, power=0, dps=0):
        WORLD.motor_set_limits(self.port, power, dps)

    def set_position(self, position):
        WORLD.motor_set_position(self.port, position)

    def set_position_relative(self, degrees):
        WORLD.motor_set_position(self.port, WORLD.motor_get_position(self.port) + degrees)

    def get_position(self):
        return WORLD.motor_get_position(self.port)

    def get_encoder(self):
        return WORLD.motor_get_position(self.port)

    def reset_encoder(self):
        WORLD.motor_reset(self.port)


class EV3ColorSensor:
    def __init__(self, port, mode=None):
        self.port = int(port)

    def get_red(self):
        return WORLD.color_red(self.port)


class EV3UltrasonicSensor:
    def __init__(self, port, mode=None):
        self.port = int(port)

    def get_cm(self):
        return WORLD.us_cm(self.port)


class TouchSensor:
    def __init__(self, port, mode=None):
        self.port = int(port)

    def is_pressed(self):
        return False


def reset_brick():
    pass


def wait_ready_sensors(debug=False):
    pass
