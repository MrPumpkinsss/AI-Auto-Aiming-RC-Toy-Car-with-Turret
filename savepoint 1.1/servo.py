import pigpio
import time


class ServoControl:
    # Complete BOARD to BCM pin mapping
    BOARD_TO_BCM = {
        3: 2,
        5: 3,
        7: 4,
        8: 14,
        10: 15,
        11: 17,
        12: 18,
        13: 27,
        15: 22,
        16: 23,
        18: 24,
        19: 10,
        21: 9,
        22: 25,
        23: 11,
        24: 8,
        26: 7,
        29: 5,
        31: 6,
        32: 12,
        33: 13,
        35: 19,
        36: 16,
        37: 26,
        38: 20,
        40: 21
    }

    def __init__(self, x_axis_pin, y_axis_pin, x_range=(45, 135), y_range=(80, 100)):
        self.pi = pigpio.pi()  # Initialize the pigpio instance
        if not self.pi.connected:
            raise RuntimeError("Unable to connect to the pigpio daemon. Make sure it's running.")

        # Map BOARD pins to BCM pins
        self.x_axis_pin = self.BOARD_TO_BCM.get(x_axis_pin)
        self.y_axis_pin = self.BOARD_TO_BCM.get(y_axis_pin)
        
        self.x_min, self.x_max = x_range
        self.y_min, self.y_max = y_range


    def _angle_to_pulsewidth(self, angle):
        """
        Converts an angle to a pulse width in microseconds.

        :param angle: Target angle (0 to 180)
        :return: Pulse width value (500 to 2500 microseconds)
        """
        return int(500 + (angle / 180.0) * 2000)

    def move_servo(self, axis, angle):
        """
        Moves the specified servo to the given angle.

        :param axis: 'x' for x-axis servo, 'y' for y-axis servo
        :param angle: Target angle
        """
        if axis == 'x':
            angle = max(self.x_min, min(angle, self.x_max))  # Clamp angle to x-axis range
            pin = self.x_axis_pin
        elif axis == 'y':
            angle = max(self.y_min, min(angle, self.y_max))  # Clamp angle to y-axis range
            pin = self.y_axis_pin
        else:
            raise ValueError("Invalid axis. Use 'x' or 'y'.")

        pulsewidth = self._angle_to_pulsewidth(angle)
        self.pi.set_servo_pulsewidth(pin, pulsewidth)

    def cleanup(self):
        """
        Stops the servos and releases resources.
        """
        self.pi.set_servo_pulsewidth(self.x_axis_pin, 0)  # Stop x-axis servo
        self.pi.set_servo_pulsewidth(self.y_axis_pin, 0)  # Stop y-axis servo
        self.pi.stop()  # Stop pigpio instance

