import RPi.GPIO as GPIO
import time
from servo import ServoControl  # Import the ServoControl class from servo.py


class FireControl:
    def __init__(self, servo_board_pin=33, relay_pin=31):
        """
        Initialize the FireControl system.

        :param servo_board_pin: BOARD pin for the servo motor
        :param relay_pin: BOARD pin for the relay
        """
        # Relay setup (still uses RPi.GPIO)
        self.relay_pin = relay_pin
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(self.relay_pin, GPIO.OUT)
        GPIO.output(self.relay_pin, GPIO.LOW)  # Ensure relay is off initially

        # Servo setup (uses ServoControl)
        self.servo = ServoControl(x_axis_pin=servo_board_pin, y_axis_pin=None, x_range=(0, 180))  # Only one axis is used

    def fire(self):
        """
        Triggers the reload and shoot action.
        """
        self.servo.move_servo('x', 180)  # Move servo to the reloading position
        GPIO.output(self.relay_pin, GPIO.HIGH)  # Turn the relay on
        time.sleep(1.5)  # Wait for firing
        self.servo.move_servo('x', 0)  # Move servo back to the default position
        time.sleep(1)
        self.servo.move_servo('x', 180)  # Reset servo for reloading
        GPIO.output(self.relay_pin, GPIO.LOW)  # Turn the relay off

    def cleanup(self):
        """
        Cleans up both the servo and the relay.
        """
        GPIO.output(self.relay_pin, GPIO.LOW)  # Ensure relay is off
        self.servo.cleanup()  # Cleanup the servo
        GPIO.cleanup()  # Cleanup GPIO pins
