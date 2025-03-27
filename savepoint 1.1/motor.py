import RPi.GPIO as GPIO

class MotorControl:
    def __init__(self, left_pwm_pin, left_forward_pin, left_reverse_pin,
                 right_pwm_pin, right_forward_pin, right_reverse_pin):
        """
        Initializes the motor control with specified GPIO pins.
        """
        self.left_pwm_pin = left_pwm_pin
        self.left_forward_pin = left_forward_pin
        self.left_reverse_pin = left_reverse_pin

        self.right_pwm_pin = right_pwm_pin
        self.right_forward_pin = right_forward_pin
        self.right_reverse_pin = right_reverse_pin

        self.left_motor_pwm = None
        self.right_motor_pwm = None

        self.__setup()

    def __setup(self):
        """
        Sets up the GPIO pins and initializes PWM for the motors.
        """
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

        # Setup GPIO pins for the left motor
        GPIO.setup(self.left_pwm_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.left_forward_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.left_reverse_pin, GPIO.OUT, initial=GPIO.LOW)

        # Setup GPIO pins for the right motor
        GPIO.setup(self.right_pwm_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.right_forward_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.right_reverse_pin, GPIO.OUT, initial=GPIO.LOW)

        # Initialize PWM for the motors
        self.left_motor_pwm = GPIO.PWM(self.left_pwm_pin, 100)  # 100 Hz frequency
        self.right_motor_pwm = GPIO.PWM(self.right_pwm_pin, 100)

        self.left_motor_pwm.start(0)  # Start with 0% duty cycle
        self.right_motor_pwm.start(0)

    def move(self, side, direction, speed):
        """
        Moves the specified motor (left or right) in the given direction at the given speed.

        :param side: "left" or "right" to specify the motor.
        :param direction: "forward", "backward", or "stop".
        :param speed: Speed as a percentage (0-100).
        """
        if side == "left":
            pwm = self.left_motor_pwm
            forward_pin = self.left_forward_pin
            reverse_pin = self.left_reverse_pin
        elif side == "right":
            pwm = self.right_motor_pwm
            forward_pin = self.right_forward_pin
            reverse_pin = self.right_reverse_pin
        else:
            raise ValueError("Invalid side: must be 'left' or 'right'")

        if direction == "forward":
            GPIO.output(forward_pin, GPIO.HIGH)
            GPIO.output(reverse_pin, GPIO.LOW)
        elif direction == "backward":
            GPIO.output(forward_pin, GPIO.LOW)
            GPIO.output(reverse_pin, GPIO.HIGH)
        elif direction == "stop":
            GPIO.output(forward_pin, GPIO.LOW)
            GPIO.output(reverse_pin, GPIO.LOW)
        else:
            raise ValueError("Invalid direction: must be 'forward', 'backward', or 'stop'")

        pwm.ChangeDutyCycle(speed)

    def stop(self):
        """
        Stops both motors.
        """
        self.move("left", "stop", 0)
        self.move("right", "stop", 0)

    def cleanup(self):
        """
        Stops the motors, stops PWM, and cleans up GPIO resources.
        """
        self.stop()
        self.left_motor_pwm.stop()
        self.right_motor_pwm.stop()
        GPIO.cleanup()
