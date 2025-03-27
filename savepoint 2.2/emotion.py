import time
import threading
from servo import ServoControl
from matrix8x8 import LEDMatrixAnimator

class EmotionActions:
    def __init__(self, servo_control, led_matrix):
        self.servo_control = servo_control
        self.led_matrix = led_matrix
        self.gimbal_x = 135  
        self.gimbal_y = 135  

    def trigger_emotion(self, emotion):
        if emotion in self.led_matrix.animations:
            animation_thread = threading.Thread(target=self.led_matrix.play_preset, args=(emotion,))
            animation_thread.start()
        self._perform_emotion_action(emotion)

    def _perform_emotion_action(self, emotion):
        if emotion == "Happy":
            for _ in range(3):
                new_y = self.gimbal_y - 10
                self.servo_control.move_servo("y", new_y)
                time.sleep(0.6) 
                self.servo_control.move_servo("y", self.gimbal_y)
                time.sleep(0.6)

        elif emotion == "Confused":
            for _ in range(3):
                new_x = self.gimbal_x - 15
                self.servo_control.move_servo("x", new_x)
                time.sleep(0.6)  
                new_x = self.gimbal_x + 15
                self.servo_control.move_servo("x", new_x)
                time.sleep(0.6)

        elif emotion == "Sad":
            for _ in range(3):
                new_y = self.gimbal_y - 5
                self.servo_control.move_servo("y", new_y)
                time.sleep(1.2)  
                self.servo_control.move_servo("y", self.gimbal_y)
                time.sleep(1.2)

        elif emotion == "Expecting":
            for _ in range(2):
                new_x = self.gimbal_x + 10
                self.servo_control.move_servo("x", new_x)
                time.sleep(0.9)  
                new_x = self.gimbal_x - 10
                self.servo_control.move_servo("x", new_x)
                time.sleep(0.9)

        time.sleep(1.5) 
        self.servo_control.move_servo("x", self.gimbal_x)
        self.servo_control.move_servo("y", self.gimbal_y)
