import time
from servo import ServoControl

class EmotionActions:
    def __init__(self, servo_control):
        self.servo_control = servo_control
        self.gimbal_x = 135  
        self.gimbal_y = 135  

    def trigger_emotion(self, emotion):
        self._perform_emotion_action(emotion)

    def _perform_emotion_action(self, emotion):
        if emotion == "Happy":
            for _ in range(3):
                new_y = self.gimbal_y - 10
                print(f"Moving Y to {new_y}")
                self.servo_control.move_servo("y", new_y)
                time.sleep(0.6) 
                
                print(f"Returning Y to {self.gimbal_y}")
                self.servo_control.move_servo("y", self.gimbal_y)
                time.sleep(0.6)

        elif emotion == "Confused":
            for _ in range(3):
                new_x = self.gimbal_x - 15
                print(f"Moving X to {new_x}")
                self.servo_control.move_servo("x", new_x)
                time.sleep(0.6)  
                
                new_x = self.gimbal_x + 15
                print(f"Moving X to {new_x}")
                self.servo_control.move_servo("x", new_x)
                time.sleep(0.6)

        elif emotion == "Sad":
            for _ in range(3):
                new_y = self.gimbal_y - 5
                print(f"Moving Y to {new_y}")
                self.servo_control.move_servo("y", new_y)
                time.sleep(1.2)  
                
                print(f"Returning Y to {self.gimbal_y}")
                self.servo_control.move_servo("y", self.gimbal_y)
                time.sleep(1.2)

        elif emotion == "Expecting":
            for _ in range(2):
                new_x = self.gimbal_x + 10
                print(f"Moving X to {new_x}")
                self.servo_control.move_servo("x", new_x)
                time.sleep(0.9)  
                
                new_x = self.gimbal_x - 10
                print(f"Moving X to {new_x}")
                self.servo_control.move_servo("x", new_x)
                time.sleep(0.9)

        time.sleep(1.5) 
        print(f"Resetting X to {self.gimbal_x}, Y to {self.gimbal_y}")
        self.servo_control.move_servo("x", self.gimbal_x)
        self.servo_control.move_servo("y", self.gimbal_y)
