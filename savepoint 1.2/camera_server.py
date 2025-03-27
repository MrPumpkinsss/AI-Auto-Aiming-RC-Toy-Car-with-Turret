import socket
import threading
import cv2
import time
import numpy as np
from flask import Flask, Response
from picamera2 import Picamera2
from servo import ServoControl
from motor import MotorControl
from fire import FireControl

class CameraServer:
    def __init__(self, flask_port=5000, socket_port=10000):
        self.app = Flask(__name__)
        self.camera = Picamera2()
        self.flask_port = flask_port
        self.socket_port = socket_port
        self.current_color = "green"
        self.COLOR_RANGES = {
            "red": [(0, 100, 100), (10, 255, 255)],
            "orange": [(11, 100, 100), (25, 255, 255)],
            "yellow": [(26, 100, 100), (35, 255, 255)],
            "green": [(36, 100, 100), (85, 255, 255)],
            "cyan": [(86, 100, 100), (95, 255, 255)],
            "blue": [(96, 100, 100), (125, 255, 255)],
            "purple": [(126, 100, 100), (145, 255, 255)],
            "pink": [(146, 100, 100), (165, 255, 255)],
            "white": [(0, 0, 200), (180, 55, 255)],
            "gray": [(0, 0, 50), (180, 50, 200)],
            "black": [(0, 0, 0), (180, 255, 50)],
        }

        # Initialize servo control
        self.servo_control = ServoControl(x_axis_pin=29, y_axis_pin=32, x_range=(0, 270), y_range=(120, 150))

        # Initialize motor control
        self.motor_control = MotorControl(
            left_pwm_pin=11, left_forward_pin=13, left_reverse_pin=15,
            right_pwm_pin=19, right_forward_pin=21, right_reverse_pin=23
        )

        # Initialize fire control
        self.fire_control = FireControl(33, 31)

        # Initialize gimbal position
        self.gimbal_x = 135  # Start at the center position for x-axis
        self.gimbal_y = 135  # Start at the center position for y-axis
        self.servo_control.move_servo("x", self.gimbal_x)
        self.servo_control.move_servo("y", self.gimbal_y)

        self.__setup_camera()
        self.__setup_flask_routes()
        
    def __setup_camera(self):
        video_config = self.camera.create_video_configuration(
            main={"size": (640, 480), "format": "RGB888"},
        )
        self.camera.configure(video_config)
        self.camera.start()

    def __setup_flask_routes(self):
        @self.app.route("/cam")
        def cam():
            return Response(self.generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


    def generate_frames(self):
        prev_time = time.time()
        fps = 0
        last_temp_check = time.time()
        cpu_temp = None

        while True:
            try:
                frame = self.camera.capture_array()
                
                # Ensure frame is valid
                if frame is None or not isinstance(frame, np.ndarray):
                    print("Error: Captured frame is invalid")
                    continue

                # Convert frame to proper format
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                frame_filter = cv2.medianBlur(frame, 3)
                hsv_frame = cv2.cvtColor(frame_filter, cv2.COLOR_BGR2HSV)

                # Color detection
                lower_bound = np.array(self.COLOR_RANGES[self.current_color][0])
                upper_bound = np.array(self.COLOR_RANGES[self.current_color][1])
                mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                for contour in contours:
                    if len(contour) >= 5:
                        area = cv2.contourArea(contour)
                        if area > 100:
                            try:
                                ellipse = cv2.fitEllipse(contour)
                                cv2.ellipse(frame, ellipse, (255, 0, 0), 2)
                                (x, y), _, _ = ellipse
                                cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)
                                cv2.putText(frame, f"({int(x)}, {int(y)})", (int(x) + 10, int(y) - 10),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                            except cv2.error as e:
                                print(f"Ellipse fitting error: {e}")

                # Draw crosshair (center of the frame) with thicker lines
                frame_height, frame_width, _ = frame.shape
                center_x, center_y = frame_width // 2, frame_height // 2
                cv2.line(frame, (center_x - 20, center_y), (center_x + 20, center_y), (0, 255, 0), 2)
                cv2.line(frame, (center_x, center_y - 20), (center_x, center_y + 20), (0, 255, 0), 2)

                # Draw gimbal-to-body direction indicator (top-right corner with rectangular body)
                indicator_size = 100
                indicator_top_right = (frame_width - 150, 10)
                overlay = frame.copy()

                # Define the center and size of the rectangle
                rect_center = (indicator_top_right[0] + 50, indicator_top_right[1] + 50)
                rect_width, rect_height = 80, 40  # Rectangle dimensions

                # Adjust rotation angle to be relative to the new center (135 degrees)
                rotation_angle = -(self.gimbal_x - 45 ) # Adjusted to use the new central position as reference

                # Rotate the rectangle around its center
                rect_points = np.array([
                    (-rect_width // 2, -rect_height // 2),
                    (rect_width // 2, -rect_height // 2),
                    (rect_width // 2, rect_height // 2),
                    (-rect_width // 2, rect_height // 2)
                ])
                rotation_matrix = cv2.getRotationMatrix2D((0, 0), rotation_angle, 1.0)
                rotated_rect = np.dot(rotation_matrix[:, :2], rect_points.T).T + rect_center

                # Convert points to integer for drawing
                rotated_rect = np.int32(rotated_rect)
                cv2.polylines(overlay, [rotated_rect], isClosed=True, color=(255, 255, 255), thickness=2)

                # Draw the red fixed pointer (always pointing upwards)
                pointer_length = 40
                pointer_start = rect_center
                pointer_end = (rect_center[0], rect_center[1] - pointer_length)
                cv2.line(overlay, pointer_start, pointer_end, (0, 0, 255), 2)

                # Blend the overlay with the original frame for transparency
                alpha = 0.6
                frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)
                # FPS and CPU temperature
                current_time = time.time()
                fps = 1 / (current_time - prev_time)
                prev_time = current_time

                if current_time - last_temp_check >= 1:
                    cpu_temp = self.get_cpu_temperature()
                    last_temp_check = current_time

                cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(frame, f"Color: {self.current_color}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                if cpu_temp is not None:
                    cv2.putText(frame, f"CPU Temp: {cpu_temp:.2f}C", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                # Resize and encode frame for streaming
                frame_resized = cv2.resize(frame, (640, 350))
                ret, buffer = cv2.imencode(".jpg", frame_resized)
                frame = buffer.tobytes()

                yield (b"--frame\r\n"
                       b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")

            except Exception as e:
                print(f"Error capturing or processing frame: {e}")
                continue


    def get_cpu_temperature(self):
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as temp_file:
                temp = temp_file.read()
            return float(temp) / 1000.0
        except Exception as e:
            print(f"Error reading CPU temperature: {e}")
            return None

    def get_ip_address(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.connect(("8.8.8.8", 80))
            ip_address = sock.getsockname()[0]
            sock.close()
            return ip_address
        except Exception as e:
            print(f"Error getting IP address: {e}")
            return "No IP"
    def parse_string(self, s):
        if len(s) == 0:
            return 0
        if s.startswith("C"):
            color_name = s[1:].lower()
            return ["C", color_name]
        elif s.startswith("L") or s.startswith("R"):
            try:
                split_index = s.find(" ")
                end_index = s.find("S")
                direction = s[0]
                param1 = int(s[1:split_index])
                param2 = int(s[split_index + 1:end_index])
                return [direction, param1, param2]
            except (ValueError, IndexError):
                return 0
        elif s == "FIRE":
            return "FIRE"
        return 0
        
    def handle_fire(self):
        """Triggers the fire action in a separate thread."""
        threading.Thread(target=self.fire_control.fire).start()

    def handle_motor(self, data):
        """Processes motor control commands based on joystick data."""
        if data[0] == "C":
            color_name = data[1]
            if color_name in self.COLOR_RANGES:
                self.current_color = color_name
                print(f"Color changed to: {self.current_color}")

        elif data[0] == "L" or data[0] == "R":
            if data[0] == "L":
                x, y = data[1], data[2]

                # Define PWM range
                joystick_max = 50  # Maximum joystick input value
                min_pwm = 35  # Minimum PWM for motors to start moving
                max_pwm = 100  # Maximum PWM for full speed
                def scale_joystick_to_pwm(value):
                    if abs(value) <= 20:  # Dead zone for small input values
                        return 0
                    scaled_speed = int(min_pwm + (abs(value) - 20) / (joystick_max - 20) * (max_pwm - min_pwm))
                    return min(max_pwm, max(min_pwm, scaled_speed))  # Clamp PWM to valid range

                # Calculate motor speed based on y-axis input
                if abs(y) > 20:  # Forward or backward
                    pwm_speed = scale_joystick_to_pwm(y)
                    if y > 20:  # Forward
                        self.motor_control.move("left", "forward", pwm_speed)
                        self.motor_control.move("right", "forward", pwm_speed)
                    elif y < -20:  # Backward
                        self.motor_control.move("left", "backward", pwm_speed)
                        self.motor_control.move("right", "backward", pwm_speed)
                elif abs(x) > 20:  # Turning in place based on x-axis input
                    pwm_speed = scale_joystick_to_pwm(x)
                    if x > 20:  # Turn right in place
                        self.motor_control.move("left", "forward", pwm_speed)
                        self.motor_control.move("right", "backward", pwm_speed)
                    elif x < -20:  # Turn left in place
                        self.motor_control.move("left", "backward", pwm_speed)
                        self.motor_control.move("right", "forward", pwm_speed)
                else:  # Stop if no significant input
                    self.motor_control.stop()
            elif data[0] == "R":
                
                x, y = data[1], data[2]

                # Update gimbal position incrementally based on joystick input
                # Scale joystick input to control speed of gimbal movement
                self.gimbal_x -= int(x / 10)  # Scale x-axis input
                self.gimbal_y -= int(y / 20)  # Scale y-axis input

                self.servo_control.move_servo("x", self.gimbal_x)
                self.servo_control.move_servo("y", self.gimbal_y)


    def socket_server(self):
        while True:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                ip_address = self.get_ip_address()
                server_address = (ip_address, self.socket_port)
                print("Starting socket server on {} port {}".format(*server_address))
                sock.bind(server_address)
                sock.listen(1)

                while True:
                    connection, client_address = sock.accept()
                    try:
                        while True:
                            received_data = connection.recv(9).decode("utf-8").strip()
                            if not received_data:
                                break
                            parsed_data = self.parse_string(received_data)
                            print(parsed_data)
                            if parsed_data != 0:
                                if parsed_data[0] == "C":
                                    if "fire" in parsed_data[1]:
                                        self.handle_fire()
                                        print("Fire signal ack")
                                self.handle_motor(parsed_data)
                    except Exception as e:
                        print(f"Error during connection: {e}")
                    finally:
                        connection.close()
            except Exception as e:
                print(f"Error setting up socket server: {e}")
            finally:
                sock.close()

    def start(self):
        t1 = threading.Thread(target=self.app.run, kwargs={"host": "0.0.0.0", "port": self.flask_port})
        t2 = threading.Thread(target=self.socket_server)

        t1.start()
        t2.start()

        t1.join()
        t2.join()
