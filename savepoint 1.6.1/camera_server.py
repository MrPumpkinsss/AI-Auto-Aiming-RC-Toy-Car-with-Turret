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
        self.aiming_mode = False 
        self.target_position = None
        self.locked = False  
        self.locked_crosshair_position = None 
        self.COLOR_RANGES = {
            "red": [(0, 100, 100), (10, 255, 255)],
            "orange": [(11, 100, 100), (25, 255, 255)],
            "yellow": [(26, 100, 100), (35, 255, 255)],
            "green": [(36, 100, 100), (85, 255, 255)],
            "cyan": [(86, 100, 100), (95, 255, 255)],
            "blue": [(96, 100, 100), (125, 255, 255)],
        }
        # Initialize PID control variables
        self.error_sum_x = 0.0
        self.error_sum_y = 0.0
        self.last_error_x = 0.0
        self.last_error_y = 0.0
        self.last_pid_output_x = 0.0
        self.last_pid_output_y = 0.0
        self.last_time = time.time() 
        
        # Limits for integral term to prevent integral windup
        self.integral_limit_x = 100.0
        self.integral_limit_y = 100.0

        # Limits for derivative term to prevent excessive response
        self.derivative_limit_x = 50.0
        self.derivative_limit_y = 50.0
        # Smooth factor for PID output (0 < alpha < 1)
        self.alpha = 0.8
        
        self.kp_x = 0.007  # Proportional gain for x-axis
        self.ki_x = 0.0002  # Integral gain for x-axis
        self.kd_x = 0.0015  # Derivative gain for x-axis
        self.kp_y = self.kp_x  # Proportional gain for y-axis
        self.ki_y = self.ki_x  # Integral gain for y-axis
        self.kd_y = self.kd_x # Derivative gain for y-axis
        # Initialize servo control
        self.servo_control = ServoControl(x_axis_pin=29, y_axis_pin=32, x_range=(0, 270), y_range=(110, 160))

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
     
    def aim_at_target(self, target_x, target_y, center_x, center_y):
        """
        Use PID control to aim the gimbal at the target.

        :param target_x: Target x-coordinate
        :param target_y: Target y-coordinate
        :param center_x: Center x-coordinate of the frame
        :param center_y: Center y-coordinate of the frame
        """
        if not self.aiming_mode:  # Only aim if aiming mode is active
            return
            
        error_x = target_x - center_x
        error_y = target_y - center_y
        if abs(error_x) <= 2 and abs(error_y) <= 2:
            return
        current_time = time.time()
        delta_time = current_time - self.last_time

        if delta_time <= 0:
            delta_time = 0.001

        # Integral term
        self.error_sum_x += error_x * delta_time
        self.error_sum_x = max(-self.integral_limit_x, min(self.integral_limit_x, self.error_sum_x))  # Limit integral

        # Derivative term
        delta_error_x = (error_x - self.last_error_x) / delta_time
        delta_error_x = max(-self.derivative_limit_x, min(self.derivative_limit_x, delta_error_x))  # Limit derivative

        # Combine PID terms
        pid_output_x = (
            self.kp_x * error_x +
            self.ki_x * self.error_sum_x +
            self.kd_x * delta_error_x
        )

        # Smooth the output
        pid_output_x = self.alpha * pid_output_x + (1 - self.alpha) * self.last_pid_output_x
        self.last_pid_output_x = pid_output_x

        # === PID calculation for Y-axis ===
        # Integral term
        self.error_sum_y += error_y * delta_time
        self.error_sum_y = max(-self.integral_limit_y, min(self.integral_limit_y, self.error_sum_y))  # Limit integral

        # Derivative term
        delta_error_y = (error_y - self.last_error_y) / delta_time
        delta_error_y = max(-self.derivative_limit_y, min(self.derivative_limit_y, delta_error_y))  # Limit derivative

        # Combine PID terms
        pid_output_y = -(
            self.kp_y * error_y +
            self.ki_y * self.error_sum_y +
            self.kd_y * delta_error_y
        )

        # Smooth the output
        pid_output_y = self.alpha * pid_output_y + (1 - self.alpha) * self.last_pid_output_y
        self.last_pid_output_y = pid_output_y

        # === Update gimbal angles ===
        # Clamp the gimbal angles to the valid range (e.g., 0 to 270 degrees)
        self.gimbal_x = max(0, min(270, self.gimbal_x - pid_output_x))
        self.gimbal_y = max(110, min(160, self.gimbal_y - pid_output_y))
        # Move the gimbal
        self.servo_control.move_servo("x", self.gimbal_x)
        self.servo_control.move_servo("y", self.gimbal_y)

        # === Update state variables ===
        self.last_error_x = error_x
        self.last_error_y = error_y
        self.last_time = current_time

    def __draw_crosshair(self, frame):
        """Draw crosshair or LOCK mode red crosshair."""
        frame_height, frame_width, _ = frame.shape
        center_x, center_y = frame_width // 2, frame_height // 2

        # Draw the green crosshair at the center of the screen
        cv2.line(frame, (center_x - 20, center_y), (center_x + 20, center_y), (0, 255, 0), 2)
        cv2.line(frame, (center_x, center_y - 20), (center_x, center_y + 20), (0, 255, 0), 2)

        if self.locked:
            # Ensure locked_crosshair_position is initialized
            if self.locked_crosshair_position is None:
                self.locked_crosshair_position = [center_x, center_y]

            # Enforce boundary limits
            x, y = self.locked_crosshair_position
            x = max(20, min(frame_width-20, x))
            y = max(20, min(frame_height-20, y))
            self.locked_crosshair_position = [x, y]

            # Draw the red crosshair at the locked position
            cv2.line(frame, (x - 20, y), (x + 20, y), (0, 0, 255), 2)
            cv2.line(frame, (x, y - 20), (x, y + 20), (0, 0, 255), 2)

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
        """Generate video frames with optional LOCK mode crosshair."""
        prev_time = time.time()
        fps = 0
        last_temp_check = time.time()
        cpu_temp = None

        while True:
            try:
                frame = self.camera.capture_array()

                if frame is None or not isinstance(frame, np.ndarray):
                    print("Error: Captured frame is invalid")
                    continue

                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                frame_filter = cv2.medianBlur(frame, 3)
                hsv_frame = cv2.cvtColor(frame_filter, cv2.COLOR_BGR2HSV)

                # Perform color detection
                self.__process_color_detection(frame, hsv_frame)

                # Draw crosshair or LOCK mode red crosshair
                self.__draw_crosshair(frame)

                # Draw gimbal-to-body direction indicator (top-right corner with rectangular body)
                indicator_size = 100
                indicator_top_right = (640 - 150, 10)
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
                
    def __process_color_detection(self, frame, hsv_frame):
        """Detect color and update target position."""
        lower_bound = np.array(self.COLOR_RANGES[self.current_color][0])
        upper_bound = np.array(self.COLOR_RANGES[self.current_color][1])
        mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        largest_area = 0
        largest_ellipse = None

        for contour in contours:
            if len(contour) >= 5:
                area = cv2.contourArea(contour)
                if area > 100:
                    try:
                        ellipse = cv2.fitEllipse(contour)
                        if area > largest_area:
                            largest_area = area
                            largest_ellipse = ellipse
                    except cv2.error as e:
                        print(f"Ellipse fitting error: {e}")

        if largest_ellipse is not None:
            (x, y), _, _ = largest_ellipse
            self.target_position = (int(x), int(y))
            cv2.ellipse(frame, largest_ellipse, (255, 0, 0), 2)
            cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)
            cv2.putText(frame, f"Target: ({int(x)}, {int(y)})", (int(x) + 10, int(y) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        else:
            self.target_position = None



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
            if s[1:] == "AIM":  # New command for enabling aiming
                return "AIM"
            elif s[1:] == "UAIM":  # New command for disabling aiming
                return "UAIM"
            elif s[1:] == "LOCK":  # New command for disabling aiming
                return "LOCK"
            elif s[1:] == "ULOCK":  # New command for disabling aiming
                return "ULOCK"
            return ["C", color_name]
        elif s.startswith("L") or s.startswith("R"):
            try:
                split_index = s.find(" ")
                direction = s[0]
                param1 = int(s[1:split_index])
                param2 = int(s[split_index + 1:7])
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
        if data == "AIM":
            self.aiming_mode = True  # Enable aiming mode
            print("Aiming mode enabled.")
        elif data == "UAIM":
            self.aiming_mode = False  # Disable aiming mode
            print("Aiming mode disabled.")
        elif data[0] == "C":
            color_name = data[1]
            if color_name in self.COLOR_RANGES:
                self.current_color = color_name
                print(f"Color changed to: {self.current_color}")
        elif data == "LOCK":
            self.locked = True
            self.locked_crosshair_position = None  # Reset crosshair to center
            print("LOCK mode activated.")
        elif data == "ULOCK":
            self.locked = False
            self.locked_crosshair_position = None
            print("LOCK mode deactivated.")
        elif self.locked and data[0] == "R":
            # Control crosshair position with joystick in LOCK mode
            x, y = data[1], data[2]
            if self.locked_crosshair_position:
                self.locked_crosshair_position[0] += int(x)
                self.locked_crosshair_position[1] -= int(y)
        elif data[0] == "L" or data[0] == "R":
            if self.aiming_mode == True:
                # Use detected target position and frame center for aiming
                frame_height, frame_width, _ = self.camera.capture_array().shape
                center_x, center_y = frame_width // 2, frame_height // 2
                if self.target_position is not None:
                    target_x, target_y = self.target_position
                    self.aim_at_target(target_x, target_y, center_x, center_y)
                    print(f"Aiming at target: {self.target_position}")
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
                self.gimbal_x -= int(x / 14)  # Scale x-axis input
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
                            received_data = connection.recv(8).decode("utf-8").strip()
                            if not received_data:
                                break

                            parsed_data = self.parse_string(received_data)
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
