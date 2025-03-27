from camera_server import CameraServer
from servo import ServoControl
from fire import FireControl
from motor import MotorControl

def main():
    try:
        camera_server = CameraServer(flask_port=5000, socket_port=10000)
        print("Starting camera server...")
        camera_server.start()

    except KeyboardInterrupt:
        print("\nExiting...")

    finally:
        servo_control.cleanup()
        fire_control.cleanup()
        motor_control.cleanup()
        print("Cleanup complete. Exiting program.")

if __name__ == "__main__":
    main()
