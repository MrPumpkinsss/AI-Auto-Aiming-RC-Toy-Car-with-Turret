import cv2
import numpy as np
from picamera2 import Picamera2

COLOR_DICT = {
    "White": ([0, 0, 90], [180, 30, 100]),
    "Silver": ([0, 0, 75], [180, 10, 90]),
    "Gray": ([0, 0, 40], [180, 10, 70]),
    "Black": ([0, 0, 0], [180, 255, 30]),
    "Red": ([0, 100, 50], [10, 255, 100]),
    "Maroon": ([0, 100, 25], [10, 255, 50]),
    "Yellow": ([25, 100, 50], [35, 255, 100]),
    "Olive": ([25, 100, 25], [35, 255, 50]),
    "Lime": ([55, 100, 50], [65, 255, 100]),
    "Green": ([55, 100, 25], [65, 255, 50]),
    "Aqua": ([85, 100, 50], [95, 255, 100]),
    "Teal": ([85, 100, 25], [95, 255, 50]),
    "Blue": ([115, 100, 50], [125, 255, 100]),
    "Navy": ([115, 100, 25], [125, 255, 50]),
    "Fuchsia": ([145, 100, 50], [155, 255, 100]),
    "Purple": ([145, 100, 25], [155, 255, 50]),
}

def nothing(x):
    pass

cv2.namedWindow("Trackbars")
cv2.createTrackbar("H Min", "Trackbars", 0, 179, nothing)
cv2.createTrackbar("H Max", "Trackbars", 179, 179, nothing)
cv2.createTrackbar("S Min", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("S Max", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("V Min", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("V Max", "Trackbars", 255, 255, nothing)

cv2.namedWindow("Color Selector")
cv2.createTrackbar("Color Index", "Color Selector", 0, len(COLOR_DICT) - 1, nothing)

camera = Picamera2()
video_config = camera.create_video_configuration(
    main={"size": (640, 480), "format": "RGB888"}
)
camera.configure(video_config)
camera.start()

color_names = list(COLOR_DICT.keys())

while True:
    frame = camera.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    frame_filter = cv2.medianBlur(frame, 3)
    hsv_frame = cv2.cvtColor(frame_filter, cv2.COLOR_BGR2HSV)

    h_min = cv2.getTrackbarPos("H Min", "Trackbars")
    h_max = cv2.getTrackbarPos("H Max", "Trackbars")
    s_min = cv2.getTrackbarPos("S Min", "Trackbars")
    s_max = cv2.getTrackbarPos("S Max", "Trackbars")
    v_min = cv2.getTrackbarPos("V Min", "Trackbars")
    v_max = cv2.getTrackbarPos("V Max", "Trackbars")

    lower_bound = np.array([h_min, s_min, v_min])
    upper_bound = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)

    color_index = cv2.getTrackbarPos("Color Index", "Color Selector")
    selected_color = color_names[color_index]
    color_lower, color_upper = COLOR_DICT[selected_color]
    color_mask = cv2.inRange(hsv_frame, np.array(color_lower), np.array(color_upper))

    combined_mask = cv2.bitwise_or(mask, color_mask)
    contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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

    cv2.putText(frame, f"Color: {selected_color}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.imshow("Original", frame)
    cv2.imshow("Mask", combined_mask)

    key = cv2.waitKey(1) & 0xFF
    if key == 27:
        break

camera.stop()
cv2.destroyAllWindows()
