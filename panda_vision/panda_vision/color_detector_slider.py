#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ColorTuner(Node):
    def __init__(self):
        super().__init__('color_tuner')

        # Subscribe to camera feed
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.bridge = CvBridge()
        self.get_logger().info("🎨 Color Range + Contour Area Tuner Started")

        # --- Create window + trackbars ---
        cv2.namedWindow("Trackbars", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Trackbars", 450, 400)

        # HSV lower/upper limits
        self.init_trackbar("LH", 35)   # Lower Hue
        self.init_trackbar("LS", 100)  # Lower Saturation
        self.init_trackbar("LV", 100)  # Lower Value
        self.init_trackbar("UH", 90)   # Upper Hue
        self.init_trackbar("US", 255)  # Upper Saturation
        self.init_trackbar("UV", 255)  # Upper Value

        # Contour area threshold
        self.init_trackbar("Min Area", 500)  # min contour area

    def init_trackbar(self, name, value):
        """Helper to create a trackbar"""
        cv2.createTrackbar(name, "Trackbars", value, 2550, lambda x: None)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # --- Read all trackbar positions ---
        lh = cv2.getTrackbarPos("LH", "Trackbars")
        ls = cv2.getTrackbarPos("LS", "Trackbars")
        lv = cv2.getTrackbarPos("LV", "Trackbars")
        uh = cv2.getTrackbarPos("UH", "Trackbars")
        us = cv2.getTrackbarPos("US", "Trackbars")
        uv = cv2.getTrackbarPos("UV", "Trackbars")
        min_area = cv2.getTrackbarPos("Min Area", "Trackbars")

        lower = np.array([lh, ls, lv])
        upper = np.array([uh, us, uv])

        # --- Create mask ---
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # --- Find contours ---
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > min_area:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                cv2.putText(frame, f"Area: {int(area)}", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        # --- Display windows ---
        cv2.imshow("Mask", mask)
        cv2.imshow("Color Detection", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ColorTuner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
