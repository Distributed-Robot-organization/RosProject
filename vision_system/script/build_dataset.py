#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import os
import threading
import time
from datetime import datetime

class ImageCapture(Node):
    def __init__(self):
        super().__init__('image_capture_node')
        self.bridge = CvBridge()
        self.current_image = None
        self.capturing = False
        self.save_dir = os.path.join("..", "data_images")
        os.makedirs(self.save_dir, exist_ok=True)

        # Subscribers
        self.create_subscription(Image, 'scanner/image_raw', self.image_callback, 10)
        self.create_subscription(Bool, 'capture_trigger', self.trigger_callback, 10)

        self.get_logger().info("Node started. Publish 'true' to /capture_trigger to start image capture.")

    def image_callback(self, msg):
        """Callback to store the latest image from the topic."""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")

    def trigger_callback(self, msg: Bool):
        """Callback to start image acquisition when trigger is received."""
        if msg.data:
            threading.Thread(target=self.capture_sequence, daemon=True).start()

    def capture_sequence(self):
        """Captures a sequence of 10 images."""
        if self.capturing:
            self.get_logger().warn("Capture already in progress.")
            return

        if self.current_image is None:
            self.get_logger().warn("No image available. Check if the camera is publishing.")
            return

        self.capturing = True
        self.get_logger().info("Starting capture of 10 images...")
        session_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        for i in range(10):
            if self.current_image is not None:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                filename = f"image_{session_timestamp}_{i+1:02d}_{timestamp}.jpg"
                filepath = os.path.join(self.save_dir, filename)
                cv2.imwrite(filepath, self.current_image)
                self.get_logger().info(f"Saved {i+1}/10: {filename}")
            time.sleep(1)

        self.get_logger().info("Image capture completed.")
        self.capturing = False
        
def main(args=None):
    rclpy.init(args=args)
    node = ImageCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# ros2 topic pub --once /capture_trigger std_msgs/Bool "data: true"