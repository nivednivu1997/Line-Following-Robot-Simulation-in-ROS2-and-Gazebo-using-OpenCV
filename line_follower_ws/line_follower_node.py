import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class LineFollowerPID(Node):
    def __init__(self):
        super().__init__('line_follower_pid')

        # Subscriptions and publishers
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/intel_realsense_r200_depth/image_raw', self.image_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # PID parameters
        self.kp = 0.00275     # proportional gain
        self.ki = 0.0002      # integral gain (can tune later)
        self.kd = 0.002     # derivative gain

        # PID variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()

        # Image crop parameters
        self.crop_top = 0.6   # take bottom 40% of image
        self.linear_speed = 0.12

    def image_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w, _ = cv_img.shape

        # Focus only on lower part of the image
        crop = cv_img[int(h*self.crop_top):h, :]
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 70, 255, cv2.THRESH_BINARY_INV)

        # Morphological ops for clean line detection
        thresh = cv2.erode(thresh, np.ones((3,3), np.uint8))
        thresh = cv2.dilate(thresh, np.ones((3,3), np.uint8))

        # Moments to find line centroid
        M = cv2.moments(thresh)
        twist = Twist()
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
           # error = cx - (w / 2)
            desired_center = w * 0.55  # shift center 5% to right
            error = (cx - desired_center)
            # PID calculations
            current_time = time.time()
            dt = current_time - self.prev_time
            if dt == 0:
                dt = 0.01

            self.integral += error * dt
            derivative = (error - self.prev_error) / dt

            control = self.kp * error + self.ki * self.integral + self.kd * derivative

            # Update history
            self.prev_error = error
            self.prev_time = current_time

            # Move forward and adjust angular velocity
            twist.linear.x = self.linear_speed
            twist.angular.z = -float(control)

            # Optional debug info
            self.get_logger().info(f"Error: {error:.2f}, Control: {control:.3f}")
        else:
            # If no line detected, spin slowly to find it
            twist.linear.x = 0.0
            twist.angular.z = 0.3

        self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    node = LineFollowerPID()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
