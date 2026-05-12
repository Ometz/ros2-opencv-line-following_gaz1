import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32


# HSV range for the blue floor line
LOWER_BLUE = np.array([100, 50, 50])
UPPER_BLUE = np.array([130, 255, 255])

# Lookahead zone for branch decisions only.
LOOK_TOP_FRACTION = 0.40
LOOK_BOTTOM_FRACTION = 0.70

# Branch decision tuning.
# Conservative values because this node must not yank the robot off the line.
CENTER_WEAK = 80
STORE_CENTER_MAX = 600
SIDE_DIFF = 1200
COMMIT_DISTANCE = 0.90
MAX_BRANCH_MEMORY_DISTANCE = 1.50
LINE_ERROR_ASSIST_THRESHOLD = 20.0
BRANCH_ANGULAR_BIAS = 0.05

SHOW_DEBUG_WINDOW = False


class BranchDecisionNode(Node):
    def __init__(self):
        super().__init__('branch_decision_node')

        self.bridge = CvBridge()

        self.odom_x = 0.0
        self.odom_y = 0.0

        self.line_error = 0.0
        self.line_found = False

        self.pending_branch = None
        self.branch_seen_x = 0.0
        self.branch_seen_y = 0.0

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.error_sub = self.create_subscription(
            Float32,
            '/line_error',
            self.error_callback,
            10
        )

        self.found_sub = self.create_subscription(
            Bool,
            '/line_found',
            self.found_callback,
            10
        )

        self.bias_pub = self.create_publisher(Float32, '/branch_bias', 10)

    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

    def error_callback(self, msg):
        self.line_error = float(msg.data)

    def found_callback(self, msg):
        self.line_found = bool(msg.data)

    def image_callback(self, msg):
        bias_msg = Float32()
        bias_msg.data = 0.0

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blue_mask = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)

        height, width = blue_mask.shape

        look_top = int(height * LOOK_TOP_FRACTION)
        look_bottom = int(height * LOOK_BOTTOM_FRACTION)
        look_mask = blue_mask[look_top:look_bottom, 0:width]

        left_pixels = cv2.countNonZero(look_mask[:, 0:width // 3])
        center_pixels = cv2.countNonZero(look_mask[:, width // 3:2 * width // 3])
        right_pixels = cv2.countNonZero(look_mask[:, 2 * width // 3:width])

        # Store one candidate branch at a time. Do not reset odom every frame.
        if self.pending_branch is None and center_pixels < STORE_CENTER_MAX:
            if left_pixels > right_pixels + SIDE_DIFF:
                self.pending_branch = 'left'
                self.branch_seen_x = self.odom_x
                self.branch_seen_y = self.odom_y
                self.get_logger().info('Stored branch candidate: LEFT')

            elif right_pixels > left_pixels + SIDE_DIFF:
                self.pending_branch = 'right'
                self.branch_seen_x = self.odom_x
                self.branch_seen_y = self.odom_y
                self.get_logger().info('Stored branch candidate: RIGHT')

        dx = self.odom_x - self.branch_seen_x
        dy = self.odom_y - self.branch_seen_y
        dist_since_seen = math.hypot(dx, dy)

        if self.pending_branch is not None and dist_since_seen > MAX_BRANCH_MEMORY_DISTANCE:
            self.get_logger().info('Clearing stale branch memory')
            self.pending_branch = None

        # Important safety rule:
        # branch bias can only ASSIST a turn the line follower is already requesting.
        if (
            self.pending_branch is not None
            and self.line_found
            and center_pixels < CENTER_WEAK
            and dist_since_seen > COMMIT_DISTANCE
        ):
            if self.pending_branch == 'left' and self.line_error < -LINE_ERROR_ASSIST_THRESHOLD:
                bias_msg.data = BRANCH_ANGULAR_BIAS
                self.get_logger().info('Assisting LEFT turn')

            elif self.pending_branch == 'right' and self.line_error > LINE_ERROR_ASSIST_THRESHOLD:
                bias_msg.data = -BRANCH_ANGULAR_BIAS
                self.get_logger().info('Assisting RIGHT turn')

        self.bias_pub.publish(bias_msg)

        if SHOW_DEBUG_WINDOW:
            debug_image = cv2.bitwise_and(frame, frame, mask=blue_mask)
            cv2.rectangle(debug_image, (0, look_top), (width - 1, look_bottom), (255, 0, 255), 2)
            cv2.imshow('Branch Decision Debug', debug_image)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = BranchDecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
