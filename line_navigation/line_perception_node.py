import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32


# HSV range for the blue floor line
LOWER_BLUE = np.array([100, 50, 50])
UPPER_BLUE = np.array([130, 255, 255])

# Close ROI controls normal line following.
# Smaller number sees farther ahead. Larger number sees closer to robot.
ROI_TOP_FRACTION = 0.43

# Ignore tiny blue specks
MIN_AREA_TRACK = 50

# Debug display
SHOW_DEBUG_WINDOW = True


class LinePerceptionNode(Node):
    def __init__(self):
        super().__init__('line_perception_node')

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.error_pub = self.create_publisher(Float32, '/line_error', 10)
        self.found_pub = self.create_publisher(Bool, '/line_found', 10)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blue_mask = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)

        height, width = blue_mask.shape

        roi_top = int(height * ROI_TOP_FRACTION)
        roi_mask = blue_mask[roi_top:height, 0:width]

        line = self.get_contour_data(roi_mask)

        found_msg = Bool()
        error_msg = Float32()

        debug_image = cv2.bitwise_and(frame, frame, mask=blue_mask)
        cv2.rectangle(debug_image, (0, roi_top), (width - 1, height - 1), (0, 255, 255), 2)

        if line:
            # Convert ROI-local y back to full-image y for drawing only.
            line['y'] += roi_top

            error = float(line['x'] - width // 2)

            found_msg.data = True
            error_msg.data = error

            cv2.circle(debug_image, (line['x'], line['y']), 5, (0, 0, 255), 7)
        else:
            found_msg.data = False
            error_msg.data = 0.0

        self.error_pub.publish(error_msg)
        self.found_pub.publish(found_msg)

        if SHOW_DEBUG_WINDOW:
            cv2.imshow('Line Perception Debug', debug_image)
            cv2.waitKey(1)

    def get_contour_data(self, mask):
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_NONE
        )

        if not contours:
            return {}

        valid_contours = [
            contour for contour in contours
            if cv2.contourArea(contour) > MIN_AREA_TRACK
        ]

        if not valid_contours:
            return {}

        largest_contour = max(valid_contours, key=cv2.contourArea)
        moment = cv2.moments(largest_contour)

        if moment['m00'] == 0:
            return {}

        return {
            'x': int(moment['m10'] / moment['m00']),
            'y': int(moment['m01'] / moment['m00'])
        }


def main(args=None):
    rclpy.init(args=args)
    node = LinePerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
