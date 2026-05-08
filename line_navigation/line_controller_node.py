import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


# Base line-following behavior only. No branch logic in this node.
LINEAR_SPEED = 0.12
LOST_LINE_SPEED = 0.06
LOST_LINE_MAX_FRAMES = 20

# Light PD controller.
# P follows the line. D damps wobble.
KP = 2.8 / 100
KD = 0.00045
KI = 0.0

INTEGRAL_LIMIT = 100.0
ANGULAR_Z_LIMIT = 0.90

PUBLISH_HZ = 20.0
DERIVATIVE_FILTER_ALPHA = 0.65


class LineControllerNode(Node):
    def __init__(self):
        super().__init__('line_controller_node')

        self.line_error = 0.0

        # Last detected line error, used for brief lost-line recovery.
        self.last_error = 0.0

        # Previous control error, used for derivative calculation.
        self.previous_control_error = 0.0

        # Filtered derivative term to avoid reacting too hard to camera noise.
        self.filtered_derivative = 0.0

        # Integral term is included, but KI starts at 0.0.
        self.integral_error = 0.0

        self.line_found = False
        self.lost_line_frames = 0

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

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_line', 10)

        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.publish_cmd)

    def error_callback(self, msg):
        self.line_error = float(msg.data)

    def found_callback(self, msg):
        self.line_found = bool(msg.data)

    def publish_cmd(self):
        cmd = Twist()
        dt = 1.0 / PUBLISH_HZ

        if self.line_found:
            self.lost_line_frames = 0

            error = self.line_error
            self.last_error = error

            cmd.linear.x = LINEAR_SPEED

        else:
            self.lost_line_frames += 1

            if self.lost_line_frames < LOST_LINE_MAX_FRAMES:
                error = self.last_error
                cmd.linear.x = LOST_LINE_SPEED
            else:
                error = 0.0
                cmd.linear.x = 0.0

                # If the line is gone for too long, do not keep old PID memory.
                self.integral_error = 0.0
                self.filtered_derivative = 0.0
                self.previous_control_error = 0.0

        # Derivative: how fast the line error is changing.
        raw_derivative = (error - self.previous_control_error) / dt

        # Low-pass filter derivative to avoid steering twitch from pixel noise.
        self.filtered_derivative = (
            DERIVATIVE_FILTER_ALPHA * self.filtered_derivative
            + (1.0 - DERIVATIVE_FILTER_ALPHA) * raw_derivative
        )

        # Integral is included for completeness, but KI is 0.0 by default.
        self.integral_error += error * dt
        self.integral_error = max(
            min(self.integral_error, INTEGRAL_LIMIT),
            -INTEGRAL_LIMIT
        )

        control = (
            KP * error
            + KD * self.filtered_derivative
            + KI * self.integral_error
        )

        # Negative sign because positive pixel error means the line is right,
        # and negative angular.z turns the robot right.
        cmd.angular.z = -control

        cmd.angular.z = max(
            min(cmd.angular.z, ANGULAR_Z_LIMIT),
            -ANGULAR_Z_LIMIT
        )

        self.previous_control_error = error

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LineControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
