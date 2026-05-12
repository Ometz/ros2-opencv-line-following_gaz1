import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32


ANGULAR_Z_LIMIT = 0.90
PUBLISH_HZ = 20.0


class CmdArbiterNode(Node):
    def __init__(self):
        super().__init__('cmd_arbiter_node')

        self.line_cmd = Twist()
        self.branch_bias = 0.0

        self.line_cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel_line',
            self.line_cmd_callback,
            10
        )

        self.branch_bias_sub = self.create_subscription(
            Float32,
            '/branch_bias',
            self.branch_bias_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.publish_final_cmd)

    def line_cmd_callback(self, msg):
        self.line_cmd = msg

    def branch_bias_callback(self, msg):
        self.branch_bias = float(msg.data)

    def publish_final_cmd(self):
        cmd = Twist()
        cmd.linear.x = self.line_cmd.linear.x
        cmd.angular.z = self.line_cmd.angular.z + self.branch_bias
        cmd.angular.z = max(min(cmd.angular.z, ANGULAR_Z_LIMIT), -ANGULAR_Z_LIMIT)

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CmdArbiterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
