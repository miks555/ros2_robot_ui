#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.get_logger().info("TurtleController started — waiting for /point messages")
        self.create_subscription(Point, '/point', self.point_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.window_center_y = 256

    def point_callback(self, msg):
        self.get_logger().info(f'Received point: ({msg.x:.1f}, {msg.y:.1f})')

        cmd = Twist()
        if msg.y < self.window_center_y:
            cmd.linear.x = 0.3
            self.get_logger().info('Point ABOVE center — moving forward 🚗')
        else:
            cmd.linear.x = 0.0
            self.get_logger().info('Point BELOW center — stopping 🛑')

        self.cmd_pub.publish(cmd)
        self.get_logger().info(f'Published /cmd_vel: linear.x={cmd.linear.x}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
