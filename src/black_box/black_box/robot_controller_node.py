#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )
        self.current_joint_positions = [0.0] * 6
        self.height = 512
        self.up_position = math.radians(-70)
        self.down_position = math.radians(0)
        self.subscription = self.create_subscription(Point, '/point', self.point_callback, 10)

    def point_callback(self, msg):
        if msg.y < self.height / 2:
            self.move_up() 
        else:
            self.move_down()

    def move_up(self):
        self.get_logger().info('Moving up to 70 degrees')
        self.send_joint_command(self.up_position)

    def move_down(self):
        self.get_logger().info('Moving down to 0 degrees')
        self.send_joint_command(self.down_position)

    def send_joint_command(self, target_pos):
        msg = JointTrajectory()
        msg.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        point = JointTrajectoryPoint()
        new_positions = self.current_joint_positions.copy()
        if len(new_positions) < 6:
            new_positions += [0.0] * (6 - len(new_positions))
        new_positions[1] = target_pos 

        point.positions = new_positions
        point.time_from_start.sec = 2
        msg.points.append(point)
        self.pub.publish(msg)
        self.current_joint_positions = new_positions

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
