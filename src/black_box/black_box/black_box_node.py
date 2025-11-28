#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import cv2
import numpy as np

class BlackBox(Node):
    def __init__(self):
        super().__init__('black_box')
        self.window_name = "black_box"
        self.point = None
        self.declare_parameter('square_size', 200)
        self.square_size = self.get_parameter('square_size').get_parameter_value().integer_value
        self.publisher_ = self.create_publisher(Point, '/point', 10)
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.on_mouse_click)
        self.get_logger().info(f'BlackBox started with square_size={self.square_size}')

    def run(self):
        height = 512
        width = 700

        while rclpy.ok():
            img = np.zeros((height, width, 3), np.uint8)
            cv2.line(img, (0, height // 2), (width, height // 2), (100, 100, 100), 1)
            if self.point is not None:
                cv2.rectangle(img,
                              self.point,
                              (self.point[0] + self.square_size, self.point[1] + self.square_size),
                              (0, 255, 0), 3)
            cv2.imshow(self.window_name, img)
            cv2.waitKey(1)

    def on_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.point = (x, y)
            msg = Point()
            msg.x = float(x)
            msg.y = float(y)
            msg.z = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info(f'Clicked ({x},{y}) -> published /point')

def main(args=None):
    rclpy.init(args=args)
    node = BlackBox()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
