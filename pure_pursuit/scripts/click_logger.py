#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import csv


class PointSaver(Node):
    def __init__(self):
        super().__init__('point_saver')
        self.subscription = self.create_subscription(
            PointStamped,
            'clicked_point',
            self.clicked_point_callback,
            10)
        self.log_dir = '/sim_ws/src/f1tenth_lab5/logs/'

    def clicked_point_callback(self, msg):
        point = msg.point
        with open(self.log_dir+'waypoints_clicked.csv', mode='a') as file:
            writer = csv.writer(file)
            writer.writerow([point.x, point.y, 0.00000, 0.00000])
        self.get_logger().info('Point saved: {}, {}'.format(point.x, point.y))


def main(args=None):
    rclpy.init(args=args)
    point_saver = PointSaver()
    rclpy.spin(point_saver)
    point_saver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
