#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import csv
import numpy as np


class PointSaver(Node):
    def __init__(self):
        super().__init__('point_saver')
        self.subscription = self.create_subscription(
            PointStamped,
            'clicked_point',
            self.clicked_point_callback,
            10)
        self.log_dir = '/sim_ws/src/f1tenth_lab5/logs/'
        self.waypoints = []
        self.density = 10

    def clicked_point_callback(self, msg):
        point = msg.point
        self.waypoints.append([point.x, point.y])

        if len(self.waypoints) > 1:
            self.interpolate_and_save()

        self.get_logger().info('Point saved: {}, {}'.format(point.x, point.y))

    def interpolate_and_save(self):
        dense_points = self.interpolate_points(self.waypoints)
        with open(self.log_dir+'waypoints_clicked.csv', mode='w') as file:
            writer = csv.writer(file)
            for point in dense_points:
                writer.writerow([point[0], point[1], 0.0, 0.0])


    def interpolate_points(self, points):
        dense_points = []
        x_coords, y_coords = zip(*points)
        poly_degree = 2  # Degree of the polynomial

        for i in range(len(points) - 1):
            x_start, y_start = points[i]
            x_end, y_end = points[i + 1]

            # Polynomial interpolation
            poly_coefficients_x = np.polyfit(
                [x_start, x_end], [y_start, y_end], poly_degree)
            poly_coefficients_y = np.polyfit(
                [y_start, y_end], [x_start, x_end], poly_degree)

            # Generate dense x values
            dense_x = np.linspace(x_start, x_end, self.density)

            # Calculate corresponding y values using polynomial
            dense_y = np.polyval(poly_coefficients_x, dense_x)

            # Calculate corresponding x values using polynomial
            dense_x_from_y = np.polyval(poly_coefficients_y, dense_y)

            dense_points.extend(np.column_stack((dense_x_from_y, dense_y)))

        return np.array(dense_points)


def main(args=None):
    rclpy.init(args=args)
    point_saver = PointSaver()
    rclpy.spin(point_saver)
    point_saver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
