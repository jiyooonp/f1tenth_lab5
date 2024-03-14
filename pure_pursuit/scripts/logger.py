#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import atexit
from os.path import expanduser, exists
from os import makedirs
from time import gmtime, strftime
from numpy import linalg as LA
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry


class WaypointsLogger(Node):

    def __init__(self):
        super().__init__('waypoints_logger')
        self.subscription = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.save_waypoint,
            10)

        log_dir = '/sim_ws/src/f1tenth_lab5/logs/'
        if not exists(log_dir):
            makedirs(log_dir)
        self.file = open(
            log_dir + strftime('wp-%Y-%m-%d-%H-%M-%S.csv', gmtime()), 'w')

    def save_waypoint(self, data):
        pose = data.pose.pose
        twist = data.twist.twist

        quaternion = np.array([pose.orientation.x,
                               pose.orientation.y,
                               pose.orientation.z,
                               pose.orientation.w])
        euler = self.quaternion_to_euler(quaternion)
        speed = LA.norm(np.array([twist.linear.x,
                                  twist.linear.y,
                                  twist.linear.z]), 2)

        if twist.linear.x > 0.:
            print(twist.linear.x)

        self.file.write('%f, %f, %f, %f\n' % (pose.position.x,
                                              pose.position.y,
                                              euler[2],
                                              speed))

    def quaternion_to_euler(self, quaternion):
        x, y, z, w = quaternion
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def shutdown(self):
        self.file.close()
        print('Goodbye')


def main(args=None):
    rclpy.init(args=args)
    waypoints_logger = WaypointsLogger()
    print('Saving waypoints...')
    try:
        rclpy.spin(waypoints_logger)
    except KeyboardInterrupt:
        waypoints_logger.shutdown()
        waypoints_logger.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
