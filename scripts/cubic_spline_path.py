#!/usr/bin/env python3
#-*- coding: utf-8 -*-
import sys
import os
import yaml
import numpy as np
from scipy.interpolate import interp1d

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class CubicSplinePath(Node):
    def __init__(self):
        super().__init__('cubic_spline_path')

        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.path_publisher = self.create_publisher(Path, 'spline_path', qos_profile)
        filename = self.declare_parameter('filename', '').value
        waypoints = yaml.load(open(filename))
        waypoint_x, waypoint_y = self.interpolation(waypoints['waypoints'])

        path = Path()
        for index in range(len(waypoint_x)):
            pose = PoseStamped()
            pose.pose.position.x = waypoint_x[index]
            pose.pose.position.y = waypoint_y[index]
            path.poses.append(pose)
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher.publish(path)

    def interpolation(self, waypoint_list, alg="cubic"):
        ix = iy = []

        temp_x = temp_y = np.array([])
        for waypoint in waypoint_list:
            temp_x = np.append(temp_x, waypoint['position']['x'])
            temp_y = np.append(temp_y, waypoint['position']['y'])
        cubic_spline = None
        if alg == "linear":
            cubic_spline = interp1d(temp_x, temp_y)
        elif alg == "cubic":
            cubic_spline = interp1d(temp_x, temp_y, kind='cubic')

        waypoint_x_start = temp_x[0]
        waypoint_x_end = temp_x[-1]
        length =  (int)(abs(waypoint_x_end - waypoint_x_start)/0.01)

        ix = np.linspace(waypoint_x_start, waypoint_x_end , num=length)
        iy = cubic_spline(ix)

        return ix, iy

def main():
    rclpy.init()
    node = CubicSplinePath()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
