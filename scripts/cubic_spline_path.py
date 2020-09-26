#!/usr/bin/env python
#-*- coding: utf-8 -*-
import rospy
import rosparam
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import sys
import os
import yaml
import numpy as np
from scipy.interpolate import interp1d


def interpolation(waypoint_list, alg="cubic"):
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
    length = (int)(abs(waypoint_x_end - waypoint_x_start)/0.01)
    print length
    ix = np.linspace(waypoint_x_start, waypoint_x_end, num=length)
    iy = cubic_spline(ix)

    return ix, iy


if __name__ == '__main__':
    rospy.init_node("cubic_spline_path")

    filename = rospy.get_param('~filename')
    file = open(filename, 'r')
    waypoints = yaml.load(file)

    waypoint_x, waypoint_y = interpolation(waypoints['waypoints'])

    path_pub = rospy.Publisher('/spline_path', Path, queue_size=10)

    path = Path()
    for index in range(len(waypoint_x)):
        pose = PoseStamped()
        pose.pose.position.x = waypoint_x[index]
        pose.pose.position.y = waypoint_y[index]
        print pose
        path.poses.append(pose)
    path.header.frame_id = "map"
    path.header.stamp = rospy.Time.now()

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        path_pub.publish(path)
        rate.sleep()
    rospy.spin()
