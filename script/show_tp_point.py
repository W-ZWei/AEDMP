#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

previous_points = []

def points_callback(data):
    global previous_points

    p = data.point
    previous_points.append(p)

    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    marker.points = previous_points

    marker_pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node('point_visualizer', anonymous=True)
    rospy.Subscriber("/clicked_point", PointStamped, points_callback)
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rospy.spin()
