#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

def path_length_callback(path):
    if len(path.poses) < 2:
        rospy.logwarn("Path is too short to compute length")
        return

    total_length = 0.0
    prev_pose = path.poses[0].pose

    for pose_stamped in path.poses[1:]:
        pose = pose_stamped.pose
        distance = math.sqrt((pose.position.x - prev_pose.position.x)**2 +
                             (pose.position.y - prev_pose.position.y)**2 +
                             (pose.position.z - prev_pose.position.z)**2)
        total_length += distance
        prev_pose = pose

    rospy.loginfo("Path length: {:.2f} meters".format(total_length))

rospy.init_node('path_length_calculator')
rospy.Subscriber('/amcl_tiago_path1', Path, path_length_callback)
rospy.spin()
