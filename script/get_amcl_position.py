#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped
import os

data = '' 

def model_states_callback(msg):
    global data
    try:
        # index = msg.name.index("tiago")
        # pose = msg.pose[index]
        
        data = '{}'.format(msg.pose.pose.position.x) + '\t' + '{}'.format(msg.pose.pose.position.y) + \
        '\t''{}'.format(msg.pose.pose.position.z) + '\t''{}'.format(msg.pose.pose.orientation.x) + \
        '\t''{}'.format(msg.pose.pose.orientation.y) + '\t''{}'.format(msg.pose.pose.orientation.z) + \
        '\t''{}'.format(msg.pose.pose.orientation.w) + '\n'



    except ValueError:
        pass

def write():
    global data
    with open("/home/service-sdu/YJF/TEST/amcl_position1.txt", "a") as file:
        file.write(data)


if __name__ == "__main__":
    rospy.init_node("amcl_position_saver", anonymous=True)
    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, model_states_callback)
    # rospy.wait_for_service("/gazebo/get_model_state")
    # get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        write()
        rate.sleep()
