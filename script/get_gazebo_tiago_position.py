#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
import os

data = '' 

def model_states_callback(msg):
    global data
    try:
        index = msg.name.index("tiago")
        pose = msg.pose[index]
        
        data = '{}'.format(pose.position.x) + '\t' + '{}'.format(pose.position.y) + \
        '\t''{}'.format(pose.position.z) + '\t''{}'.format(pose.orientation.x) + \
        '\t''{}'.format(pose.orientation.y) + '\t''{}'.format(pose.orientation.z) + \
        '\t''{}'.format(pose.orientation.w) + '\n'



    except ValueError:
        pass

def write():
    global data
    with open("/home/service-sdu/YJF/472/5c.txt", "a") as file:
        file.write(data)


if __name__ == "__main__":
    rospy.init_node("model_position_saver")
    rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)
    # rospy.wait_for_service("/gazebo/get_model_state")
    # get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        write()
        rate.sleep()

