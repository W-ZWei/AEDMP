#!/usr/bin/env python

import rospy
import os
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import tf.transformations as tft
# import copy
'''
This program reads the 2-D pose data from the file and publishes it to a topic, displaying the historical trajectory represented in the file in rviz.
This program can be regarded as just drawing coordinates without binding any frame.

Use with program storage_tiago_pose_in_gazebo.py to storage poses.
'''
class trajectory():
    def __init__(self):
        # self.name = raw_input('file name:')
        self._filename = '/home/service-sdu/YJF/TEST/amcl_position1.txt'
        rospy.init_node('show_storaged_trajectory_amcl', anonymous=True)

        self.topic = rospy.Publisher("/amcl_tiago_path1", Path, queue_size = 1)
        self.path = Path()
        self.path.header.frame_id = "/map"
        self.path.header.stamp = rospy.get_rostime()

        self.fill_data()

        

    def pub(self):

        self.topic.publish(self.path)
        # self.cmd_path.publish(self.path)


    def fill_data(self):

        f=open(self._filename,'r')
        sourceInLine = f.readlines()
        dataset=[]
        po = []    

        for line in sourceInLine:
            temp1 = line.strip('\n')
            temp2 = temp1.split('\t')

            dataset.append(temp2)

        for i in xrange(0,len(dataset)):

            posestamp = PoseStamped()
            posestamp.header.frame_id = "/map"

            for j in xrange(7):
                    
                po.append(float(dataset[i][j]))
            posestamp.pose.position.x = po[0]
            posestamp.pose.position.y = po[1]
            posestamp.pose.position.z = po[2]
            posestamp.pose.orientation.x = po[3]
            posestamp.pose.orientation.y = po[4]
            posestamp.pose.orientation.z = po[5]
            posestamp.pose.orientation.w = po[6]
            posestamp.header.stamp = rospy.get_rostime()
                # self.path.poses.append(copy.deepcopy(posestamp))
            self.path.poses.append(posestamp)
            po = []
        f.close()

    def shutdown(self):
        rospy.loginfo("end...")


if __name__ == '__main__':
    try:
        t = trajectory()
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            t.pub()
            rate.sleep()
    except rospy.ROSInterruptException:
        #self.f.close()
        rospy.loginfo("end.")


