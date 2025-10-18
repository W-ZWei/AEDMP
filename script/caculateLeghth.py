#!/usr/bin/env python

import rospy
import os
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class trajectory():
    def __init__(self):
        self._filename = '/home/service-sdu/YJF/472/5c.txt'
        rospy.init_node('show_storaged_trajectory', anonymous=True)

        self.topic = rospy.Publisher("/gazebo_tiago_path2", Path, queue_size=1)
        self.path = Path()
        self.path.header.frame_id = "/map"
        self.path.header.stamp = rospy.get_rostime()

        self.path_length = 0.0 
        self.fill_data()

    def pub(self):
        self.topic.publish(self.path)

    def fill_data(self):
        f = open(self._filename, 'r')
        sourceInLine = f.readlines()
        dataset = []
        po = []

        for line in sourceInLine:
            temp1 = line.strip('\n')
            temp2 = temp1.split('\t')
            dataset.append(temp2)

        print(len(dataset))
        print(dataset[203][6])

        prev_pose = None  

        for i in xrange(0, len(dataset)):
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

    
            if prev_pose is not None:
              
                distance = self.calculate_distance(prev_pose, posestamp)
                self.path_length += distance

    
            prev_pose = posestamp

            self.path.poses.append(posestamp)
            po = []

        f.close()
        print("Total path length: {:.4f} meters".format(self.path_length)) 

    def calculate_distance(self, pose1, pose2):
      
        dx = pose2.pose.position.x - pose1.pose.position.x
        dy = pose2.pose.position.y - pose1.pose.position.y
        dz = pose2.pose.position.z - pose1.pose.position.z
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def shutdown(self):
        rospy.loginfo("end...")

if __name__ == '__main__':
    try:
        t = trajectory()
        rate = rospy.Rate(10)  
        while not rospy.is_shutdown():
            t.pub()
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("end.")
