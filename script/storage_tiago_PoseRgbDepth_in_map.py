#!/usr/bin/env python

import rospy
import os
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import numpy as np
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
import math
import tf.transformations as tft
import tf_lookup.srv as tls
import sys 
import select 

'''
This program gets the true pose of tiago in gazebo frame. Therefore, we can get the true moving trajectory during mapping or navigation.

This grogram can get true pose of any model in gazebo frame.

Use with program show_tiago_storage_path.py to show storaged poses.

We can get the true moving trajectory of robot in gazebo frame. But I don't known how to get the relationship between gazebo frame and map frame. 
So, I can't draw the true trajectory on the map
'''
class storage_model_states():
    def __init__(self):
        self.num = 0
        self.name = raw_input('file name:')
        self._filename ='PoseRgbDepthSave/' + self.name + '.txt'
        print 'file name is ' + self.name
        print 'position.x position.y position.z orientation.x orientation.y orientation.z orientation.w'
        print self._filename
        self.f = open(self._filename, 'w')
        rospy.init_node('storage_tiago_PoseRgbDepth_in_gazebo', anonymous=True)
        self.start_time = rospy.get_time()
        rospy.on_shutdown(self.shutdown)
        self.tf_lookup = rospy.ServiceProxy('/lookupTransform', tls.lookupTransform, persistent=True)
        rate = rospy.Rate(20) # 10hz

        self.depth_image_topic = "/xtion/depth_registered/image_raw"
        self.image_topic = "/xtion/rgb/image_raw"

        self.counterrgb = 1
        self.counterdepth = 1

        while not rospy.is_shutdown():
            if sys.stdin in select.select([sys.stdin],[],[],0)[0]:
                line = sys.stdin.readline()
                if line.strip() == 's':
                    self.storage_robot_pose()
                    self.depth_sub = rospy.Subscriber(self.depth_image_topic, Image, self.depth_image_callback)
                    self.rgb_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
                    rospy.loginfo("saved")
            rate.sleep()
            # print 99 
            # self.storage_robot_pose()
        # The frenqucy of topic /gazebo/model_states is 1000Hz
        rospy.spin()

    def depth_image_callback(self, msg):
        bridge = CvBridge()
        cv_depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding = "passthrough")
        depth_array = np.array(cv_depth_image, dtype = np.float32)
        cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)

        pathdepth = "PoseRgbDepthSave/saved_depth{}.png".format(self.counterdepth)
        cv2.imwrite(pathdepth, depth_array * 255)

        self.counterdepth += 1
        self.depth_sub.unregister()


    def image_callback(self, msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding = "bgr8")

        pathrgb = "PoseRgbDepthSave/saved_rgb{}.png".format(self.counterrgb)
        cv2.imwrite(pathrgb,cv_image)

        self.counterrgb += 1
        self.rgb_sub.unregister()

    def storage_robot_pose(self):
        map_frame = "/map"
        base_frame = "/base_footprint"
        
        # During localization (amcl running) we have the 'amcl_pose'
        # (including the covariance).
        # On the other side, during mapping we have the TF (/map -> /base_footprint).
        try:
            trans = self.tf_lookup(map_frame, base_frame, rospy.Time.from_sec(0))
        except rospy.ServiceException as exc:
            rospy.logwarn("Could not get tf from " + map_frame + " to " + base_frame + ": " + str(exc))
            return
        transform = trans.transform.transform
        pose = Pose(transform.translation, transform.rotation)
        data = '{}'.format(rospy.get_rostime().secs+rospy.get_rostime().nsecs/1000000000.0-self.start_time) + '\t' + '{}'.format(pose.position.x) + '\t' +'{}'.format(pose.position.y) + '\t' +'{}'.format(pose.position.z) + '\t' + \
                '{}'.format(pose.orientation.x) + '\t' +'{}'.format(pose.orientation.y) + '\t' + '{}'.format(pose.orientation.z) + \
                    '\t' +'{}'.format(pose.orientation.w) + '\n'
        self.f.write(data)

    def shutdown(self):
        self.f.close()
        # Always stop the robot when shutting down the node.
        rospy.loginfo("end...")

    def callback(self, data):
        if self.num == 150:
            self.num=0
            model_index = -1
            model_index = data.name.index("tiago")
            if model_index == -1:
                rospy.loginfo("cann't find the model")
            else:
                pose = data.pose[model_index]
                data = '{}'.format(pose.position.x) + '\t' +'{}'.format(pose.position.y) + '\t' +'{}'.format(pose.position.z) + '\t' + \
                     '{}'.format(pose.orientation.x) + '\t' +'{}'.format(pose.orientation.y) + '\t' + '{}'.format(pose.orientation.z) + \
                         '\t' +'{}'.format(pose.orientation.w) + '\n'
                self.f.write(data)
        self.num = self.num +1

if __name__ == '__main__':
    try:
        storage_model_states()
    except rospy.ROSInterruptException:
        pass
