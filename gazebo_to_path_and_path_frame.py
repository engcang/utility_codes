#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue May 19 00:28:30 2020

@author: mason
"""

''' import libraries '''
import time
import numpy as np

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped
from gazebo_msgs.msg import ModelStates

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


''' class '''

class path_pub():
    def __init__(self):
        rospy.init_node('path_pubb', anonymous=True)
        self.parent_frame_id = rospy.get_param("/parent_frame_id", '/world')
        self.model_name = rospy.get_param("/ModelStates", 'm100')
        self.out_topic_name = rospy.get_param("/out_topic_name", '/my_path')
        self.append_rate = rospy.get_param("/append_rate", 10)
        self.my_pose = rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        self.my_path_pub = rospy.Publisher(self.out_topic_name, Path, queue_size=1)
        self.lc_path_sub = rospy.Subscriber('/loop_fusion/pose_graph_path', Path, self.path_callback)
        self.lc_path_pub = rospy.Publisher('/lc/path', Path, queue_size=1)

        self.rate = rospy.Rate(self.append_rate)
        self.my_path = Path()
        self.pose = Pose()
        self.pose_offset = Pose()
        self.check = 0

    def gazebo_callback(self, msg):
        for i in range(len(msg.name)):
            if msg.name[i] == self.model_name:
                self.pose = msg.pose[i]
                if self.check == 0 :
                    self.pose_offset = self.pose
                self.check = 1

    def path_callback(self, msg):
        msg.header.frame_id = self.parent_frame_id
        for i in range(len(msg.poses)):
            msg.poses[i].header.frame_id = self.parent_frame_id
        self.lc_path_pub.publish(msg)


''' main '''
path_pub_class = path_pub()

if __name__ == '__main__':
    while 1:
        try:
            if path_pub_class.check == 1:
                pose = PoseStamped()
                pose.pose.position.x = path_pub_class.pose.position.x - path_pub_class.pose_offset.position.x
                pose.pose.position.y = path_pub_class.pose.position.y - path_pub_class.pose_offset.position.y
                pose.pose.position.z = path_pub_class.pose.position.z - path_pub_class.pose_offset.position.z
                pose.pose.orientation.x = path_pub_class.pose.orientation.x
                pose.pose.orientation.y = path_pub_class.pose.orientation.y
                pose.pose.orientation.z = path_pub_class.pose.orientation.z
                pose.pose.orientation.w = path_pub_class.pose.orientation.w
                pose.header.frame_id = path_pub_class.parent_frame_id
                pose.header.stamp = rospy.Time.now()
                path_pub_class.my_path.header.frame_id=path_pub_class.parent_frame_id
                path_pub_class.my_path.poses.append(pose)
                path_pub_class.my_path.header.stamp = rospy.Time.now()
                path_pub_class.my_path_pub.publish(path_pub_class.my_path)

                path_pub_class.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
        # except:
        #     print("exception")
        #     pass
