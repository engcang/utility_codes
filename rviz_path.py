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
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker

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
        self.target_pose = rospy.Subscriber('/gazebo/model_states', ModelStates, self.target_callback)
        self.my_pose = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.my_callback)
        self.target_path_pub = rospy.Publisher('/target_path', Path, queue_size=1)
        self.my_path_pub = rospy.Publisher('/my_path', Path, queue_size=1)
        self.geofence_pub = rospy.Publisher('/geofence', Marker, queue_size=1)

        self.rate = rospy.Rate(30)
        self.target_path = Path()
        self.my_path = Path()
        self.check_target = 0
        self.check_my = 0

    def target_callback(self, msg):
        self.check_target = 1
        for i in range(len(msg.name)):
            if msg.name[i]=='JinWoo':
                self.msg1 = msg.pose[i]

    def my_callback(self, msg):
        self.check_my = 1
        my_p = msg
        my_p.header.frame_id = 'map'
        my_p.header.stamp = rospy.Time.now()
        self.my_path.header.frame_id='map'
        self.my_path.poses.append(my_p)
        self.my_path.header.stamp = rospy.Time.now()

''' main '''
path_pub_class = path_pub()

if __name__ == '__main__':
    while 1:
        try:
            if path_pub_class.check_target == 1 and path_pub_class.check_my == 1:
                target_p = PoseStamped()
                target_p.pose.position.x = path_pub_class.msg1.position.x
                target_p.pose.position.y = path_pub_class.msg1.position.y
                target_p.pose.position.z = path_pub_class.msg1.position.z
                target_p.pose.orientation.x = path_pub_class.msg1.orientation.x
                target_p.pose.orientation.y = path_pub_class.msg1.orientation.y
                target_p.pose.orientation.z = path_pub_class.msg1.orientation.z
                target_p.pose.orientation.w = path_pub_class.msg1.orientation.w
                target_p.header.frame_id = 'map'
                target_p.header.stamp = rospy.Time.now()
                path_pub_class.target_path.header.frame_id='map'
                path_pub_class.target_path.poses.append(target_p)
                path_pub_class.target_path.header.stamp = rospy.Time.now()
                path_pub_class.target_path_pub.publish(path_pub_class.target_path)

                path_pub_class.my_path_pub.publish(path_pub_class.my_path)

                geo = Marker()
                geo.header.stamp = rospy.Time.now()
                geo.header.frame_id='map'
                geo.type=3
                geo.lifetime=rospy.Duration(1)
                geo.color.r=1
                geo.color.a=1
                geo.pose.position.x = 40
                geo.pose.position.y = 20
                geo.pose.position.z = 10
                geo.pose.orientation.w = 1.0
                

                geo.scale.x = 20.0
                geo.scale.y = 20.0
                geo.scale.z = 20.0
                path_pub_class.geofence_pub.publish(geo)

                path_pub_class.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
        # except:
        #     print("exception")
        #     pass
