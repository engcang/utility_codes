#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue May 19 00:28:30 2020

@author: mason
"""

''' import libraries '''
import time

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped


import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!, GT pub ends')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


''' class '''

class gt_pub():
    def __init__(self):
        rospy.init_node('gt_pub', anonymous=True)
        # self.append_rate = rospy.get_param("~append_rate", 5)
        self.my_pose = rospy.Subscriber("/gazebo/model_states", ModelStates, self.gt_cb)
        self.my_gt_pub_list = []
        self.rate = rospy.Rate(1)
        self.check = 0

    def gt_cb(self, msg):
        if self.check == 0:
            for i in range(len(msg.name)):
                self.my_gt_pub_list.append(rospy.Publisher("/gt/"+msg.name[i], PoseStamped, queue_size=1))
            self.check = 1
        else:
            for i in range(len(msg.name)):
                data = PoseStamped()
                data.pose = msg.pose[i]
                data.header.stamp = rospy.Time.now()
                self.my_gt_pub_list[i].publish(data)

''' main '''
gt_pub_class = gt_pub()

if __name__ == '__main__':
    while 1:
        try:
                gt_pub_class.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
        # except:
        #     print("exception")
        #     pass
