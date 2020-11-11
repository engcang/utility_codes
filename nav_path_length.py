#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Aug 22 00:02:30 2020

@author: mason
"""

''' import libraries '''
import time
import numpy as np
from math import sqrt,pow

import rospy
from nav_msgs.msg import Path

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
        self.my_pose = rospy.Subscriber('/trajectory_mav_mbp', Path, self.my_callback)
        self.first=0
        self.leng=0

    def my_callback(self, msg):
        for i in range(len(msg.poses)):
            if self.first==0:
                self.first=1
                self.px = 0
                self.py = 0
                self.pz = 0
            else :
                self.leng = self.leng + sqrt((msg.poses[i].pose.position.x-self.px)**2 + (msg.poses[i].pose.position.y-self.py)**2 + (msg.poses[i].pose.position.z-self.pz)**2)
                self.px = msg.poses[i].pose.position.x
                self.py = msg.poses[i].pose.position.y
                self.pz = msg.poses[i].pose.position.z
        print("length : %.2f", self.leng)
        self.leng=0

''' main '''
path_pub_class = path_pub()

if __name__ == '__main__':
    while 1:
        try:
            pass
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
        # except:
        #     print("exception")
        #     pass
