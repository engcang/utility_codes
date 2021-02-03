#!/usr/bin/env python
"""
Created on Thu Jan 21 04:52:04 2021
@author: EungChang Mason Lee
"""

import serial
import time

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import sys
import signal
def signal_handler(signal, frame):
    print('Pressed Ctrl+C')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


''' class '''
class serial_to_NUC():
    def __init__(self):
        rospy.init_node('serial_connector', anonymous=True)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.rate = rospy.Rate(5)
        self.ser = serial.Serial('/dev/ttyUSB0', 19200, timeout=1)

    def odom_callback(self, msg):
        if self.ser.is_open:
            data = 's,%.2f,%.2f,%.2f,%.6f,%.6f,%.6f,%.6f,'%(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
            while len(data)<80:
                data= data+'x'
            self.ser.write(data)
                

if __name__== '__main__':
    ser_nuc = serial_to_NUC()
    while 1:
        try:
            if ser_nuc.ser.is_open:
                d=ser_nuc.ser.read(2)
                print(d)
            ser_nuc.rate.sleep()
        except (SystemExit, KeyboardInterrupt):
            ser.close()
            sys.exit(0)
