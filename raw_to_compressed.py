#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rospy
import cv2
import numpy as np
import sys
import signal

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError


def signal_handler(signal, frame): # ctrl + c -> exit program
    print('You pressed Ctrl+C!')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


class converter():
    def __init__(self):
        rospy.init_node('compressed_to_raw', anonymous=True)
        self.comp_sub = rospy.Subscriber('/camera/fisheye1/image_raw', Image, self.callback)
        self.img_pub = rospy.Publisher('/camera/fisheye1/image_raw/compressed', CompressedImage, queue_size=10)
        self.bridge = CvBridge()

    def callback(self,data):
        try :
            cv_image=self.bridge.imgmsg_to_cv2(data,"bgr8")
            img=self.bridge.cv2_to_compressed_imgmsg(cv_image)
            # img.header.stamp = rospy.Time.now()
            img.header.stamp = data.header.stamp
            self.img_pub.publish(img)
        except CvBridgeError as e:
            pass


if __name__=='__main__':
    cvt=converter()
    time.sleep(1)
    while 1:
        pass
