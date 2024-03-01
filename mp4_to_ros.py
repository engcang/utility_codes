#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time, sys, os
import roslib, rospy
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image
import numpy as np

from cv_bridge import CvBridge
import cv2

import sys
import signal
def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


def CreateVideoBag(videopath, TOPIC='usb_cam/image_raw'):
    rospy.init_node('bag', anonymous=True)
    pubimage = rospy.Publisher(TOPIC, Image, queue_size=20)

    cap = cv2.VideoCapture(videopath)
    cb = CvBridge()
    prop_fps = cap.get(cv2.CAP_PROP_FPS)
    if prop_fps != prop_fps or prop_fps <= 1e-2:
        print("Warning: can't get FPS. Assuming 30.")
        prop_fps = 30

    rate = rospy.Rate(prop_fps)
    ret = True
    frame_id = 0
    while(ret):
        ret, frame = cap.read()
        if not ret:
            break
        stamp = rospy.rostime.Time.from_sec(float(frame_id) / prop_fps)
        frame_id += 1
        #img = cv2.resize(frame, dsize=(1280,720), interpolation=cv2.INTER_LINEAR)
        #image = cb.cv2_to_imgmsg(img, encoding='bgr8')
        image = cb.cv2_to_imgmsg(frame, encoding='bgr8')
        image.header.stamp = stamp
        image.header.frame_id = "camera"
        pubimage.publish(image)
        rate.sleep()
    cap.release()

if __name__ == "__main__":
    if len( sys.argv ) == 2 or len( sys.argv ) == 3:
        CreateVideoBag(*sys.argv[1:])
    else:
        print( "Usage: video2ros video_file_name topic_name")
