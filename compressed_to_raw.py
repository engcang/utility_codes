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
from sensor_msgs.msg import Imu
from cv_bridge import CvBridge, CvBridgeError


def signal_handler(signal, frame): # ctrl + c -> exit program
    print('You pressed Ctrl+C!')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


class converter():
    def __init__(self):
        rospy.init_node('compressed_to_raw', anonymous=True)
        self.comp_sub1 = rospy.Subscriber('/camera/infra1/image_rect_raw/compressed',CompressedImage,self.callback1)
        self.comp_sub2 = rospy.Subscriber('/camera/infra2/image_rect_raw/compressed',CompressedImage,self.callback2)
        self.imu_subscriber = rospy.Subscriber('/mavros/imu/data_raw', Imu, self.imu_callback)
        self.img_pub1 = rospy.Publisher('/camera/infra1/img',Image,queue_size=100)
        self.img_pub2 = rospy.Publisher('/camera/infra2/img',Image,queue_size=100)
        self.imu_publisher = rospy.Publisher('/imu_data', Imu, queue_size=100)
        self.bridge = CvBridge()

    def callback1(self, data):
        try : 
            np_arr = np.fromstring(data.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
            cv_image=cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
            img=self.bridge.cv2_to_imgmsg(cv_image, "mono8")
            img.header.stamp = data.header.stamp
#            img.header.stamp = rospy.Time.now()
            self.img_pub1.publish(img)
        except CvBridgeError as e:
            pass

    def callback2(self, data):
        try : 
            np_arr = np.fromstring(data.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
            cv_image=cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
            img=self.bridge.cv2_to_imgmsg(cv_image, "mono8")
            img.header.stamp = data.header.stamp
#            img.header.stamp = rospy.Time.now()
            self.img_pub2.publish(img)
        except CvBridgeError as e:
            pass

#    def callback(self,data):
#        try :
##		self.time = time.time()
#        cvimage=self.bridge.imgmsg_to_cv2(data,"bgr8")

#        cv_image=cv2.cvtColor(cvimage,cv2.COLOR_BGR2GRAY)

#        img=self.bridge.cv2_to_imgmsg(cv_image, "mono8")

#        img.header.stamp = rospy.Time.now()
#        self.img_publisher.publish(img)
#        #print(time.time()-self.time)
#    except CvBridgeError as e:
#        pass

    def imu_callback(self,data):
#        data.header.stamp = rospy.Time.now()
        new_data = data
        new_data.header.stamp = data.header.stamp
        self.imu_publisher.publish(data)

if __name__=='__main__':
    cvt=converter()
    time.sleep(1)
    while 1:
        pass
