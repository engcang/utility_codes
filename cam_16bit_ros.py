import cv2
import numpy as np
import time
import math

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

import signal
import sys

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


def img_to_8bit_img(bit_img):
    minVal = np.amin(bit_img); maxVal = np.amax(bit_img)
    it_img = bit_img - minVal
    it_img = it_img / float(maxVal-minVal) * 255.0
    return it_img.astype('uint8') 

class cam():
    def __init__(self):
        rospy.init_node('cam_controller', anonymous=True)
        self.img_publisher = rospy.Publisher('/thermal/image_raw', Image, queue_size=1)
        self.img_publisher2 = rospy.Publisher('/thermal/image_raw/compressed', CompressedImage, queue_size=1)
        self.camera_id = rospy.get_param("/cam_id", '/dev/video2')
        self.bridge = CvBridge()
        self.cam = cv2.VideoCapture(self.camera_id)
        self.rate = rospy.Rate(30)

        print(self.cam.get(cv2.CAP_PROP_FORMAT))
        #print(cam.get(cv2.CAP_PROP_MODE))

        self.cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y','1','6',' '))
        self.cam.set(cv2.CAP_PROP_CONVERT_RGB, 0)

        self.cam.set(cv2.CAP_PROP_FORMAT,cv2.CV_16U)
        self.cam.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        self.cam.set(cv2.CAP_PROP_MODE, 3)  
        #print(cam.get(cv2.CAP_PROP_FORMAT))
        #print(cam.get(cv2.CAP_PROP_MODE))


c=cam()
time.sleep(1)
while True:
    try:
        _, frame = c.cam.read()
        frame_120160 = frame[:120,:]
    #    thermal_calib = 0.0217*(frame_120160-8192.0)+293.0-273.0
        gray_8bit = img_to_8bit_img(frame_120160)
        gray_8bit_resized=cv2.resize(gray_8bit, (480, 320), interpolation = cv2.INTER_AREA)
        colored_img = cv2.applyColorMap(gray_8bit_resized, cv2.COLORMAP_JET)
        img=c.bridge.cv2_to_imgmsg(colored_img, "bgr8")
        img_compressed=c.bridge.cv2_to_compressed_imgmsg(colored_img)
        img.header.stamp = rospy.Time.now()
        img_compressed.header.stamp = rospy.Time.now()
        c.img_publisher.publish(img)
        c.img_publisher2.publish(img_compressed)

    except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt):
        c.cam.release()
        sys.exit(0)
c.cam.release()
