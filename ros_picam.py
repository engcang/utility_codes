import cv2
import rospy
import time

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
    print('You pressed Ctrl+C!')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


'''using Gstreamer'''
#for NVargus
#camSet = 'nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=3264, height=2464, framerate=21/1, format=NV12 ! nvvidconv flip-method=2 ! video/x-raw, width=800, height=600, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink' 
#flip-method=0 or 2

#wbmode : white balance, 0:off, 1:auto(default)
#tnr-mode : noise reduction mode, 0:off, 1:Fast(default), 2:High Quality
#ee-mode : edge enhancement mode, 0:off, 1:Fast(def), 2:High Qual
#contrast 0~2, brightness-1~1, saturation0~2
camSet = 'nvarguscamerasrc sensor-id=0 tnr-mode=1 wbmode=1 ! video/x-raw(memory:NVMM), width=1280, height=720, framerate=21/1, format=NV12 ! nvvidconv flip-method=2 ! video/x-raw, width=800, height=600, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! videobalance contrast=1.2 brightness=-.15 saturation=1.0 ! appsink'
#flip-method=0 or 2


class cam():
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.img_publisher = rospy.Publisher('/pi_cam',Image,queue_size=1)
        self.bridge = CvBridge()
        self.cam = cv2.VideoCapture(camSet)
        self.rate = rospy.Rate(21)
	    	    
if __name__=='__main__':
    camera = cam()
    time.sleep(2)
    while True:
        try:
            _, frame = camera.cam.read()
            img=camera.bridge.cv2_to_imgmsg(frame, "bgr8")
            img.header.stamp = rospy.Time.now()
            camera.img_publisher.publish(img)
            camera.rate.sleep()	    
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            camera.cam.release()
            sys.exit(0)
        #except:
        #    print("exception")
        #    pass
    camera.cam.release()
