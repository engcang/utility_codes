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


class cam():
    def __init__(self):
        rospy.init_node('img_saver', anonymous=True)
        self.img_count = rospy.get_param("/count", 0)
        self.img_subscriber = rospy.Subscriber('/d435i/depth/rgb_image_raw', Image, self.callback_saver)
        self.image_path = '/home/mason/images/'
        self.bridge = CvBridge()
        self.rate = rospy.Rate(3)
        self.flag = False

    def callback_saver(self, msg):
        self.img=self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.flag = True

	    	    
if __name__=='__main__':
    camera = cam()
    time.sleep(1)
    while True:
        try:
            if (camera.flag):
                cv2.imwrite(camera.image_path+'%04d.jpg'%camera.img_count, camera.img)
                camera.img_count = camera.img_count+1
                print(camera.img_count)
            camera.rate.sleep()	    
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
