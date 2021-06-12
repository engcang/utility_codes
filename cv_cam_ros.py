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
        rospy.init_node('cam_controller', anonymous=True)
        self.img_publisher = rospy.Publisher('/image', Image, queue_size=1)
        self.img_publisher2 = rospy.Publisher('/image/compressed', CompressedImage, queue_size=1)
        self.camera_id = rospy.get_param("/cam_id", '/dev/video0')
        self.bridge = CvBridge()
        self.cam = cv2.VideoCapture(self.camera_id)
        self.rate = rospy.Rate(30)
	    	    
if __name__=='__main__':
    camera = cam()
    time.sleep(1)
    while True:
        try:
            _, frame = camera.cam.read()
            img=camera.bridge.cv2_to_imgmsg(frame, "bgr8")
            img_compressed=camera.bridge.cv2_to_compressed_imgmsg(frame)
            img.header.stamp = rospy.Time.now()
            img_compressed.header.stamp = rospy.Time.now()
            camera.img_publisher.publish(img)
            camera.img_publisher2.publish(img_compressed)
            camera.rate.sleep()	    
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            camera.cam.release()
            sys.exit(0)
        #except:
        #    print("exception")
        #    pass
    camera.cam.release()
