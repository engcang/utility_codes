import time, sys, os
from ros import rosbag
import roslib, rospy
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2

import sys
import signal
def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

def Bagtoimage(bagname, folder='./'):
    bridge=CvBridge()
    topics = []
    for topic, msg, t in rosbag.Bag(bagname):
        if topic not in topics:
            topics.append(topic)

    for topi in topics:
        img_name_prefix="_".join(topi.split('/'))
        count=0
        for topic, msg, t in rosbag.Bag(bagname):
            if topic == topi:
#                img=bridge.imgmsg_to_cv2(msg, "bgr8")
                img=bridge.imgmsg_to_cv2(msg)
                #img=bridge.compressed_imgmsg_to_cv2(msg)
#                cv2.imshow('test', img)
#                cv2.waitKey(1)
                print(folder+'/'+img_name_prefix+'_%06d.png'%count)
                cv2.imwrite(folder+'/'+img_name_prefix+'_%06d.png'%count, img)
                count=count+1

if __name__ == "__main__":
    if len( sys.argv ) == 3:
        Bagtoimage(*sys.argv[1:])
    else:
        print( "Usage: python bag2img.py bagfilename output_foldername")
