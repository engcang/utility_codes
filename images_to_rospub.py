#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import os
import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
    print('You pressed Ctrl+C!')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

def publish_images(folder_path, topic_name, fps):
    rospy.init_node('image_publisher', anonymous=True)
    pub = rospy.Publisher(topic_name, CompressedImage, queue_size=10)
    rate = rospy.Rate(float(fps))
    bridge = CvBridge()

    image_files = [f for f in os.listdir(folder_path) if f.endswith(('png', 'jpg', 'jpeg'))]
    image_files.sort()

    while not rospy.is_shutdown():
        for image_file in image_files:
            image_path = os.path.join(folder_path, image_file)
            image = cv2.imread(image_path)
            image = cv2.resize(image, (640, 480))

            if image is None:
                rospy.logwarn("Image read failed: %s", image_path)
                continue

            try:
                ros_image = bridge.cv2_to_compressed_imgmsg(image)
                pub.publish(ros_image)
                rate.sleep()
            except rospy.ROSInterruptException:
                break
            except Exception as e:
                rospy.logerr("Error converting image: %s", e)


if __name__ == '__main__':
    if len( sys.argv ) == 4:
        publish_images(*sys.argv[1:])
    else:
        print( "Usage: python folder_path topic_name fps")
