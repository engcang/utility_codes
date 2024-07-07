import rosbag
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
from PIL import Image
import io

def create_gif_from_rosbag(bag_file, topic, output_gif):
    # Initialize the ROS bag
    bag = rosbag.Bag(bag_file, 'r')
    bridge = CvBridge()
    
    images = []
    
    # Read messages from the specified topic
    for topic, msg, t in bag.read_messages(topics=[topic]):
        # Convert the compressed image to an OpenCV image
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # Convert OpenCV image to PIL image
        pil_image = Image.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        images.append(pil_image)
    
    bag.close()
    
    # Save the list of PIL images as a GIF
    if images:
        images[0].save(output_gif, save_all=True, append_images=images[1:], loop=0, duration=100)
    else:
        print("No images found in the specified topic")

# Example usage
bag_file = './sixth_contact.bag'
topic = '/detected_image/compressed'
output_gif = './horizon.gif'

create_gif_from_rosbag(bag_file, topic, output_gif)

