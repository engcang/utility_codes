import rospy
import numpy as np
from livox_ros_driver.msg import CustomMsg
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.point_cloud2 import create_cloud
import math

class LivoxToOuster:
    def __init__(self):
        # ROS Node Initialization
        rospy.init_node("livox_to_ouster", anonymous=True)

        # Parameters
        self.intensity_conv_coef = rospy.get_param("~intensityConvCoef", 1.0)

        # ROS Subscribers and Publishers
        self.livox_cloud_sub = rospy.Subscriber("/livox/lidar", CustomMsg, self.cloud_handler, queue_size=50)
        self.ouster_cloud_pub = rospy.Publisher("/livox/lidar_ouster", PointCloud2, queue_size=50)

    def cloud_handler(self, msg):
        # Create an empty PointCloud2 message
        header = msg.header
        header.stamp = rospy.Time.now()

        # Convert Livox CustomMsg to Ouster-like PointCloud2
        points = []
        for point in msg.points:
            x, y, z = point.x, point.y, point.z
            intensity = point.reflectivity * self.intensity_conv_coef
            ring = point.line
            offset_time = point.offset_time
            range_val = math.sqrt(x ** 2 + y ** 2 + z ** 2) * 1000.0  # Convert to millimeters

            points.append([x, y, z, intensity, ring, offset_time, range_val])

        # Define PointFields for PointCloud2
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("intensity", 12, PointField.FLOAT32, 1),
            PointField("ring", 16, PointField.UINT16, 1),
            PointField("t", 18, PointField.UINT32, 1),
            PointField("range", 22, PointField.FLOAT32, 1),
        ]

        # Create PointCloud2
        cloud_msg = create_cloud(header, fields, points)

        # Publish the converted PointCloud2
        self.ouster_cloud_pub.publish(cloud_msg)

if __name__ == "__main__":
    try:
        converter = LivoxToOuster()
        rospy.loginfo("Livox to Ouster conversion node started")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Livox to Ouster conversion node terminated")

