import rosbag
import csv
from geometry_msgs.msg import PoseStamped

def bag_to_tum(input_bag, output_csv, topic_name):
    """
    Converts a rosbag topic to TUM format CSV.

    Args:
        input_bag (str): Path to the input rosbag file.
        output_csv (str): Path to the output CSV file.
        topic_name (str): Name of the topic to extract.
    """
    with rosbag.Bag(input_bag, 'r') as bag, open(output_csv, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=' ')

        for topic, msg, t in bag.read_messages():
            if topic == topic_name:
                # Extract timestamp in seconds
                timestamp = msg.header.stamp.to_sec()
                
                # Extract position (tx, ty, tz)
                tx = msg.pose.position.x
                ty = msg.pose.position.y
                tz = msg.pose.position.z

                # Extract orientation (qx, qy, qz, qw)
                qx = msg.pose.orientation.x
                qy = msg.pose.orientation.y
                qz = msg.pose.orientation.z
                qw = msg.pose.orientation.w

                # Write to CSV in TUM format
                writer.writerow([timestamp, tx, ty, tz, qx, qy, qz, qw])

# Example usage
input_bag_path = './sbs_02/sbs_02.bag'  # Replace with your bag file path
output_csv_path = './sbs_02/tum_gt.csv'  # Replace with desired output CSV file name
topic_name = '/leica/pose/relative'  # Topic to extract

bag_to_tum(input_bag_path, output_csv_path, topic_name)

