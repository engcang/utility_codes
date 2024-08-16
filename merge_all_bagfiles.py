import os
import rosbag

def merge_bag_files(input_directory, output_bag_file):
    """
    Merges all .bag files in the specified directory into a single new bag file.
    
    :param input_directory: The path to the directory containing the .bag files to merge.
    :param output_bag_file: The path and name of the output merged bag file.
    """
    # List all bag files in the input directory
    bag_files = [os.path.join(input_directory, f) for f in os.listdir(input_directory) if f.endswith('.bag')]
    bag_files.sort()  # Optional: sort files by name

    with rosbag.Bag(output_bag_file, 'w') as outbag:
        for bag_file in bag_files:
            print(f"Merging {bag_file}...")
            with rosbag.Bag(bag_file, 'r') as inbag:
                for topic, msg, t in inbag.read_messages():
                    outbag.write(topic, msg, t)
    print(f"All bag files have been merged into {output_bag_file}")

# Example usage
input_directory = '/home/mason/bags/subt_mrs/sensor fusion challenges/SubT_MRS_Hawkins_Multi_Floor_LegRobot'
output_bag_file = '/home/mason/bags/subt_mrs/sensor fusion challenges/SubT_MRS_Hawkins_Multi_Floor_LegRobot.bag'
merge_bag_files(input_directory, output_bag_file)
