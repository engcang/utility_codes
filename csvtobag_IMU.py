#!/usr/bin/env python

import sys
import rosbag
import rospy
from sensor_msgs.msg import Imu
import pandas as pd

def CreateImuBag(csvname, outname):
  dt = pd.read_csv(csvname)
  print(dt)
  bag = rosbag.Bag(outname, 'w')

  for row in range(dt.shape[0]):
    timestamp = rospy.Time.from_sec(dt['time'][row])
    imu_msg = Imu()
    imu_msg.header.stamp = timestamp
    imu_msg.angular_velocity.x = dt['gx'][row]
    imu_msg.angular_velocity.y = dt['gy'][row]
    imu_msg.angular_velocity.z = dt['gz'][row]
    imu_msg.linear_acceleration.x = dt['ax'][row]
    imu_msg.linear_acceleration.y = dt['ay'][row]
    imu_msg.linear_acceleration.z = dt['az'][row]
    imu_msg.orientation.w = dt['qw'][row]
    imu_msg.orientation.x = dt['qx'][row]
    imu_msg.orientation.y = dt['qy'][row]
    imu_msg.orientation.z = dt['qz'][row]
    bag.write("/imu", imu_msg, timestamp)

  bag.close()
  return


if __name__ == "__main__":
  if len( sys.argv ) == 3:
    CreateImuBag(*sys.argv[1:])
  else:
    print( "Usage: python csvtobag_IMU.py csvname outbagfilename")