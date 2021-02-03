#!/usr/bin/env python
# from serial import Serial
import serial
import time

import rospy
from geometry_msgs.msg import Pose

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
  print('You pressed Ctrl+C!')
  sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


if __name__ == '__main__':
  rospy.init_node("serial_connector_2", anonymous=True)
  received_odom_pub = rospy.Publisher("/received_pose", Pose, queue_size=1)
  ser = serial.Serial('/dev/ttyUSB0', 19200, timeout=1)  # open serial port
  print(ser.name)         # check which port was really used

  while 1:
    try:
      if ser.is_open:
        d=ser.read(80)
        b=d.split(',')
        if b[0]!='s':
          ser.close()
        else:
          print(b)
          pmsg = Pose()
          pmsg.position.x = float(b[1])
          pmsg.position.y = float(b[2])
          pmsg.position.z = float(b[3])
          pmsg.orientation.x = float(b[4])
          pmsg.orientation.y = float(b[5])
          pmsg.orientation.z = float(b[6])
          pmsg.orientation.w = float(b[7])
          received_odom_pub.publish(pmsg)
          ser.write('h')
      else:
        ser = serial.Serial('/dev/ttyUSB0', 19200, timeout=1)  # open serial port
    except (SystemExit, KeyboardInterrupt) :
      ser.close()             # close port
      sys.exit(0)
