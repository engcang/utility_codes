#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 26 02:02:03 2019
@author: mason
"""

''' import libraries '''
import time
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
    print('You pressed Ctrl+C!')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


def pt_to_geo(x, y, z):
    geo = Point()
    geo.x = x
    geo.y = y
    geo.z = z
    return geo


''' class '''
class robot():
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.position_pub = rospy.Publisher('/exploration/bound', Marker, queue_size=10)
        self.rate = rospy.Rate(1)
        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.type = 5;
        self.marker.scale.x = 0.06
        self.marker.color.r = 1
        self.marker.color.b = 1
        self.marker.color.a = 0.6
        self.marker.pose.orientation.w = 1
        
        minx = -2
        miny = -2
        minz = -1
        maxx = 10
        maxy = 28
        maxz = 4
        
        self.marker.points.append(pt_to_geo(minx, miny, minz))
        self.marker.points.append(pt_to_geo(minx, miny, maxz))
        self.marker.points.append(pt_to_geo(minx, miny, minz))
        self.marker.points.append(pt_to_geo(minx, maxy, minz))
        self.marker.points.append(pt_to_geo(minx, miny, minz))
        self.marker.points.append(pt_to_geo(maxx, miny, minz))
        self.marker.points.append(pt_to_geo(maxx, miny, minz))
        self.marker.points.append(pt_to_geo(maxx, maxy, minz))
        self.marker.points.append(pt_to_geo(maxx, miny, minz))
        self.marker.points.append(pt_to_geo(maxx, miny, maxz))
        self.marker.points.append(pt_to_geo(minx, maxy, minz))
        self.marker.points.append(pt_to_geo(minx, maxy, maxz))
        self.marker.points.append(pt_to_geo(minx, maxy, minz))
        self.marker.points.append(pt_to_geo(maxx, maxy, minz))
        self.marker.points.append(pt_to_geo(maxx, maxy, minz))
        self.marker.points.append(pt_to_geo(maxx, maxy, maxz))
        
        self.marker.points.append(pt_to_geo(minx, miny, maxz))
        self.marker.points.append(pt_to_geo(minx, maxy, maxz))
        self.marker.points.append(pt_to_geo(minx, miny, maxz))
        self.marker.points.append(pt_to_geo(maxx, miny, maxz))
        self.marker.points.append(pt_to_geo(maxx, miny, maxz))
        self.marker.points.append(pt_to_geo(maxx, maxy, maxz))
        self.marker.points.append(pt_to_geo(maxx, maxy, maxz))
        self.marker.points.append(pt_to_geo(minx, maxy, maxz))

##############################################################################################

mav_ctr = robot()
time.sleep(0.5) #wait 1 second to assure that all data comes in

''' main '''
if __name__ == '__main__':
    print("STARTING VIS")
    while 1:
        try:
            mav_ctr.position_pub.publish(mav_ctr.marker);
            mav_ctr.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
        #except:
        #    print("exception")
        #    pass