#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 26 02:02:03 2019
@author: mason
"""

''' import libraries '''
import time
import rospy
from geometry_msgs.msg import Point, PoseStamped
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
        self.global_marker_pub = rospy.Publisher('/exploration/bound', Marker, queue_size=10)
        self.local_marker_pub = rospy.Publisher('/local/bound', Marker, queue_size=10)
        self.pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        self.rate = rospy.Rate(1)
        self.global_marker = Marker()
        self.global_marker.header.frame_id = "map"
        self.global_marker.type = 5;
        self.global_marker.scale.x = 0.1
        self.global_marker.color.r = 1
        self.global_marker.color.b = 1
        self.global_marker.color.a = 0.6
        self.global_marker.pose.orientation.w = 1

        minx = -2
        miny = -2
        minz = -1
        maxx = 10
        maxy = 28
        maxz = 4
        
        self.global_marker.points.append(pt_to_geo(minx, miny, minz))
        self.global_marker.points.append(pt_to_geo(minx, miny, maxz))
        self.global_marker.points.append(pt_to_geo(minx, miny, minz))
        self.global_marker.points.append(pt_to_geo(minx, maxy, minz))
        self.global_marker.points.append(pt_to_geo(minx, miny, minz))
        self.global_marker.points.append(pt_to_geo(maxx, miny, minz))
        self.global_marker.points.append(pt_to_geo(maxx, miny, minz))
        self.global_marker.points.append(pt_to_geo(maxx, maxy, minz))
        self.global_marker.points.append(pt_to_geo(maxx, miny, minz))
        self.global_marker.points.append(pt_to_geo(maxx, miny, maxz))
        self.global_marker.points.append(pt_to_geo(minx, maxy, minz))
        self.global_marker.points.append(pt_to_geo(minx, maxy, maxz))
        self.global_marker.points.append(pt_to_geo(minx, maxy, minz))
        self.global_marker.points.append(pt_to_geo(maxx, maxy, minz))
        self.global_marker.points.append(pt_to_geo(maxx, maxy, minz))
        self.global_marker.points.append(pt_to_geo(maxx, maxy, maxz))
        
        self.global_marker.points.append(pt_to_geo(minx, miny, maxz))
        self.global_marker.points.append(pt_to_geo(minx, maxy, maxz))
        self.global_marker.points.append(pt_to_geo(minx, miny, maxz))
        self.global_marker.points.append(pt_to_geo(maxx, miny, maxz))
        self.global_marker.points.append(pt_to_geo(maxx, miny, maxz))
        self.global_marker.points.append(pt_to_geo(maxx, maxy, maxz))
        self.global_marker.points.append(pt_to_geo(maxx, maxy, maxz))
        self.global_marker.points.append(pt_to_geo(minx, maxy, maxz))

        self.local_marker = Marker()
        self.local_marker.header.frame_id = "map"
        self.local_marker.type = 5;
        self.local_marker.scale.x = 0.06
        self.local_marker.color.b = 1
        self.local_marker.color.g = 1
        self.local_marker.color.a = 0.7
        self.local_marker.pose.orientation.w = 1

        loc_minx = -4
        loc_miny = -4
        loc_minz = -1.5
        loc_maxx = 4
        loc_maxy = 4
        loc_maxz = 1.5

        self.local_marker.points.append(pt_to_geo(loc_minx, loc_miny, loc_minz))
        self.local_marker.points.append(pt_to_geo(loc_minx, loc_miny, loc_maxz))
        self.local_marker.points.append(pt_to_geo(loc_minx, loc_miny, loc_minz))
        self.local_marker.points.append(pt_to_geo(loc_minx, loc_maxy, loc_minz))
        self.local_marker.points.append(pt_to_geo(loc_minx, loc_miny, loc_minz))
        self.local_marker.points.append(pt_to_geo(loc_maxx, loc_miny, loc_minz))
        self.local_marker.points.append(pt_to_geo(loc_maxx, loc_miny, loc_minz))
        self.local_marker.points.append(pt_to_geo(loc_maxx, loc_maxy, loc_minz))
        self.local_marker.points.append(pt_to_geo(loc_maxx, loc_miny, loc_minz))
        self.local_marker.points.append(pt_to_geo(loc_maxx, loc_miny, loc_maxz))
        self.local_marker.points.append(pt_to_geo(loc_minx, loc_maxy, loc_minz))
        self.local_marker.points.append(pt_to_geo(loc_minx, loc_maxy, loc_maxz))
        self.local_marker.points.append(pt_to_geo(loc_minx, loc_maxy, loc_minz))
        self.local_marker.points.append(pt_to_geo(loc_maxx, loc_maxy, loc_minz))
        self.local_marker.points.append(pt_to_geo(loc_maxx, loc_maxy, loc_minz))
        self.local_marker.points.append(pt_to_geo(loc_maxx, loc_maxy, loc_maxz))
        
        self.local_marker.points.append(pt_to_geo(loc_minx, loc_miny, loc_maxz))
        self.local_marker.points.append(pt_to_geo(loc_minx, loc_maxy, loc_maxz))
        self.local_marker.points.append(pt_to_geo(loc_minx, loc_miny, loc_maxz))
        self.local_marker.points.append(pt_to_geo(loc_maxx, loc_miny, loc_maxz))
        self.local_marker.points.append(pt_to_geo(loc_maxx, loc_miny, loc_maxz))
        self.local_marker.points.append(pt_to_geo(loc_maxx, loc_maxy, loc_maxz))
        self.local_marker.points.append(pt_to_geo(loc_maxx, loc_maxy, loc_maxz))
        self.local_marker.points.append(pt_to_geo(loc_minx, loc_maxy, loc_maxz))

    def pose_callback(self, msg):
        self.local_marker.pose.position = msg.pose.position
        self.local_marker_pub.publish(self.local_marker)

##############################################################################################

mav_ctr = robot()
time.sleep(0.5) #wait 1 second to assure that all data comes in

''' main '''
if __name__ == '__main__':
    print("STARTING VIS")
    while 1:
        try:
            mav_ctr.global_marker_pub.publish(mav_ctr.global_marker);
            mav_ctr.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
        #except:
        #    print("exception")
        #    pass