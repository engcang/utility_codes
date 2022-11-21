#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thur Nov 17 04:25:30 2022
@author: mason
"""

''' import libraries '''
import time
import math
import os

import piexif
from fractions import Fraction
from datetime import datetime

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

import sys
import signal
def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


def to_deg(value, loc):
    """convert decimal coordinates into degrees, munutes and seconds tuple
    Keyword arguments: value is float gps-value, loc is direction list ["S", "N"] or ["W", "E"]
    return: tuple like (25, 13, 48.343 ,'N')
    """
    if value < 0.0:
        loc_value = loc[0]
    elif value > 0.0:
        loc_value = loc[1]
    else:
        loc_value = ""
    abs_value = abs(value)
    deg =  int(abs_value)
    t1 = (abs_value-deg)*60.0
    min = int(t1)
    sec = round((t1 - min)* 60.0, 5)
    return (deg, min, sec, loc_value)
def change_to_rational(number):
    """convert a number to rantional
    Keyword arguments: number
    return: tuple like (1, 2), (numerator, denominator)
    """
    f = Fraction(str(number))
    return (f.numerator, f.denominator)
def set_gps_location(file_name, lat, lng, altitude, gpsTime):
    """Adds GPS position as EXIF metadata
    Keyword arguments:
    file_name -- image file
    lat -- latitude (as float)
    lng -- longitude (as float)
    altitude -- altitude (as float)
    """
    lat_deg = to_deg(lat, ["S", "N"])
    lng_deg = to_deg(lng, ["W", "E"])

    exiv_lat = (change_to_rational(lat_deg[0]), change_to_rational(lat_deg[1]), change_to_rational(lat_deg[2]))
    exiv_lng = (change_to_rational(lng_deg[0]), change_to_rational(lng_deg[1]), change_to_rational(lng_deg[2]))

    gps_ifd = {
        piexif.GPSIFD.GPSVersionID: (2, 0, 0, 0),
        piexif.GPSIFD.GPSAltitudeRef: 0,
        piexif.GPSIFD.GPSAltitude: change_to_rational(round(altitude)),
        piexif.GPSIFD.GPSLatitudeRef: lat_deg[3],
        piexif.GPSIFD.GPSLatitude: exiv_lat,
        piexif.GPSIFD.GPSLongitudeRef: lng_deg[3],
        piexif.GPSIFD.GPSLongitude: exiv_lng,
        piexif.GPSIFD.GPSDateStamp: gpsTime
    }

    exif_dict = {"GPS": gps_ifd}
    exif_data = piexif.load(file_name)
    exif_data.update(exif_dict)
    # print(exif_data)
    exif_bytes = piexif.dump(exif_data)
    piexif.insert(exif_bytes, file_name)
    return


''' class '''
class gps_log_class():
    def __init__(self):
        rospy.init_node('gps_logger', anonymous=True)
        self.gps_sub = rospy.Subscriber('/dji_osdk_ros/gps_position', NavSatFix, self.gps_cb)
        self.img_name_sub = rospy.Subscriber('/telicam/img_name', String, self.img_name_cb)
        self.gps_array = []
        self.z_offset = 0.0

    def img_name_cb(self, msg):
        file_path = msg.data
        file_time = msg.data.split('/')[-1].split('.')[0]
        sec = float(file_time.split('_')[0])
        nsec = float(file_time.split('_')[1]) * 1e-9
        file_time_float = sec + nsec
        # get nearest
        time_diff = 10.0
        nearest_gps = self.gps_array[0]
        for gps in self.gps_array:
            if abs(file_time_float - gps.header.stamp.to_sec()) < abs(nearest_gps.header.stamp.to_sec() - file_time_float):
                nearest_gps = gps
        # wait till imwrite
        time.sleep(0.3)
        # write gps data
        print (self.z_offset)
        if nearest_gps.altitude <= 0.0 and self.z_offset==0.0:
            self.z_offset = -nearest_gps.altitude + 50.0
        set_gps_location(file_path, nearest_gps.latitude, nearest_gps.longitude, nearest_gps.altitude+self.z_offset, datetime.utcfromtimestamp(file_time_float).strftime('%Y:%m:%d %H:%M:%S'))
        return

    def gps_cb(self, msg):
        self.gps_array.append(msg)
        while rospy.Time.now().to_sec() - self.gps_array[0].header.stamp.to_sec() > 10.0:
            del self.gps_array[0]
        return

''' main '''
gps_logger_ = gps_log_class()

if __name__ == '__main__':
    while 1:
        try:
            time.sleep(1)            
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
        # except:
        #     print("exception")
        #     pass