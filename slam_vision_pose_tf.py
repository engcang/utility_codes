#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 03 05:39:30 2023

@author: mason
"""

''' import libraries '''
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
    print('You pressed Ctrl+C!')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

def quaternion_rotation_matrix(Q):
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                            [r10, r11, r12],
                            [r20, r21, r22]])
    return rot_matrix

def rotationMatrixToQuaternion1(m):
    #q0 = qw
    t = np.matrix.trace(m)
    q = np.asarray([0.0, 0.0, 0.0, 0.0], dtype=np.float64)

    if(t > 0):
        t = np.sqrt(t + 1)
        q[3] = 0.5 * t
        t = 0.5/t
        q[0] = (m[2,1] - m[1,2]) * t
        q[1] = (m[0,2] - m[2,0]) * t
        q[2] = (m[1,0] - m[0,1]) * t

    else:
        i = 0
        if (m[1,1] > m[0,0]):
            i = 1
        if (m[2,2] > m[i,i]):
            i = 2
        j = (i+1)%3
        k = (j+1)%3

        t = np.sqrt(m[i,i] - m[j,j] - m[k,k] + 1)
        q[i] = 0.5 * t
        t = 0.5 / t
        q[3] = (m[k,j] - m[j,k]) * t
        q[j] = (m[j,i] + m[i,j]) * t
        q[k] = (m[k,i] + m[i,k]) * t

    return q

''' class '''

class gtpub():
    def __init__(self):
        rospy.init_node('gtgt', anonymous=True)
        self.my_pose = rospy.Subscriber('/Odometry', Odometry, self.odom_cb)
        self.pose_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)

        self.rate = rospy.Rate(1)
        self.transform_LIO_wrt_IMU = np.array([[-0.7986335, 0.0, 0.6018150, 0.080331],
                                               [0.0, 1.0, 0.0, 0.0],
                                               [-0.6018150,  0.0, -0.7986335, -0.089397],
                                               [0,          0,          0,          1]])
        
    def odom_cb(self, msg):
        data = PoseStamped()
        data.header = msg.header
        
        pose_before_tf_ = np.eye(4)
        pose_before_tf_[0, 3] = msg.pose.pose.position.x
        pose_before_tf_[1, 3] = msg.pose.pose.position.y
        pose_before_tf_[2, 3] = msg.pose.pose.position.z
        # rot_mat_ = quaternion_rotation_matrix([msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z])
        rot_mat_ = quaternion_rotation_matrix([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.x, msg.pose.pose.orientation.w])
        pose_before_tf_[0, 0] = rot_mat_[0, 0]
        pose_before_tf_[0, 1] = rot_mat_[0, 1]
        pose_before_tf_[0, 2] = rot_mat_[0, 2]
        pose_before_tf_[1, 0] = rot_mat_[1, 0]
        pose_before_tf_[1, 1] = rot_mat_[1, 1]
        pose_before_tf_[1, 2] = rot_mat_[1, 2]
        pose_before_tf_[2, 0] = rot_mat_[2, 0]
        pose_before_tf_[2, 1] = rot_mat_[2, 1]
        pose_before_tf_[2, 2] = rot_mat_[2, 2]
        
        tfed_pose_ = self.transform_LIO_wrt_IMU@pose_before_tf_
        tfed_rotation_ = rotationMatrixToQuaternion1(np.array([[tfed_pose_[0, 0], tfed_pose_[0, 1], tfed_pose_[0, 2]],
                                       [tfed_pose_[1, 0], tfed_pose_[1, 1], tfed_pose_[1, 2]],
                                       [tfed_pose_[2, 0], tfed_pose_[2, 1], tfed_pose_[0, 2]]]))
        
        data.pose.position.x = tfed_pose_[0, 3]
        data.pose.position.y = tfed_pose_[1, 3]
        data.pose.position.z = tfed_pose_[2, 3]
        data.pose.orientation.w = tfed_rotation_[0]
        data.pose.orientation.x = tfed_rotation_[1]
        data.pose.orientation.y = tfed_rotation_[2]
        data.pose.orientation.z = tfed_rotation_[3]

        self.pose_pub.publish(data)

''' main '''
pub_class = gtpub()

if __name__ == '__main__':
    while 1:
        try:
            pub_class.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
