#!/usr/bin/env python
import rospy
import time
import mavros

from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import signal
import sys
import math
import numpy as np

global r2d
global d2r
r2d = 180/np.pi
d2r = np.pi/180

def signal_handler(signal,frame):
	print('pressed ctrl + c!!!')
	motor_left.run(Adafruit_MotorHAT.RELEASE)
	motor_right.run(Adafruit_MotorHAT.RELEASE)
	sys.exit(0)
signal.signal(signal.SIGINT,signal_handler)


# setup motor controller
motor_driver = Adafruit_MotorHAT(i2c_bus=1)

motor_left_ID = 1
motor_right_ID = 2

# setup ros node
motor_left = motor_driver.getMotor(motor_left_ID)
motor_right = motor_driver.getMotor(motor_right_ID)

def set_speed(motor_ID, value): # sets motor speed between [-1.0, 1.0]
	max_pwm = 115.0
	speed = int(min(max(abs(value * max_pwm), 0), max_pwm))
	if motor_ID == 1:
		motor = motor_left
	elif motor_ID == 2:
		motor = motor_right
	else:
		rospy.logerror('set_speed(%d, %f) -> invalid motor_ID=%d', motor_ID, value, motor_ID)
		return
	motor.setSpeed(speed)
	if value > 0:
		motor.run(Adafruit_MotorHAT.FORWARD)
	else:
		motor.run(Adafruit_MotorHAT.BACKWARD)

def all_stop(): # stops all motors
	motor_left.setSpeed(0)
	motor_right.setSpeed(0)
	motor_left.run(Adafruit_MotorHAT.RELEASE)
	motor_right.run(Adafruit_MotorHAT.RELEASE)
def pause(): # pause motors
	motor_left.setSpeed(0)
	motor_right.setSpeed(0)


class jetbot():
	def __init__(self):
		rospy.init_node('control_jetbot_imu',anonymous=True)
		rospy.Subscriber('/mavros/local_position/odom', Odometry, self.local_val_callback)
		rospy.Subscriber('/jet_cmd', String, self.desired_val_callback)
		rospy.Subscriber('/missin_fin', String, self.isfin_callback)

		self.slowly = 0
		self.chk = 0 
		self.d_speed = 0
		self.d_degree = 0
		self.desired_in=0

	def local_val_callback(self,msg):
		# self.slowly+=1
		# if self.slowly%2==0:
		orientation_list = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
		(c_roll, c_pitch, self.c_yaw) = euler_from_quaternion(orientation_list)
		#print(rospy.Time.now(), self.c_yaw*r2d)

	def desired_val_callback(self,msg):
		self.desired_in=1
		Data = msg.data
		d_degree,d_speed = Data.split(',')
		self.d_degree = float(d_degree)*r2d # desired degree
		self.d_speed = float(d_speed)*0.239 # desired lin vel, -0.239 ~ +0.239
		print("Desired_deg : %f, Desired_lin_vel : %f" %(self.d_degree, self.d_speed))


	def isfin_callback(self,msg):
	        self.chk = int(msg.data)
		if self.chk == 1:
		    all_stop()
		    print("stop!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

	def set_lin_velocity(self,lin_vel): #set lin vel as desired using set_speed
		print('i am here and vel : %f'%lin_vel)
		set_speed(motor_left_ID,lin_vel)
		set_speed(motor_right_ID,lin_vel)

	def set_ang_velocity(self): #set ang vel as desired using set_speed
		error=(self.d_degree-self.c_yaw)
		if error > 180:
			error-=360
		if error < -180:
			error+=360
		error = error*0.3 # P gain
		if error > 0 : #unclockwise
			set_speed(motor_left_ID,-error)
			set_speed(motor_right_ID,error)
		elif error < 0 : #clockwise
			set_speed(motor_left_ID,error)
			set_speed(motor_right_ID,-error)


rbt = jetbot()
if __name__ == '__main__':
	# running rate
	r=rospy.Rate(10)

	while 1 :
		if rbt.desired_in==1:
			print(rbt.d_degree-rbt.c_yaw*r2d)
			while abs(rbt.d_degree-rbt.c_yaw*r2d) > 12:
				rbt.set_ang_velocity()
				r.sleep()
			pause()
			rbt.set_lin_velocity(rbt.d_speed)
			r.sleep()
