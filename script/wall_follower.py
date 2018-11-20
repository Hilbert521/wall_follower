#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
import math
from std_msgs.msg import Int16,Float32,Bool,Float32MultiArray,Int16MultiArray
from nav_msgs.msg import Odometry
from fastsim.srv import *
import tf

# lasers=Float32MultiArray()
lasers = list()
odom = Odometry()

# Rate for ROS in Hz
rate = 10

# Finish line variables
fl1_x = 120
fl1_y = 0
fl1_h = 50
fl1_w = 10

fl2_x = 120
fl2_y = 300
fl2_h = 50
fl2_w = 10

# Robot lasers range

l_range = 100

class Rect:
	def __init__(self, x, y, h, w):
		self._x = x;
		self._y = y;
		self._h = h;
		self._w = w;

#-------------------------------------------
def callback_lasers(data):
	global lasers
	lasers=list(data.data)
	for i in range(0, len(lasers)):
		if(lasers[i] == -1):
			lasers[i] = l_range

#-------------------------------------------
def callback_odom(data):
	global odom
	odom=data

#-------------------------------------------
def toAbs(ang):
	if(ang < 0):
		return 2*math.pi + ang
	elif(ang > 2*math.pi):
		return ang - 2*math.pi
	else:
		return ang

#-------------------------------------------
def isAtFinishLine(fl_nb, rect):
	# 1) has the robot crossed the finish line ?
	rx = odom.pose.pose.position.x
	ry = odom.pose.pose.position.y

	if ((rx > rect._x and ry > rect._y) and (rx < rect._x + rect._w and ry < rect._y + rect._h)) and fl_nb == 1:
		rospy.wait_for_service('simu_fastsim/teleport')
		try:
			# teleport robot
			teleport = rospy.ServiceProxy('simu_fastsim/teleport', Teleport)
			x  = fl2_x - 1
			y  = 325
			th = math.pi
			resp1 = teleport(x, y, th)
			# store information about the duration of the finishing trial:
			currT = rospy.get_time()

			startT = currT
			rx = x
			ry = y

		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	elif ((rx > rect._x and ry > rect._y) and (rx < rect._x + rect._w and ry < rect._y + rect._h)) and fl_nb == 2:
		rospy.wait_for_service('simu_fastsim/teleport')
		try:
			# teleport robot
			teleport = rospy.ServiceProxy('simu_fastsim/teleport', Teleport)
			x  = fl1_x - 1
			y  = 25
			th = math.pi
			resp1 = teleport(x, y, th)
			# store information about the duration of the finishing trial:
			currT = rospy.get_time()

			startT = currT
			rx = x
			ry = y

		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

#-------------------------------------------
def wall_follower():
	rospy.init_node('wall_follower', anonymous=True)

	# The node publishes movement orders for simu_fastsim :
	pub_l = rospy.Publisher('/simu_fastsim/speed_left', Float32 , queue_size=10)
	pub_r = rospy.Publisher('/simu_fastsim/speed_right', Float32 , queue_size=10)

	# If necessary, the node receives sensory information from simu_fastsim:
	# rospy.Subscriber("/simu_fastsim/laser_scan", LaserScan, callback_lasers)
	rospy.Subscriber("/simu_fastsim/lasers", Float32MultiArray, callback_lasers)
	rospy.Subscriber("/simu_fastsim/odom", Odometry, callback_odom)

	# Targetted operating frequency of the node:
	r = rospy.Rate(rate) # 10hz
	
	# start time and timing related things
	startT = rospy.get_time()
	rospy.loginfo("Start time" + str(startT))

	# Step number
	step = 0

	fl1 = Rect(fl1_x, fl1_y, fl1_h, fl1_w)

	fl2 = Rect(fl2_x, fl2_y, fl2_h, fl2_w)

	# Initial desired angle
	ang_des = math.radians(180)

	# Proportionnal gain for P filter on orientation
	Kp = 0.5

	# Forward speed
	u = 1

	# Threshold for distance detection 
	seuil = 30.0

	# Flag that triggers when the robot is rotating
	rotating = False

	# Main loop:
	while (not rospy.is_shutdown()):
		speed_l=0
		speed_r=0

		# Sum of values from the lasers segmented in 6 regions (front left, front right, front, rear left, rear right, rear)
		sum_l = 0
		sum_r = 0
		sum_f = 0

		# Number of lasers
		sz = len(lasers)

		# Processing of the finish line
		#------------------------------------------------
		isAtFinishLine(1, fl1);
		isAtFinishLine(2, fl2);
		
		# Processing of the sensory data :
		#------------------------------------------------
		if sz != 0 and rotating == False:
			for i in range(0,2):
				sum_l += lasers[i + 1]
				sum_r += lasers[i + 5]
				if(i < 2):
					sum_f += lasers[i + 3]
			sum_f /= 2
			sum_l /= 3
			sum_r /= 3

			# rospy.loginfo(str(sum_l) + " | " + str(sum_f) + " | " + str(sum_r))
		
			if(sum_r < seuil):			# Can't turn right
				if(sum_f < seuil - 5):	# Can't go forward
					if(sum_l < seuil):	# Can't turn left
						rospy.loginfo("I'm turning of 180 deg")
						rotating = True
						ang_des -= math.radians(180)
					else:				# Can turn left
						rospy.loginfo("I'm turning on my left")
						rotating = True
						ang_des -= math.radians(90)
				else:					# Can go forward
						ang_des += 0
			elif(sum_f < seuil - 5):	# Can turn right
				rospy.loginfo("I'm turning on my right")
				rotating = True
				ang_des += math.radians(90)

		# Processing of the orientation
		#------------------------------------------------
		quaternion = (
		    odom.pose.pose.orientation.x,
		    odom.pose.pose.orientation.y,
		    odom.pose.pose.orientation.z,
		    odom.pose.pose.orientation.w)
		ang_mes = tf.transformations.euler_from_quaternion(quaternion)[2]
		ang_mes = toAbs(ang_mes)
		ang_des = toAbs(ang_des)

		err = ang_des - ang_mes

		# Choice of the strategy : rotating or moving forward
		#------------------------------------------------
		if(abs(err) > 0.01):
			cmd = Kp * err
			if err > math.radians(180):
				speed_l = cmd
				speed_r = -cmd
			else:
				speed_l = -cmd
				speed_r = cmd
		else:
			rotating = False
			cmd = u
			speed_l = cmd
			speed_r = cmd

		pub_l.publish(speed_l)
		pub_r.publish(speed_r)
		r.sleep()   

		step = step + 1

#-------------------------------------------
if __name__ == '__main__':
	try:
		wall_follower()
	except rospy.ROSInterruptException: pass