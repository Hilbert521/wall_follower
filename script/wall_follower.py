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
fl1_y = 250
fl1_h = 50
fl1_w = 10

fl2_x = 120
fl2_y = 300
fl2_h = 50
fl2_w = 10

# Robot lasers range
l_range = 100

# Robot maximal command on each motor
max_cmd = 0.8

# Lap counter
lap = 0

# Time since last lap
last_lap = 0

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
def to_abs(ang):
	if(ang < 0):
		return 2*math.pi + ang
	elif(ang >= 2*math.pi):
		return ang - 2*math.pi
	else:
		return ang

#-------------------------------------------
def is_at_finish_line(fl_nb, rect):
	global lap
	# Has the robot crossed the finish line ?
	rx = odom.pose.pose.position.x
	ry = odom.pose.pose.position.y

	if ((rx > rect._x and ry > rect._y) and (rx < rect._x + rect._w and ry < rect._y + rect._h)) and fl_nb == 1 and lap == 3:
		rospy.wait_for_service('simu_fastsim/teleport')
		try:
			# teleport robot
			teleport = rospy.ServiceProxy('simu_fastsim/teleport', Teleport)
			x  = fl2_x + fl2_w + 1
			y  = 325
			th = 0
			resp1 = teleport(x, y, th)

			rx = x
			ry = y

			lap = 0

			return True

		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	elif ((rx > rect._x and ry > rect._y) and (rx < rect._x + rect._w and ry < rect._y + rect._h)) and fl_nb == 2 and lap == 3:
		rospy.wait_for_service('simu_fastsim/teleport')
		try:
			# teleport robot
			teleport = rospy.ServiceProxy('simu_fastsim/teleport', Teleport)
			x  = fl1_x + fl1_w + 1
			y  = 280
			th = 0
			resp1 = teleport(x, y, th)

			rx = x
			ry = y

			lap = 0

			return True

		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	return False

#-------------------------------------------
def lap_inc(rect):
	global lap, last_lap
	# Has the robot crossed a finish line ?
	rx = odom.pose.pose.position.x
	ry = odom.pose.pose.position.y

	if ((rx > rect._x and ry > rect._y) and (rx < rect._x + rect._w and ry < rect._y + rect._h)) and (rospy.get_time() - last_lap) > 15:
		lap += 1
		last_lap = rospy.get_time()

#-------------------------------------------
def wall_follower():
	rospy.init_node('wall_follower', anonymous=True)

	# The node publishes movement orders for simu_fastsim :
	pub_l = rospy.Publisher('/simu_fastsim/speed_left', Float32 , queue_size=10)
	pub_r = rospy.Publisher('/simu_fastsim/speed_right', Float32 , queue_size=10)

	# The node receives sensory information from simu_fastsim:
	# rospy.Subscriber("/simu_fastsim/laser_scan", LaserScan, callback_lasers)
	rospy.Subscriber("/simu_fastsim/lasers", Float32MultiArray, callback_lasers)
	rospy.Subscriber("/simu_fastsim/odom", Odometry, callback_odom)

	# Targetted operating frequency of the node:
	r = rospy.Rate(rate) # 10hz
	
	# start time and timing related things
	startT = rospy.get_time()
	rospy.loginfo("Start time: " + str(startT))

	# Step number
	step = 0

	fl1 = Rect(fl1_x, fl1_y, fl1_h, fl1_w)

	fl2 = Rect(fl2_x, fl2_y, fl2_h, fl2_w)

	# Initial desired angle
	ang_des = math.radians(0)

	# Proportionnal gain for P filter on orientation
	Kp = 0.8

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

		# Processing of the lap counter
		#------------------------------------------------
		lap_inc(fl1)
		lap_inc(fl2)
		# rospy.loginfo("Lap number: " + str(lap+1))

		# Processing of the finish line
		#------------------------------------------------
		if(is_at_finish_line(1, fl1)):
			ang_des = math.radians(0)
		if(is_at_finish_line(2, fl2)):
			ang_des = math.radians(0)
		
		# Processing of the sensory data :
		#------------------------------------------------
		if sz != 0 and rotating == False:
			for i in range(1,3):
				sum_l += lasers[i]
				sum_r += lasers[i + 4]
				sum_f += lasers[i + 2]
			sum_f /= 2
			sum_l /= 2
			sum_r /= 2

			# rospy.loginfo("Raw lasers values: " + str(lasers[0]) + " | " + str(lasers[1]) + " | " + str(lasers[2]) + " | " + str(lasers[3]) \
			# 		+ " | " + str(lasers[4]) + " | " + str(lasers[5]) + " | " + str(lasers[6]) + " | " + str(lasers[7]))

			# rospy.loginfo("Sum in regions: " + str(sum_l) + " | " + str(sum_f) + " | " + str(sum_r))
		
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
		rospy.loginfo("ang_mes avant: " + str(ang_mes))
		ang_mes = to_abs(ang_mes)
		ang_des = to_abs(ang_des)

		rospy.loginfo("ang_des: " + str(ang_des) + " ang_mes: " + str(ang_mes))

		err = ang_des - ang_mes

		# Case where the robot has to turn in anti-trigo way but goes the other
		if err < -math.radians(180):
			err += 2*math.pi

		# Choice of the strategy : rotating or moving forward
		#------------------------------------------------
		if(abs(err) > 0.01):
			cmd = Kp * err
			cmd = (cmd if (cmd<max_cmd) else max_cmd) if (cmd>0) else (cmd if (cmd>-max_cmd) else -max_cmd)
			if err > math.radians(180):
				speed_l = cmd
				speed_r = -cmd
			else:
				speed_l = -cmd
				speed_r = cmd
		else:
			rotating = False
			cmd = max_cmd
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