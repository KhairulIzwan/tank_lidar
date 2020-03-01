#!/usr/bin/env python

#Title: Python Subscriber for Laser Scan (RPLiDar)
#Author: Khairul Izwan Bin Kamsani - [12-02-2020]
#Description: Distance Detection Subcriber Nodes (Python) -- for Automatic Parking System

from __future__ import print_function
from __future__ import division

#remove or add the library/libraries for ROS
import sys
import rospy
import os
import cv2
import imutils
import math

#remove or add the message type
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool

LINEAR_VEL = 0.22
ANGULAR_VEL = 2.84
STOP_DISTANCE = 0.2
PARK_DISTANCE = 0.2
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
SAFE_PARK_DISTANCE = PARK_DISTANCE + LIDAR_ERROR

e = """
Communications Failed
"""

class Parking():
	def __init__(self):
		# Initializing ROS Node
		rospy.init_node('tank_parking_node', anonymous=True)
		
		# shutdown
		rospy.on_shutdown(self.shutdown)

		self.rate = rospy.Rate(1000) # 10hz

		# Publish to the cmd_vel topic
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		# Publish to the reset_left_encoder topic
		self.reset_leftEnc_pub = rospy.Publisher('/reset_leftEnc', Bool, queue_size=10)

		# Publish to the reset_right_encoder topic
		self.reset_rightEnc_pub = rospy.Publisher('/reset_rightEnc', Bool, queue_size=10)

		# Publish to the self.min_distance_front topic
		self.min_distance_front_pub = rospy.Publisher('/min_front', Float32, queue_size=10)

		# Publish to the self.min_distance_left topic
		self.min_distance_left_pub = rospy.Publisher('/min_left', Float32, queue_size=10)

		# Publish to the self.min_distance_right topic
		self.min_distance_right_pub = rospy.Publisher('/min_right', Float32, queue_size=10)

		# Subscribe to the encoder (Left) topic
		self.encLeft_sub = rospy.Subscriber("/left_encoder", Float32, self.callback_encLeft)
		
		# Subscribe to the encoder (Right) topic
		self.encRight_sub = rospy.Subscriber("/right_encoder", Float32, self.callback_encRight)

		# Subscribe to the scan topic
		# self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.callback_laser)
		self.scan_sub = rospy.Subscriber("/scan", LaserScan)
		self.scan_parking_spot()

	# Encode the subscribed scan topic
	def callback_laser(self, data):
		# Read on selected scan area
		self.get_scan(data)

		# Decision making
		self.scan_parking_spot()
		
	# Encode the subscribed left_encoder topic
	def callback_encLeft(self, data):
		self.leftEnc = data.data
		
	# Encode the subscribed right_encoder topic
	def callback_encRight(self, data):
		self.rightEnc = data.data

	def get_scan(self):
		# Wait for the topic
		scan = rospy.wait_for_message('/scan', LaserScan)
		
		# Create an empty array (for scan-ed data)
		self.scan_filter = []
		
		# Numbers of sample scan-ed
		samples = len(scan.ranges)
		
		# Numbers of data required	
		samples_view = 181
		
		# Normalize 360 only bruuhh
		if samples_view > samples:
			samples_view = samples
		
		# Viewing angle
		if samples_view is 1:
			self.scan_filter.append(scan.ranges[0])

		else:
			left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
			right_lidar_samples_ranges = samples_view//2

			left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
			right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
			self.scan_filter.extend(left_lidar_samples + right_lidar_samples)
		
		# Clean the datas
		for i in range(samples_view):
			if self.scan_filter[i] == float('Inf'):
				self.scan_filter[i] = 3.5
			elif math.isnan(self.scan_filter[i]):
				self.scan_filter[i] = 0

	def scan_parking_spot(self):
		# Initiate the topic
		self.twist = Twist()
		self.reset_leftEnc = Bool()
		self.reset_rightEnc = Bool()
		self.min_distance_front = Float32()
		self.min_distance_left = Float32()
		self.min_distance_right = Float32()

		# Initiate step
		step = 0
		
		while not rospy.is_shutdown():
			# Get the scan-ed data
			self.get_scan()

			# Check on min distance (Front, Left, and Right))
			self.min_distance_right.data = self.scan_filter[0]
			self.min_distance_front.data = self.scan_filter[90]
			self.min_distance_left.data = self.scan_filter[180]
			
			# First step, parking area entrance -- checking for parking
			if step == 0:
				# No Parking
				if self.min_distance_front.data > SAFE_STOP_DISTANCE and self.min_distance_left.data < SAFE_PARK_DISTANCE and self.min_distance_right.data < SAFE_PARK_DISTANCE:
					# Move forward
					self.twist.linear.x = LINEAR_VEL
					self.twist.angular.z = 0.0
					rospy.logerr('No Parking Space Available! Find Next')

				# Parking on both Left and Right	
				elif self.min_distance_front.data > SAFE_STOP_DISTANCE and self.min_distance_left.data > SAFE_PARK_DISTANCE and self.min_distance_right.data > SAFE_PARK_DISTANCE:
					# Stop
					self.twist.linear.x = 0.0
					self.twist.angular.z = 0.0
					rospy.logwarn('Parking Space Available! (Left and Right)')

					# reset_leftEnc and reset_rightEnc
					self.reset_leftEnc.data = True
					self.reset_rightEnc.data = True
					self.reset_leftEnc_pub.publish(self.reset_leftEnc)
					self.reset_rightEnc_pub.publish(self.reset_rightEnc)

					step = 1

				# Parking on Left	
				elif self.min_distance_front.data > SAFE_STOP_DISTANCE and self.min_distance_left.data > SAFE_PARK_DISTANCE and self.min_distance_right.data < SAFE_PARK_DISTANCE:
					# Stop
					self.twist.linear.x = 0.0
					self.twist.angular.z = 0.0
					rospy.logwarn('Parking Space Available! (Left)')

					# reset_leftEnc and reset_rightEnc
					self.reset_leftEnc.data = True
					self.reset_rightEnc.data = True
					self.reset_leftEnc_pub.publish(self.reset_leftEnc)
					self.reset_rightEnc_pub.publish(self.reset_rightEnc)

					step = 1

				# Parking on Right	
				elif self.min_distance_front.data > SAFE_STOP_DISTANCE and self.min_distance_left.data < SAFE_PARK_DISTANCE and self.min_distance_right.data > SAFE_PARK_DISTANCE:
					# Stop
					self.twist.linear.x = 0.0
					self.twist.angular.z = 0.0
					rospy.logwarn('Parking Space Available! (Right)')

					# reset_leftEnc and reset_rightEnc
					self.reset_leftEnc.data = True
					self.reset_rightEnc.data = True
					self.reset_leftEnc_pub.publish(self.reset_leftEnc)
					self.reset_rightEnc_pub.publish(self.reset_rightEnc)

					step = 2

			# Found parking spot available on both side -- choose Left over Right
			elif step == 1:
				rospy.logwarn('Turn to Left')
				if self.leftEnc <= -1000 and self.rightEnc >= 1000:
					# reset_leftEnc and reset_rightEnc
					self.reset_leftEnc.data = True
					self.reset_rightEnc.data = True
					self.reset_leftEnc_pub.publish(self.reset_leftEnc)
					self.reset_rightEnc_pub.publish(self.reset_rightEnc)

					step = 3
				else:
					self.twist.linear.x = 0.0
					self.twist.angular.z = ANGULAR_VEL

			# Found parking spot available on right side
			elif step == 2:
				rospy.logwarn('Turn to Right')
				if self.leftEnc >= 1000 and self.rightEnc <= -1000:
					# reset_leftEnc and reset_rightEnc
					self.reset_leftEnc.data = True
					self.reset_rightEnc.data = True
					self.reset_leftEnc_pub.publish(self.reset_leftEnc)
					self.reset_rightEnc_pub.publish(self.reset_rightEnc)

					step = 3
				else:
					self.twist.linear.x = 0.0
					self.twist.angular.z = -ANGULAR_VEL

			# Move in to parking slot
			elif step == 3:
				if self.min_distance_front.data > SAFE_STOP_DISTANCE:
					self.twist.linear.x = LINEAR_VEL
					self.twist.angular.z = 0.0
					rospy.logwarn('Move in to parking slot')
				else:
					step = 4

			# Autoparking Done
			elif step == 4:
				self.twist.linear.x = 0.0
				self.twist.angular.z = 0.0
				rospy.logwarn('Auto Parking Done')
				self.cmd_vel_pub.publish(self.twist)
				sys.exit()

			self.cmd_vel_pub.publish(self.twist)
			self.min_distance_front_pub.publish(self.min_distance_front)
			self.min_distance_left_pub.publish(self.min_distance_left)
			self.min_distance_right_pub.publish(self.min_distance_right)

			self.rate.sleep()

	def shutdown(self):
		try:
			self.twist.linear.x = 0.0
			self.twist.angular.z = 0.0
			self.cmd_vel_pub.publish(self.twist)
			rospy.loginfo("Parking Detection. [OFFLINE]...")
		finally:
			pass

def main(args):
	try:
		vn = Parking()
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Parking Detection. [OFFLINE]...")

if __name__ == '__main__':
	rospy.loginfo("Parking Detection. [ONLINE]...")
	main(sys.argv)
