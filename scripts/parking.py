#!/usr/bin/env python

#Title: Python Subscriber for Laser Scan (RPLiDar)
#Author: Khairul Izwan Bin Kamsani - [12-02-2020]
#Description: Distance Detection Subcriber Nodes (Python)

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
from std_msgs.msg import Float32

LINEAR_VEL = 0.22
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
		
		# 
		rospy.on_shutdown(self.shutdown)

		# Subscribe to the scan topic
		self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.callback_laser)

		# Subscribe to the encoder (Left) topic
		self.encLeft_sub = rospy.Subscriber("/left_encoder", Float32, self.callback_encLeft)
		
		# Subscribe to the encoder (Right) topic
		self.encRight_sub = rospy.Subscriber("/right_encoder", Float32, self.callback_encRight)
		
		# Publish to the cmd_vel topic
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

	# Encode the subscribed scan topic
	def callback_laser(self, data):
		# Read on selected scan area
		self.get_scan(data)

		# Decision making
		self.find_parking()

	def get_scan(self, scan):
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

	def find_parking(self):
		# Initiate the topic
		self.twist = Twist()
		
		# Initiate turtlebot movement
		turtlebot_moving = True

		# Check on min distance (Front, Left, and Right))
		min_distance_right = self.scan_filter[0]
		min_distance_front = self.scan_filter[90]
		min_distance_left = self.scan_filter[180]

		#rospy.loginfo([min_distance_left, min_distance_front, min_distance_right])

		if min_distance_front < SAFE_STOP_DISTANCE:
			if turtlebot_moving:
				self.twist.linear.x = 0.0
				self.twist.angular.z = 0.0
				self.cmd_vel_pub.publish(self.twist)
				turtlebot_moving = False
				rospy.logerr('Stop!')
		
		elif min_distance_left > SAFE_PARK_DISTANCE and min_distance_right < SAFE_PARK_DISTANCE:
			if turtlebot_moving:
				self.twist.linear.x = 0.0
				self.twist.angular.z = 0.0
				self.cmd_vel_pub.publish(self.twist)
				turtlebot_moving = False
				rospy.loginfo('Parking Available on Left!')
				
		elif min_distance_left < SAFE_PARK_DISTANCE and min_distance_right > SAFE_PARK_DISTANCE:
			if turtlebot_moving:
				self.twist.linear.x = 0.0
				self.twist.angular.z = 0.0
				self.cmd_vel_pub.publish(self.twist)
				turtlebot_moving = False
				rospy.loginfo('Parking Available on Right!')
		
		elif min_distance_left > SAFE_PARK_DISTANCE and min_distance_right > SAFE_PARK_DISTANCE:
			if turtlebot_moving:
				self.twist.linear.x = 0.0
				self.twist.angular.z = 0.0
				self.cmd_vel_pub.publish(self.twist)
				turtlebot_moving = False
				rospy.loginfo('Parking Available on Left and Right!')
		
		else:
			self.twist.linear.x = LINEAR_VEL
			self.twist.angular.z = 0.0
			self.cmd_vel_pub.publish(self.twist)
			turtlebot_moving = True
			rospy.logwarn("Searching for Parking!")
			rospy.loginfo('Distance of the left, front, and right : %f: %f: %f' % (min_distance_left, min_distance_front, min_distance_right))

	def shutdown(self):
		try:
			self.twist.linear.x = 0.0
			self.twist.angular.z = 0.0
			self.cmd_vel_pub.publish(self.twist)
			rospy.loginfo("Parking Detection. [OFFLINE]...")
		finally:
			pass

def main(args):
	vn = Parking()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Parking Detection. [OFFLINE]...")

if __name__ == '__main__':
	rospy.loginfo("Parking Detection. [ONLINE]...")
	main(sys.argv)
