#!/usr/bin/env python
import os
import rospy
from duckietown import DTROS
import numpy as np
from duckietown_msgs.msg import LanePose, Twist2DStamped, EncoderTicksStamped, FusionLanePose
from std_msgs.msg import Header

from sensor_msgs.msg import CompressedImage



class Sensor_Fusion(DTROS):

	def __init__(self, node_name):

		# initialize the DTROS parent class
		super(Sensor_Fusion, self).__init__(node_name=node_name)

		self.diff_left = 0
		self.diff_right = 0
		self.last_left_ticks = 0
		self.last_right_ticks = 0
		self.tick_to_meter = np.pi * 0.067 / 135.   # 6.5cm diameter, 135 = #ticks per revolution
		self.z_m = np.zeros((4, 1)) # d_enc, phi_enc, d_cam, phi_cam
		self.old_z_m1 = 0
		self.wheelbase = 0.103 # ueli 1: 0.104, ueli 2: 0.101----1.03
		self.msg_fusion = FusionLanePose()
		self.first_left = 0
		self.first_right = 0
		self.first_encoder_meas = 0


		# Publisher
		self.pub_pose_est = rospy.Publisher("~fusion_lane_pose", FusionLanePose, queue_size=1) # for lane_controller_node

		# Subscriber
		self.sub_encoder_ticks =rospy.Subscriber("encoder_ticks_node/encoder_ticks", EncoderTicksStamped, self.update_encoder_measurement, queue_size=1)
	



	def update_encoder_measurement(self, msg_encoder_ticks):
		if self.first_encoder_meas == 0:
			self.first_left = msg_encoder_ticks.left_ticks
			self.first_right = msg_encoder_ticks.right_ticks
			self.last_left_ticks = self.first_left
			self.last_right_ticks = self.first_right
			self.first_encoder_meas = 1
			return
		
		self.diff_left = msg_encoder_ticks.left_ticks - self.last_left_ticks
		self.diff_right = msg_encoder_ticks.right_ticks - self.last_right_ticks
		self.last_left_ticks = msg_encoder_ticks.left_ticks
		self.last_right_ticks = msg_encoder_ticks.right_ticks
		# self.diff_left = msg_encoder_ticks.left_ticks
		# self.diff_right = msg_encoder_ticks.right_ticks

		alpha = self.tick_to_meter * (self.diff_right - self.diff_left) / self.wheelbase

		self.z_m[1] += alpha


		if alpha == 0:
			self.z_m[0] += self.diff_left * self.tick_to_meter * np.sin(self.z_m[1]) 

		else:
			self.z_m[0] += self.tick_to_meter * 0.5 * (self.diff_left + self.diff_right) / alpha * (np.cos(self.old_z_m1) - np.cos(self.z_m[1]))
		

		self.old_z_m1 = np.copy(self.z_m[1])

		ticks = 0.5 * (msg_encoder_ticks.left_ticks + msg_encoder_ticks.right_ticks - self.first_left -self.first_right)
		formula = 135 /(np.pi * ticks)

		rospy.loginfo("#ticks: %s  ->  diameter = %s * measured_distance" %(ticks, formula))

		# drive straight
		self.msg_fusion.header.stamp = rospy.get_rostime()
		self.msg_fusion.d = self.z_m[0]
		self.msg_fusion.phi = self.z_m[1]
		self.pub_pose_est.publish(self.msg_fusion)
		return




if __name__ == '__main__':
	node = Sensor_Fusion(node_name='my_sensor_fusion_node')
	rospy.loginfo("sensor_fusion_node is up and running...")
	rospy.spin()
