#!/usr/bin/env python3
import os
import rospy
from duckietown.dtros import DTROS, NodeType
import numpy as np
from duckietown_msgs.msg import LanePose, Twist2DStamped, WheelEncoderStamped, FusionLanePose, WheelsCmdStamped #EncoderTicksStamped
from std_msgs.msg import Header

from sensor_msgs.msg import CompressedImage



class Sensor_Fusion(DTROS):

	def __init__(self, node_name):

		# initialize the DTROS parent class
		super(Sensor_Fusion, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

		self.last_left_ticks = 0
		self.last_right_ticks = 0
		self.tick_to_meter = np.pi * 0.067 / 135.   # 6.5cm diameter, 135 = #ticks per revolution
		self.z_m = np.zeros((3, 1)) # d_enc, phi_enc, d_cam, phi_cam
		self.old_z_m0 = 0
		self.wheelbase = 0.099 #0.102 # ueli 1: 0.104, ueli 2: 0.101----1.03
		self.msg_fusion = FusionLanePose()

		self.time_last_image = 0
		self.time_last_est = 0
		self.x = np.zeros((2,1))
		self.P = np.eye(2) #
		self.K = np.zeros((2,3))
		self.Q = 0.5 * np.eye(2) #
		self.q = 5 # review
		self.R = 0.2 * np.eye(3) #
		self.R[0][0] = 0.0025 #0.1   0.0025
		self.R[1][1] = 0.04 #0.05    0.04
		self.R[2][2] = 0.01 #0.2   0.0025
		self.I = np.eye(2)
		self.H = np.array([[0, 1], [1, 0], [0, 1]])

		self.first_encoder_meas_l = 0
		self.first_encoder_meas_r = 0

		self.i_first_calibration = 0 # number of iterat. for first calib

		##recalibration:

		self.recalib_status = 0
		self.count = 0

		##
		self.distance = 0
		self.old_distance = 0
		self.save_alpha = np.zeros((20,1))
		self.save_timestamp = np.zeros((20,1))
		self.save_distance = np.zeros((20,1))
		self.save_delta_d = np.zeros((20,1))
		self.a = 0
		self.b = 0

		# vel encoder
		self.vel_left = 1
		self.vel_right = 1
		self.old_time_now = 0

		self.direction_l = 1
		self.direction_r = 1

		self.ignore_only_cam = 0
		self.turn = 0
		self.recalib_distance = 0

		# higher pub rate
		self.higher_pub_rate = 0 # 1 = on; 0 = off
		self.ticks_for_pub = 14 # review
		self.sum_alpha = 0
		self.sum_ticks = 0

		# Publisher
		self.pub_pose_est = rospy.Publisher("~fusion_lane_pose", FusionLanePose, queue_size=1) # for lane_controller_node

		# Subscriber
		#self.sub_encoder_ticks =rospy.Subscriber("encoder_ticks_node/encoder_ticks", EncoderTicksStamped, self.update_encoder_measurement, queue_size=1)
		self.sub_camera_pose =rospy.Subscriber("lane_filter_node/lane_pose", LanePose, self.predict, queue_size=1)
		self.sub_controls = rospy.Subscriber("lane_controller_node/car_cmd", Twist2DStamped, self.save_inputs, queue_size=1)	
		#/sepp/left_wheel_encoder_node/tick
		self.sub_encoder_ticks =rospy.Subscriber("left_wheel_encoder_node/tick", WheelEncoderStamped, self.update_encoder_measurement_l, queue_size=1)
		self.sub_encoder_ticks =rospy.Subscriber("right_wheel_encoder_node/tick", WheelEncoderStamped, self.update_encoder_measurement_r, queue_size=1)
		self.sub_encoder_ticks =rospy.Subscriber("wheels_driver_node/wheels_cmd_executed", WheelsCmdStamped, self.save_wheelscmdstamped, queue_size=1)

	def save_inputs(self, msg_Twist2DStamped):
		#self.v = msg_Twist2DStamped.v
		#self.omega = msg_Twist2DStamped.omega
		return

	def save_wheelscmdstamped(self, msg_wheelscmdstamped):
		self.direction_l = np.sign(msg_wheelscmdstamped.vel_left)
		self.direction_r = np.sign(msg_wheelscmdstamped.vel_right)

	def update_encoder_measurement_l(self, msg_encoder_ticks):
		if self.first_encoder_meas_l == 0:
			self.last_left_ticks = msg_encoder_ticks.data
			self.first_encoder_meas_l = 1

		else:
			#rospy.loginfo("l: %s" %(msg_encoder_ticks.data))
			self.diff_left = (msg_encoder_ticks.data - self.last_left_ticks) * self.direction_l
			self.last_left_ticks = msg_encoder_ticks.data
			alpha = self.tick_to_meter * self.diff_left / self.wheelbase
			self.z_m[0] -= alpha
			self.b -= alpha
			#rospy.loginfo("r_%s" %(self.diff_left))
			diff_dist = self.tick_to_meter * 0.5 * self.diff_left
			#self.left_dist_vel += diff_dist * 2 # !!! vel_enc
			self.save_encoder_measurements(diff_dist, msg_encoder_ticks, alpha)
		
		if self.higher_pub_rate == 1:
			if self.sum_ticks > self.ticks_for_pub:
				#addition_phi, addition_d, not_used = self.encoder_information_until_now(msg_encoder_ticks.header.stamp.secs + msg_encoder_ticks.header.stamp.nsecs/1e9, self.x[1])
				self.msg_fusion.header.stamp = rospy.get_rostime()
				self.msg_fusion.d = self.x[0] * 1.5 ##!!!!!
				self.msg_fusion.phi = self.x[1]
				self.pub_pose_est.publish(self.msg_fusion)

		return

	def update_encoder_measurement_r(self, msg_encoder_ticks):
		if self.first_encoder_meas_r == 0:
			self.last_right_ticks = msg_encoder_ticks.data
			self.first_encoder_meas_r = 1

		else:
			#rospy.loginfo("r: %s" %(msg_encoder_ticks.data))
			self.diff_right = (msg_encoder_ticks.data - self.last_right_ticks) * self.direction_r
			self.last_right_ticks = msg_encoder_ticks.data
			alpha = self.tick_to_meter * self.diff_right / self.wheelbase
			self.z_m[0] += alpha
			self.b += alpha
			#rospy.loginfo("l_%s" %(self.diff_right))
			diff_dist = self.tick_to_meter * 0.5 * self.diff_right
			#self.right_dist_vel += diff_dist * 2 # vel_enc
			self.save_encoder_measurements(diff_dist, msg_encoder_ticks, alpha)

		if self.higher_pub_rate == 1:
			if self.sum_ticks > self.ticks_for_pub:
				#addition_phi, addition_d, not_used = self.encoder_information_until_now(msg_encoder_ticks.header.stamp.secs + msg_encoder_ticks.header.stamp.nsecs / 1e9, self.x[1])
				self.msg_fusion.header.stamp = msg_encoder_ticks.header.stamp
				self.msg_fusion.d = self.x[0] * 1.5 ##!!!!!
				self.msg_fusion.phi = self.x[1]
				self.pub_pose_est.publish(self.msg_fusion)

		return

	def save_encoder_measurements(self, diff_dist, msg_encoder_ticks, alpha):

		self.distance += diff_dist

		self.save_alpha = np.append(self.save_alpha, alpha)
		self.save_alpha = np.delete(self.save_alpha, 0)
		self.save_timestamp = np.append(self.save_timestamp, msg_encoder_ticks.header.stamp.secs + msg_encoder_ticks.header.stamp.nsecs/1e9)
		self.save_timestamp = np.delete(self.save_timestamp, 0)

		self.save_distance = np.append(self.save_distance, diff_dist)
		self.save_distance = np.delete(self.save_distance, 0)

		return


	def curve_detection(self):


		if abs(self.z_m[0] - self.x[1]) > 0.6:

			return(1)

		else:
			return(0)

	


	def first_calibration(self, phi_cam):

		self.z_m[0] += 0.015 * phi_cam 
		self.i_first_calibration += 1

		if self.i_first_calibration == 50:
			self.time_last_est = rospy.get_rostime().secs + rospy.get_rostime().nsecs / 1e9
			self.time_last_image = self.time_last_est

		return



	def recalibration(self):
		if self.turn == 3 and self.recalib_status == 0:
			self.recalib_distance = self.distance
			self.recalib_status = 1
			rospy.logwarn("1")

		if self.recalib_status == 1 and self.distance - self.recalib_distance > 0.3 and self.distance - self.recalib_distance < 0.6:
			self.b +=1
			self.a += self.z_m[0]
			rospy.loginfo("recalib")

		elif self.recalib_status == 1 and self.distance -self.recalib_distance > 0.6:
			c = self.a /self.b
			rospy.loginfo("gemittelt:%s" %c)
			self.recalib_status = 0
			if abs(c) > 0.1:
				self.count += 1
				self.turn = 2
				self.a = 0
				if self.count == 2:
					self.z_m[0] -= 1.2 * c
					rospy.logwarn("RECALIB")
					self.count = 0
					self.turn = 0
					self.b = 0
			else:
				self.a = 0
				self.b = 0
				self.count = 0
				self.turn = 0
		return





	def encoder_information_until_now(self, t_image, phi):

		addition_phi = 0
		addition_d = 0
		addition_distance = 0
		for i in range(0,20):
			if self.save_timestamp[i] > t_image:
				##
				phi += self.save_alpha[i]
				addition_d += self.save_distance * np.sin(phi)
				addition_phi += self.save_alpha[i]
				addition_distance += self.save_distance[i]

		return(addition_phi, addition_distance, addition_d)



	def predict(self, msg_camera_pose):

		self.sum_alpha = 0

		self.x[0] = msg_camera_pose.d
		self.x[1] = msg_camera_pose.phi

		time_image = msg_camera_pose.header.stamp.secs + msg_camera_pose.header.stamp.nsecs / 1e9 #nu
		addition_phi, addition_d, not_used = self.encoder_information_until_now(time_image, self.x[1])
		
		if self.curve_detection() == 1:
			addition_d = 0
			addition_phi = 0
		

		self.x[0] += addition_d
		self.x[1] += addition_phi

		self.msg_fusion.header.stamp = rospy.get_rostime()
		self.msg_fusion.d = self.x[0] * 1.5 ##!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		self.msg_fusion.phi = self.x[1]
		self.pub_pose_est.publish(self.msg_fusion)
		
		return



if __name__ == '__main__':
	node = Sensor_Fusion(node_name='my_sensor_fusion_node')
	rospy.loginfo("sensor_fusion_node is up and running...")
	rospy.spin()
