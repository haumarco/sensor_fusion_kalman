#!/usr/bin/env python3
import os
import rospy
from duckietown.dtros import DTROS, NodeType
import numpy as np
from duckietown_msgs.msg import LanePose, Twist2DStamped, WheelEncoderStamped, FusionLanePose, WheelsCmdStamped, WheelsCmdStampedEncoder#EncoderTicksStamped
from std_msgs.msg import Header

from sensor_msgs.msg import CompressedImage



class Sensor_Fusion(DTROS):

	def __init__(self, node_name):

		# initialize the DTROS parent class
		super(Sensor_Fusion, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

		# covariance noise matrices
		self.R = np.zeros((2,3))
		self.R[0][0] = 0.01		#!
		self.R[1][1] = 0.01
		self.R[2][2] = 0.04
		self.Q = 0.005 * np.eye(2)

		# higher publishing frequency
		self.higher_pub_rate = 0 # 1 = on; 0 = off
		self.ticks_for_pub = 14 # amount of ticks between published poses
		
		# parameter
		self.tick_to_meter = np.pi * 0.067 / 135.   # 6.7cm diameter, 135 = #ticks per revolution
		self.wheeltrack = 0.101 

		# msg
		self.msg_fusion = LanePose()
		self.msg_enc_pose = LanePose()

		# values which need to be available over multiple calls
		self.z_m = np.zeros((3, 1)) # (d_enc, phi_enc, d_cam, phi_cam)^T
		self.old_z_m0 = 0
		self.last_left_ticks, self.last_right_ticks = 0, 0
		self.time_last_image = 0
		self.first_encoder_meas_l, self.first_encoder_meas_r = 0, 0
		self.i_first_calibration = 0 
		self.distance, self.old_distance = 0, 0
		self.direction_l, self.direction_r = 1, 1
		self.only_cam = 0
		self.sum_alpha, self.sum_ticks = 0, 0
		self.recalib_counter, self.recalib_old_phi = 0, 0
		self.ticks_encoder_left, self.ticks_encoder_left_old = 0, 0
		self.ticks_encoder_right, self.ticks_encoder_right_old = 0, 0

		self.x = np.zeros((2,1))
		self.P = np.eye(2)
		self.K = np.zeros((2,3))
		self.I = np.eye(2)
		self.H = np.array([[0, 1], [1, 0], [0, 1]])
		self.save_alpha = np.zeros((80,1))
		self.save_timestamp = np.zeros((80,1))
		self.save_distance = np.zeros((80,1))

		# Publisher
		self.pub_pose_est = rospy.Publisher("~fusion_lane_pose", LanePose, queue_size=10) # for lane_controller_node
		#self.pub_enc_est = rospy.Publisher("~encoder_pose", LanePose, queue_size=10) ## just for test

		# Subscriber
		self.sub_camera_pose =rospy.Subscriber("lane_filter_node/lane_pose", LanePose, self.SF, queue_size=1)
		self.sub_encoder_ticks =rospy.Subscriber("left_wheel_encoder_node/tick", WheelEncoderStamped, self.update_encoder_measurement_l, queue_size=1)
		self.sub_encoder_ticks =rospy.Subscriber("right_wheel_encoder_node/tick", WheelEncoderStamped, self.update_encoder_measurement_r, queue_size=1)
		self.sub_encoder_ticks =rospy.Subscriber("wheels_driver_node/wheels_cmd_executed", WheelsCmdStamped, self.save_wheelscmdstamped, queue_size=1)


	def save_wheelscmdstamped(self, msg_wheelscmdstamped):
		self.direction_l = np.sign(msg_wheelscmdstamped.vel_left)
		self.direction_r = np.sign(msg_wheelscmdstamped.vel_right)
		return

	def update_encoder_measurement_l(self, msg_encoder_ticks):
		if self.first_encoder_meas_l == 0:
			self.last_left_ticks = msg_encoder_ticks.data
			self.first_encoder_meas_l = 1

		else:
			self.ticks_encoder_left = msg_encoder_ticks.data
			diff_left = (msg_encoder_ticks.data - self.last_left_ticks) * self.direction_l
			self.last_left_ticks = msg_encoder_ticks.data
			alpha = - self.tick_to_meter * diff_left / self.wheeltrack
			self.z_m[0] += alpha
			diff_dist = self.tick_to_meter * 0.5 * diff_left
			self.save_encoder_measurements(diff_dist, msg_encoder_ticks, alpha)

		
			if self.higher_pub_rate == 1:
				self.high_pub_rate(alpha, diff_left)

		return

	def update_encoder_measurement_r(self, msg_encoder_ticks):
		if self.first_encoder_meas_r == 0:
			self.last_right_ticks = msg_encoder_ticks.data
			self.first_encoder_meas_r = 1

		else:
			self.ticks_encoder_right = msg_encoder_ticks.data
			diff_right = (msg_encoder_ticks.data - self.last_right_ticks) * self.direction_r
			self.last_right_ticks = msg_encoder_ticks.data
			alpha = self.tick_to_meter * diff_right / self.wheeltrack
			self.z_m[0] += alpha
			diff_dist = self.tick_to_meter * 0.5 * diff_right
			self.save_encoder_measurements(diff_dist, msg_encoder_ticks, alpha)

			if self.higher_pub_rate == 1:
				self.high_pub_rate(alpha, diff_right)

		return

	def high_pub_rate(self, alpha, diff):
		self.sum_alpha += alpha
		self.sum_ticks += diff
		if self.sum_ticks > self.ticks_for_pub:
			self.msg_fusion.header.stamp = rospy.get_rostime()
			# d - take last published value, which is already self.msg_fusion.d
			self.msg_fusion.phi = self.msg_fusion.phi + self.sum_alpha
			self.msg_fusion.in_lane = True
			self.pub_pose_est.publish(self.msg_fusion)
			self.sum_alpha = 0
			self.sum_ticks = 0


	def save_encoder_measurements(self, diff_dist, msg_encoder_ticks, alpha):

		self.distance += diff_dist

		self.save_alpha = np.append(self.save_alpha, alpha)
		self.save_alpha = np.delete(self.save_alpha, 0)
		self.save_timestamp = np.append(self.save_timestamp, rospy.get_rostime().secs + rospy.get_rostime().nsecs / 1e9)
		self.save_timestamp = np.delete(self.save_timestamp, 0)
		self.save_distance = np.append(self.save_distance, diff_dist)
		self.save_distance = np.delete(self.save_distance, 0)

		return


	def curve_detection(self, addition_phi, addition_d):

		if abs(self.z_m[0]) > 1:
			self.z_m[0] -= np.sign(self.z_m[0]) * np.pi /2
			self.only_cam = 1


		if abs(self.z_m[0] - self.z_m[2]) > 0.5 and self.only_cam == 0:
			self.msg_fusion.header.stamp = rospy.get_rostime()
			self.msg_fusion.d = self.z_m[1] #+ addition_d
			self.msg_fusion.phi = self.z_m[2] #+ addition_phi
			self.msg_fusion.in_lane = True
			self.pub_pose_est.publish(self.msg_fusion)
			self.x[0] = self.msg_fusion.d
			self.x[1] = self.msg_fusion.phi
			return(1)
		else:
			self.only_cam = 0

		return(0)

	


	def first_calibration(self, phi_cam):

		self.z_m[0] += 0.015 * phi_cam 
		self.i_first_calibration += 1

		if self.i_first_calibration == 50:
			self.time_last_image = rospy.get_rostime().secs + rospy.get_rostime().nsecs / 1e9





	def recalibration(self):
		if self.z_m[2] == self.recalib_old_phi and abs(self.z_m[2]) < 0.2:
			self.recalib_counter += 1
		else:
			self.recalib_counter = 0
		
		if self.recalib_counter == 4:		
			self.z_m[0] = 0.75 * self.z_m[0] + 0.25 * self.z_m[2]
			self.recalib_counter = 0

		self.recalib_old_phi = np.copy(self.z_m[2])
		return



	def encoder_information_until_now(self, t_image, phi):

		addition_phi = 0
		addition_d = 0
		addition_distance = 0
		for i in range(0,80):
			if self.save_timestamp[i] > t_image:
				phi += self.save_alpha[i]
				addition_d += self.save_distance[i] * np.sin(phi)
				addition_phi += self.save_alpha[i]
				addition_distance += self.save_distance[i]


		return(addition_phi, float(addition_d), addition_distance)



	def SF(self, msg_camera_pose):

		if self.i_first_calibration  < 50:
			self.first_calibration(msg_camera_pose.phi)
			return

		self.sum_alpha = 0

		self.z_m[1] = msg_camera_pose.d
		self.z_m[2] = msg_camera_pose.phi

		time_image = msg_camera_pose.header.stamp.secs + msg_camera_pose.header.stamp.nsecs / 1e9
		delta_t = time_image - self.time_last_image

		addition_phi, addition_d, addition_distance = self.encoder_information_until_now(time_image, self.x[1])

		if self.curve_detection(addition_phi, addition_d) == 1:
			return

		self.z_m[0] -= addition_phi



		# prediction part
		
		omega = (self.z_m[0] - self.old_z_m0)/delta_t
		self.old_z_m0 = np.copy(self.z_m[0])
		delta_d = self.distance - addition_distance - self.old_distance
		self.old_distance = self.distance - addition_distance
		
		A = np.array([[1, delta_d],[0, 1]]) # self.v* delta_t = delta_d
		B = np.array([[0.5 * delta_d * delta_t], [delta_t]]) # self.v* delta_t**2 = delta_d * delta_t
		self.x = np.dot(A, self.x) + np.dot(B, omega[0])
		self.P = np.dot(np.dot(A, self.P), A.T) + self.Q

		self.time_last_image = time_image


		# correction part

		S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
		self.K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))

		y = self.z_m - np.dot(self.H, self.x)
		self.x += np.dot(self.K, y)
		J = self.I - np.dot(self.K, self.H)
		self.P = np.dot(J, self.P)

		self.msg_fusion.header.stamp = rospy.get_rostime()
		self.msg_fusion.d = self.x[0] + addition_d
		self.msg_fusion.phi = self.x[1] + addition_phi
		self.msg_fusion.in_lane = msg_camera_pose.in_lane
		self.pub_pose_est.publish(self.msg_fusion)
		
		self.z_m[0] += addition_phi

		self.ticks_encoder_left_old = self.ticks_encoder_left
		self.ticks_encoder_right_old = self.ticks_encoder_right

		self.recalibration()

		# publish encoder pose seperatly
		#	dont forget to uncomment Publisher
		# self.msg_enc_pose.header.stamp = self.msg_fusion.header.stamp
		# self.msg_enc_pose.d = 0
		# self.msg_enc_pose.phi = self.z_m[0]
		# self.pub_enc_est.publish(self.msg_enc_pose)

		return



if __name__ == '__main__':
	node = Sensor_Fusion(node_name='my_sensor_fusion_node')
	rospy.loginfo("sensor_fusion_node is up and running...")
	rospy.spin()
