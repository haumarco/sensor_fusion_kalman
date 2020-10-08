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
		self.wheelbase = 0.104 #0.102 # ueli 1: 0.104, ueli 2: 0.101----1.03
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
		self.recalib_counter = 0
		self.recalib_phi = 0

		# higher pub rate
		self.higher_pub_rate = 1 # 1 = on; 0 = off
		self.ticks_for_pub = 10 # review
		self.sum_alpha = 0
		self.sum_ticks = 0

		# Publisher
		self.pub_pose_est = rospy.Publisher("~fusion_lane_pose", FusionLanePose, queue_size=1) # for lane_controller_node

		# Subscriber
		#self.sub_encoder_ticks =rospy.Subscriber("encoder_ticks_node/encoder_ticks", EncoderTicksStamped, self.update_encoder_measurement, queue_size=1)
		self.sub_camera_pose =rospy.Subscriber("lane_filter_node/lane_pose", LanePose, self.predict, queue_size=1)
		self.sub_encoder_ticks =rospy.Subscriber("left_wheel_encoder_node/tick", WheelEncoderStamped, self.update_encoder_measurement_l, queue_size=1)
		self.sub_encoder_ticks =rospy.Subscriber("right_wheel_encoder_node/tick", WheelEncoderStamped, self.update_encoder_measurement_r, queue_size=1)
		self.sub_encoder_ticks =rospy.Subscriber("wheels_driver_node/wheels_cmd_executed", WheelsCmdStamped, self.save_wheelscmdstamped, queue_size=1)


	def save_wheelscmdstamped(self, msg_wheelscmdstamped):
		# self.direction_l = np.sign(msg_wheelscmdstamped.vel_left)
		# self.direction_r = np.sign(msg_wheelscmdstamped.vel_right)
		# if self.direction_l == -1 or self.direction_r == -1:
		# 	rospy.logwarn("MINUS")
		return

	def update_encoder_measurement_l(self, msg_encoder_ticks):
		if self.first_encoder_meas_l == 0:
			self.last_left_ticks = msg_encoder_ticks.data
			self.first_encoder_meas_l = 1

		else:
			#rospy.loginfo("l: %s" %(msg_encoder_ticks.data))
			diff_left = (msg_encoder_ticks.data - self.last_left_ticks) * self.direction_l
			self.last_left_ticks = msg_encoder_ticks.data
			alpha = self.tick_to_meter * diff_left / self.wheelbase
			self.z_m[0] -= alpha
			self.b -= alpha
			#rospy.loginfo("r_%s" %(diff_left))
			diff_dist = self.tick_to_meter * 0.5 * diff_left
			self.save_encoder_measurements(diff_dist, msg_encoder_ticks, alpha)
		
			if self.higher_pub_rate == 1:
				self.sum_alpha += alpha
				self.sum_ticks += diff_left
				if self.sum_ticks > self.ticks_for_pub:
					addition_phi, addition_d, not_used = self.encoder_information_until_now(msg_encoder_ticks.header.stamp.secs + msg_encoder_ticks.header.stamp.nsecs/1e9, self.x[1])
					self.msg_fusion.header.stamp = rospy.get_rostime()
					self.msg_fusion.d = (self.x[0] + addition_d)* 1.5 ##!!!!!
					self.msg_fusion.phi = self.x[1] + addition_phi
					self.msg_fusion.in_lane = True
					self.pub_pose_est.publish(self.msg_fusion)
					self.sum_alpha = 0
					self.sum_ticks = 0
					#rospy.loginfo("high_pub_enc phi: %s   d: %s" %(self.msg_fusion.phi, self.msg_fusion.d))

		return

	def update_encoder_measurement_r(self, msg_encoder_ticks):
		if self.first_encoder_meas_r == 0:
			self.last_right_ticks = msg_encoder_ticks.data
			self.first_encoder_meas_r = 1

		else:
			#rospy.loginfo("r: %s" %(msg_encoder_ticks.data))
			diff_right = (msg_encoder_ticks.data - self.last_right_ticks) * self.direction_r
			self.last_right_ticks = msg_encoder_ticks.data
			alpha = self.tick_to_meter * diff_right / self.wheelbase
			self.z_m[0] += alpha
			self.b += alpha
			#rospy.loginfo("TIMESTAMP %s.%s" %(msg_encoder_ticks.header.stamp.secs, msg_encoder_ticks.header.stamp.nsecs))
			diff_dist = self.tick_to_meter * 0.5 * diff_right
			self.save_encoder_measurements(diff_dist, msg_encoder_ticks, alpha)

			if self.higher_pub_rate == 1:
				self.sum_alpha += alpha
				self.sum_ticks += diff_right
				if self.sum_ticks > self.ticks_for_pub:
					addition_phi, addition_d, not_used = self.encoder_information_until_now(msg_encoder_ticks.header.stamp.secs + msg_encoder_ticks.header.stamp.nsecs/1e9, self.x[1])
					self.msg_fusion.header.stamp = rospy.get_rostime()
					self.msg_fusion.d = (self.x[0] + addition_d)* 1.5 ##!!!!!
					self.msg_fusion.phi = self.x[1] + addition_phi
					self.msg_fusion.in_lane = True
					self.pub_pose_est.publish(self.msg_fusion)
					self.sum_alpha = 0
					self.sum_ticks = 0
					#rospy.loginfo("high_pub_enc phi: %s   d: %s" %(self.msg_fusion.phi, self.msg_fusion.d))


		return

	def save_encoder_measurements(self, diff_dist, msg_encoder_ticks, alpha):

		self.distance += diff_dist

		self.save_alpha = np.append(self.save_alpha, alpha)
		self.save_alpha = np.delete(self.save_alpha, 0)
		#self.save_timestamp = np.append(self.save_timestamp, msg_encoder_ticks.header.stamp.secs + msg_encoder_ticks.header.stamp.nsecs/1e9)
		self.save_timestamp = np.append(self.save_timestamp, rospy.get_rostime().secs + rospy.get_rostime().nsecs / 1e9)
		self.save_timestamp = np.delete(self.save_timestamp, 0)
		self.save_distance = np.append(self.save_distance, diff_dist)
		self.save_distance = np.delete(self.save_distance, 0)

		return


	def curve_detection(self, addition_phi, addition_d):

		if abs(self.z_m[0]) > 1:
			rospy.loginfo("turn %s" %(self.z_m[0]))
			self.z_m[0] -= np.sign(self.z_m[0]) * np.pi /2
			self.ignore_only_cam = 1
			self.turn += 1


		if abs(self.z_m[0] - self.z_m[2]) > 0.6 and self.ignore_only_cam == 0:
			rospy.loginfo("only cam c: %s  e: %s %s" %(self.z_m[2], self.z_m[0], self.b))
			self.msg_fusion.header.stamp = rospy.get_rostime()
			#rospy.loginfo("%s %s" %(msg_camera_pose.d, msg_camera_pose.phi))
			self.msg_fusion.d = self.z_m[1] + addition_d
			self.msg_fusion.phi = self.z_m[2] + addition_phi
			self.pub_pose_est.publish(self.msg_fusion)
			self.x[0] = self.msg_fusion.d
			self.x[1] = self.msg_fusion.phi
			return(1)
		else:
			self.ignore_only_cam = 0

		return(0)

	


	def first_calibration(self, phi_cam):

		self.z_m[0] += 0.015 * phi_cam 
		self.i_first_calibration += 1

		if self.i_first_calibration == 50:
			self.time_last_est = rospy.get_rostime().secs + rospy.get_rostime().nsecs / 1e9
			self.time_last_image = self.time_last_est




	def recalibration(self):
		if self.turn == 3 and self.recalib_status == 0:
			self.recalib_distance = self.distance
			self.recalib_status = 1
			rospy.logwarn("1")

		if self.recalib_status == 1 and self.distance - self.recalib_distance > 0.3 and self.distance - self.recalib_distance < 0.6:
			self.recalib_counter +=1
			self.recalib_phi += self.z_m[0]
			rospy.loginfo("recalib")

		elif self.recalib_status == 1 and self.distance -self.recalib_distance > 0.6:
			c = self.recalib_phi /self.recalib_counter
			rospy.loginfo("gemittelt:%s" %c)
			self.recalib_status = 0
			if abs(c) > 0.1:
				self.count += 1
				self.turn = 2
				self.recalib_phi = 0
				if self.count == 2:
					self.z_m[0] -= 1.2 * c
					rospy.logwarn("RECALIB")
					self.count = 0
					self.turn = 0
					self.recalib_counter = 0
			else:
				self.recalib_phi = 0
				self.recalib_counter = 0
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
				addition_d += self.save_distance[i] * np.sin(phi)
				addition_phi += self.save_alpha[i]
				addition_distance += self.save_distance[i]
			

		# rospy.loginfo("phi %s" %(addition_phi))	
		# rospy.loginfo("d %s" %(addition_d))	
		# rospy.loginfo("dist %s" %(addition_distance))	


		return(addition_phi, float(addition_d), addition_distance)



	def predict(self, msg_camera_pose):

		# drive straight
		# self.msg_fusion.header.stamp = rospy.get_rostime()
		# self.msg_fusion.d = self.z_m[0]
		# self.msg_fusion.phi = self.z_m[1]
		# self.msg_fusion.in_lane = msg_camera_pose.in_lane
		# self.msg_fusion.status = msg_camera_pose.status
		# rospy.loginfo("%s, %s" %(self.z_m[0], self.z_m[1]))
		# self.pub_pose_est.publish(self.msg_fusion)
		# return
		
		#only camera pose
		# self.msg_fusion.header.stamp = rospy.get_rostime()
		# #rospy.loginfo("%s %s" %(msg_camera_pose.d, msg_camera_pose.phi))
		# self.msg_fusion.d = msg_camera_pose.d
		# self.msg_fusion.phi = msg_camera_pose.phi
		# self.pub_pose_est.publish(self.msg_fusion)
		# rospy.loginfo("e %s" %(self.z_m[0]))
		# return


		if self.i_first_calibration  < 50:
			self.first_calibration(msg_camera_pose.phi)
			return

		self.sum_alpha = 0

		self.z_m[1] = msg_camera_pose.d
		self.z_m[2] = msg_camera_pose.phi


		time_now = rospy.get_rostime().secs + rospy.get_rostime().nsecs / 1e9

		time_image = msg_camera_pose.header.stamp.secs + msg_camera_pose.header.stamp.nsecs / 1e9 #nu
		delta_t = time_image - self.time_last_est

		addition_phi, addition_d, addition_distance = self.encoder_information_until_now(time_image, self.x[1])
		#addition_phi, addition_d, addition_distance = 0,0,0


		if self.curve_detection(addition_phi, addition_d) == 1:
			return

		self.z_m[0] -= addition_phi

		## predict part
		
		# linearized:
		omega = (self.z_m[0] - self.old_z_m0)/delta_t
		self.old_z_m0 = np.copy(self.z_m[0])
		delta_d = self.distance - addition_distance - self.old_distance
		self.old_distance = self.distance - addition_distance
		

		# review: choose Q
		#self.Q = self.q * np.array([[(delta_t**3)/3, (delta_t**2)/2],[(delta_t**2)/2, delta_t]])

		#EKF
		# A = np.array([[1, delta_d * np.cos(self.x[1])],[0, 1]])
		# B = np.array([[0.5 * delta_d * delta_t * np.cos(self.x[1])], [delta_t]])
		# self.x = np.array([[self.x[0] + delta_d * np.sin(self.x[1])],[self.x[1]]]) + np.dot(B, omega[0]) ########

		#KF
		A = np.array([[1, delta_d],[0, 1]]) # self.v* delta_t = delta_d
		B = np.array([[0.5 * delta_d * delta_t], [delta_t]]) # self.v* delta_t**2 = delta_d * delta_t
		self.x = np.dot(A, self.x) + np.dot(B, omega[0])
		self.P = np.dot(np.dot(A, self.P), A.T) + self.Q

		self.time_last_est = time_image


		# update part
		S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
		self.K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))

		y = self.z_m - np.dot(self.H, self.x)
		self.x += np.dot(self.K, y)
		J = self.I - np.dot(self.K, self.H)
		self.P = np.dot(J, self.P)


		self.msg_fusion.header.stamp = rospy.get_rostime()
		self.msg_fusion.d = (self.x[0] + addition_d) * 1.5 ##!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		self.msg_fusion.phi = self.x[1] + addition_phi
		self.msg_fusion.in_lane = msg_camera_pose.in_lane
		self.pub_pose_est.publish(self.msg_fusion)


		
		self.z_m[0] += addition_phi
		
		self.recalibration()

		rospy.loginfo("u %s %s c %s %s e %s %s d [%s]" %(self.msg_fusion.d, self.msg_fusion.phi, self.z_m[1], self.z_m[2], self.z_m[0], self.b, self.distance))

		return



if __name__ == '__main__':
	node = Sensor_Fusion(node_name='my_sensor_fusion_node')
	rospy.loginfo("sensor_fusion_node is up and running...")
	rospy.spin()
