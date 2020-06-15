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
		self.wheelbase = 0.1
		self.msg_fusion = FusionLanePose()

		self.A = np.array([])
		self.time_last_image = 0
		self.time_last_est = 0
		self.B = np.zeros((2,1))
		self.x = np.zeros((2,1))
		self.P = np.eye(2) #
		self.K = np.zeros((2,4))
		self.Q = 0.8 * np.eye(2) #
		self.R = 0.2 * np.eye(4) #
		self.R[0][0] = 0.4
		self.R[1][1] = 0.2
		self.R[2][2] = 0.2
		self.R[3][3] = 0.4
		self.I = np.eye(2)
		self.H = np.array([[1, 0], [0, 1], [1, 0], [0, 1]])
		self.v = 0
		self.omega = 0
		self.first_encoder_meas = 0

		self.last_msg_camera_time = 0
		self.last_camera_d = 0
		self.last_camera_phi = 0

		self.store_enc_meas = np.zeros((3,1))
		self.ensure_turn = 0
		self.block_turn = 0
		self.right_turn = 0
		self.left_turn = 0
		self.i_first_calibration = 0 # number of iterat. for first calib
		self.take_d_from_cam = 0

		##recalibration:
		self.save_phi_enc = np.zeros(10)
		self.save_d_cam = np.zeros(10)
		self.save_phi_cam = np.zeros(10)
		self.average_d_cam = 0
		self.average_phi_cam = 0
		self.i_recalib = 0
		self.recalib_status = 0
		##

		self.a = 0
		self.b = 0

		# Publisher
		self.pub_pose_est = rospy.Publisher("~fusion_lane_pose", FusionLanePose, queue_size=1) # for lane_controller_node

		# Subscriber
		self.sub_encoder_ticks =rospy.Subscriber("encoder_ticks_node/encoder_ticks", EncoderTicksStamped, self.update_encoder_measurement, queue_size=1)
		self.sub_camera_pose =rospy.Subscriber("lane_filter_node/lane_pose", LanePose, self.predict, queue_size=1)
		#self.sub_controls = rospy.Subscriber("lane_controller_node/car_cmd", Twist2DStamped, self.save_inputs, queue_size=1)
		self.sub_controls = rospy.Subscriber("lane_controller_node/car_cmd", Twist2DStamped, self.save_inputs, queue_size=1)	


	def save_inputs(self, msg_Twist2DStamped):
		self.v = msg_Twist2DStamped.v
		self.omega = msg_Twist2DStamped.omega
		return


	def update_encoder_measurement(self, msg_encoder_ticks):
		if self.first_encoder_meas == 0:
			self.last_left_ticks = msg_encoder_ticks.left_ticks
			self.last_right_ticks = msg_encoder_ticks.right_ticks
			self.first_encoder_meas = 1
			return
		
		self.diff_left = msg_encoder_ticks.left_ticks - self.last_left_ticks
		self.diff_right = msg_encoder_ticks.right_ticks - self.last_right_ticks
		self.last_left_ticks = msg_encoder_ticks.left_ticks
		self.last_right_ticks = msg_encoder_ticks.right_ticks
		# self.diff_left = msg_encoder_ticks.left_ticks
		# self.diff_right = msg_encoder_ticks.right_ticks


		alpha = self.tick_to_meter * (self.diff_right - self.diff_left) / self.wheelbase /1.04

		if self.diff_left == 0 or self.diff_right == 0:
			alpha = alpha /1.04
			if self.diff_left > 6 or self.diff_right > 6:
				alpha = alpha * 0.85

		self.z_m[1] += alpha

		# if alpha == 0:
		# 	self.z_m[0] += self.diff_left * self.tick_to_meter * np.sin(self.z_m[1]) 

		# elif self.diff_left == 0 or self.diff_right == 0:
		# 	self.z_m[0] += self.tick_to_meter * (self.diff_left + self.diff_right) / alpha * (np.cos(self.z_m[1] - alpha ) - np.cos(self.z_m[1]))

		# else:
		# 	self.z_m[0] += self.tick_to_meter * 0.5 * (self.diff_left + self.diff_right) / alpha * (np.cos(self.old_z_m1) - np.cos(self.z_m[1]))
		

		# self.old_z_m1 = np.copy(self.z_m[1])

		#encoder_time = rospy.get_rostime().secs + rospy.get_rostime().nsecs /1e9
		#rospy.loginfo("[%s] [%s] [%s] %s" %(self.diff_left, self.diff_right, alpha, self.z_m[1]))
		#rospy.loginfo("%s %s" %(msg_encoder_ticks.left_ticks, msg_encoder_ticks.right_ticks))

		#e_states = np.vstack((self.z_m[0], self.z_m[1], encoder_time))
		#self.store_enc_meas = np.hstack((self.store_enc_meas, e_states))
		return



	def first_calibration(self, phi_cam):
		#self.z_m[0] += 0.015 * msg_camera_pose.d
		self.z_m[1] += 0.015 * phi_cam
		self.i_first_calibration += 1

		if self.i_first_calibration == 50:
			self.time_last_est = rospy.get_rostime().secs + rospy.get_rostime().nsecs / 1e9
			self.time_last_image = self.time_last_est
			rospy.loginfo("first_calibration done")



	def recalibration(self):
		if self.recalib_status == 0:

			self.save_d_cam = np.append(self.save_d_cam, self.z_m[2])
			self.save_d_cam = np.delete(self.save_d_cam, 0)
			self.save_phi_cam = np.append(self.save_phi_cam, self.z_m[3])
			self.save_phi_cam = np.delete(self.save_phi_cam, 0)

			# self.average_d_cam += (self.save_d_cam[9] - self.save_d_cam[0]) / 10
			# self.average_phi_cam += (self.save_phi_cam[9] - self.save_phi_cam[0]) / 10
			self.average_d_cam = np.mean(self.save_d_cam[:7])
			self.average_phi_cam = np.mean(self.save_phi_cam[:7])


			if self.i_recalib == 10:
				self.save_phi_enc = np.append(self.save_phi_enc, self.z_m[1])
				self.save_phi_enc = np.delete(self.save_phi_enc, 0)
				diff_recalib = abs(self.save_phi_enc[9] - self.save_phi_enc[0])

				#rospy.loginfo("%s %s %s" %(self.average_d_cam, self.average_phi_cam, diff_recalib))
				if diff_recalib < 0.05 and abs(self.average_d_cam) < 0.1 and abs(self.average_phi_cam) < 0.1:
					self.z_m[1] = 0.3 * self.z_m[1] + 0.3 * self.average_phi_cam
					#self.z_m[1] *= 0.5
					rospy.logwarn("RECALIBRATION")



			else:
				self.save_phi_enc[self.i_recalib] = self.z_m[1]
				self.i_recalib += 1




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

		# only camera pose
		# self.msg_fusion.header.stamp = rospy.get_rostime()
		# #rospy.loginfo("%s %s" %(msg_camera_pose.d, msg_camera_pose.phi))
		# self.msg_fusion.d = msg_camera_pose.d
		# self.msg_fusion.phi = msg_camera_pose.phi
		# self.pub_pose_est.publish(self.msg_fusion)
		# rospy.loginfo("e %s  %s" %(self.z_m[0], self.z_m[1]))
		# return

		###
		#camera_time = msg_camera_pose.header.stamp.secs + msg_camera_pose.header.stamp.nsecs / 1e9
		# while self.store_enc_meas[2][0] < camera_time and self.store_enc_meas.shape[1] > 1:
		# 	self.store_enc_meas = np.delete(self.store_enc_meas, 0, 1)
		# #rospy.loginfo("c-e_d %s  c-e_phi %s" %(msg_camera_pose.d - self.store_enc_meas[0][0], msg_camera_pose.phi - self.store_enc_meas[1][0]))
		# ##rospy.loginfo("c_d %s  c_phi %s" %(msg_camera_pose.d, msg_camera_pose.phi ))

		if self.i_first_calibration  < 50:
			self.first_calibration(msg_camera_pose.phi)
			return


		self.recalibration()

		time_now = rospy.get_rostime().secs + rospy.get_rostime().nsecs / 1e9
		if time_now - self.block_turn > 2.5 :

			if self.recalib_status == 1 or ( self.recalib_status == 2 and time_now - self.block_turn > 3.5 ):
				self.i_recalib = 0
				self.recalib_status = 0

			if msg_camera_pose.phi > 0.85 and self.z_m[1] < 0.4: # right turn
				self.ensure_turn += 1
				if self.ensure_turn > 2:
					rospy.loginfo("-------------------- RIGHT TURN --------------------")
					self.block_turn = time_now
					self.ensure_turn = 0
					self.right_turn = 1
					self.recalib_status = 1

			elif msg_camera_pose.phi < -0.55 and self.z_m[1] > -0.2 and self.z_m[1] < 0.5 and abs(self.z_m[2]) < 0.11 and time_now - self.block_turn > 3.5: # left turn
				self.ensure_turn += 1
				if self.ensure_turn > 2:
					rospy.loginfo("-------------------- LEFT TURN --------------------")
					self.block_turn = time_now
					self.ensure_turn = 0
					self.left_turn = 1
					self.recalib_status = 2

			else:
				self.ensure_turn = 0

				

		# elif time_now - self.block_turn < 2.5 and self.right_turn > 0:
		# 	self.take_d_from_cam = 1

		# elif time_now - self.block_turn < 3 and self.left_turn > 0:
		# 	self.take_d_from_cam = 1

		# test: time to adjust angle
		# if (self.right_turn > 4 or self.left_turn >4) and self.a == 0:
		# 	if self.z_m[1] < 0.4:
		# 		rospy.loginfo("%s" %(time_now - self.block_turn))
		# 		self.a = 1


		if self.right_turn > 0 and self.right_turn < 10:
			if self.right_turn == 1:
				self.z_m[1] += 0.1 * np.pi
			else:
				self.z_m[1] += 0.05 * np.pi   ###### eigentlich 0.05 (((- 0.065
			if self.right_turn == 5:
				rospy.loginfo("end adjusting")
			self.right_turn += 1


		if self.left_turn > 0 and self.left_turn < 25: 
			if self.left_turn == 1:
				self.z_m[1] -= 0.04 * np.pi ##0.2, 14, 0.025; 0.1, 18, 0.025;; 0.05 20 0.025;
			else:
				self.z_m[1] -= 0.02 * np.pi
			if self.left_turn == 24:
				rospy.loginfo("end adjusting")
			self.left_turn += 1


		


		## predict part


		
		#time_now = rospy.get_rostime().secs + rospy.get_rostime().nsecs / 1e9
		delta_t = time_now - self.time_last_est
		time_image = msg_camera_pose.header.stamp.secs + msg_camera_pose.header.stamp.nsecs / 1e9 #nu

		# if self.x[1] == 0:
		# 	self.A = np.array([[1, self.v * delta_t * np.sin(self.omega*delta_t)],[0, 1]])
		# elif self.omega == 0:
		# 	self.A = np.array([[1, delta_t * self.v * np.sin(self.x[1]) / self.x[1]],[0, 1]])
		# else:
		# 	self.A = np.array([[1, self.v /self.omega *(np.cos(self.x[1]) - np.cos(self.x[1]+ self.omega * delta_t))],[0, 1]])
		# self.B = np.array([[0], [delta_t]])
		
		# linearized:
		self.A = np.array([[1, self.v * delta_t],[0, 1]])
		self.B = np.array([[0.5 * self.v * delta_t**2], [delta_t]])
		self.x = np.dot(self.A, self.x) + np.dot(self.B, self.omega)
		self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

		x_cam = np.array([[msg_camera_pose.d],[msg_camera_pose.phi]])
		# x_cam = np.dot(self.A, x_cam) + np.dot(self.B, np.average(self.save_omega))
		self.z_m[2] = x_cam[0][0] #unnecessary
		self.z_m[3] = x_cam[1][0]

		self.time_last_est = time_now

		self.update()
		return


	def update(self):
		#return
		# update part
		S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
		self.K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
		self.take_d_from_cam = 1 ############ !!
		if self.take_d_from_cam == 1:
			#rospy.loginfo("take d form cam")
			self.take_d_from_cam = 0
			self.z_m[0] = self.z_m[2]
			# self.z_m[3] = self.z_m[1] ########
		#copy_x = np.copy(self.x)
		y = self.z_m - np.dot(self.H, self.x)
		self.x += np.dot(self.K, y)
		J = self.I - np.dot(self.K, self.H)
		self.P = np.dot(J, self.P)
		self.msg_fusion.header.stamp = rospy.get_rostime()
		self.msg_fusion.d = self.x[0]
		self.msg_fusion.phi = self.x[1]
		self.pub_pose_est.publish(self.msg_fusion)

		# rospy.loginfo("update: %s  %s" %(self.msg_fusion.d, self.msg_fusion.phi))
		# rospy.loginfo("camera: %s  %s" %(self.z_m[2], self.z_m[3]))
		# rospy.loginfo("e %s  %s" %(self.z_m[0], self.z_m[1]))
		rospy.loginfo("u %s %s c %s %s e %s %s" %(self.msg_fusion.d, self.msg_fusion.phi, self.z_m[2], self.z_m[3], self.z_m[0], self.z_m[1]))

		#rospy.loginfo("\nK\n%s\nP\n%s\n" %(self.K, self.P))
		return



if __name__ == '__main__':
	node = Sensor_Fusion(node_name='my_sensor_fusion_node')
	rospy.loginfo("sensor_fusion_node is up and running...")
	rospy.spin()
