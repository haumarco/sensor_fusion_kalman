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
		self.wheelbase = 0.102
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
		self.block_turn = -1.05
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
		self.count_curves_axle = 0
		self.phi_tot = 0
		self.phi_0 = 0
		self.distance = 0
		self.cs_transform = 0
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

		alpha = self.tick_to_meter * (self.diff_right - self.diff_left) / self.wheelbase
		# ueli 1: 1.04, ueli 2: 1.01
		self.phi_tot += alpha * 1.02

		# if self.diff_left == 0 or self.diff_right == 0:
		# 	self.phi_0 += alpha
		# 	alpha = alpha /1.05 #1.04
		

		self.z_m[1] += alpha

		self.distance += self.tick_to_meter * 0.5 * (self.diff_left + self.diff_right)

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


	def curve_detection(self, time_now):
		if self.distance - self.block_turn > 0.3 : ##2.5,2.8

			if self.recalib_status == 1 or ( self.recalib_status == 2 and self.distance - self.block_turn > 0.8 ):
				self.i_recalib = 0
				self.recalib_status = 0

			if self.z_m[3] > 0.85 and self.z_m[1] < 0.4: # right turn
				self.ensure_turn -= 1
				if self.ensure_turn < -2:
					rospy.loginfo("-------------------- RIGHT TURN --------------------")
					self.block_turn = self.distance
					self.ensure_turn = 0
					self.right_turn = 1
					self.recalib_status = 1
					self.count_curves_axle += 0.5 * np.pi

			elif self.z_m[3] < -0.55 and self.z_m[1] > -0.2 and self.z_m[1] < 0.5 and abs(self.z_m[2]) < 0.11 and self.distance - self.block_turn > 1.1: # left turn
				self.ensure_turn += 1
				#rospy.loginfo("ensure %s" %(self.z_m[1]))
				if self.ensure_turn > 2:
					rospy.loginfo("-------------------- LEFT TURN --------------------")
					self.block_turn = self.distance
					self.ensure_turn = 0
					self.left_turn = 1
					self.recalib_status = 2
					self.count_curves_axle -= 0.5 * np.pi

			else:
				self.ensure_turn = 0



		if self.right_turn > 0 and self.right_turn < 3:
			if self.right_turn == 1:
				self.z_m[1] += 0.1 * np.pi
				self.right_turn = 2
			self.z_m[1] -= self.cs_transform
			self.cs_transform = (self.distance - self.block_turn) / 0.25 * np.pi #adjust in 0.1m
			if self.cs_transform > 0.4 * np.pi:
				self.cs_transform = 0.4 * np.pi
				self.z_m[1] += self.cs_transform
				self.cs_transform = 0
				self.right_turn = 3
			else:
				self.z_m[1] += self.cs_transform

		if self.left_turn !=0:
			self.z_m[1] -= self.cs_transform
			self.cs_transform = -(self.distance - self.block_turn) / 0.8 * np.pi #adjust in 0.4m
			if self.cs_transform < -0.5 * np.pi:
				self.cs_transform = -0.5 * np.pi
				self.z_m[1] += self.cs_transform
				self.cs_transform = 0
				self.left_turn = 0
			else:
				self.z_m[1] += self.cs_transform
		
		




	def first_calibration(self, phi_cam):
		#self.z_m[0] += 0.015 * msg_camera_pose.d
		self.z_m[1] += 0.015 * phi_cam
		self.i_first_calibration += 1

		if self.i_first_calibration == 50:
			self.time_last_est = rospy.get_rostime().secs + rospy.get_rostime().nsecs / 1e9
			self.time_last_image = self.time_last_est
			#rospy.loginfo("first_calibration done")



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
					#rospy.logwarn("RECALIBRATION %s, %s" %(self.average_phi_cam, self.average_d_cam))
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

		#only camera pose
		# self.msg_fusion.header.stamp = rospy.get_rostime()
		# #rospy.loginfo("%s %s" %(msg_camera_pose.d, msg_camera_pose.phi))
		# self.msg_fusion.d = msg_camera_pose.d
		# self.msg_fusion.phi = msg_camera_pose.phi
		# self.pub_pose_est.publish(self.msg_fusion)
		# rospy.loginfo("c [%s] [%s] e %s" %(msg_camera_pose.d, msg_camera_pose.phi, self.z_m[1]))

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

		self.z_m[2] = msg_camera_pose.d
		self.z_m[3] = msg_camera_pose.phi

		self.recalibration()

		time_now = rospy.get_rostime().secs + rospy.get_rostime().nsecs / 1e9

		self.curve_detection(time_now)


		## predict part
		
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


		self.time_last_est = time_now

		self.update()
		return


	def update(self):

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
		#rospy.loginfo("u %s %s c %s %s e %s %s" %(self.msg_fusion.d, self.msg_fusion.phi, self.z_m[2], self.z_m[3], self.z_m[0], self.z_m[1]))
		rospy.loginfo("u %s %s c %s %s e %s %s d %s" %(self.msg_fusion.d, self.msg_fusion.phi, self.z_m[2], self.z_m[3], self.z_m[0], self.z_m[1], self.distance))

		# rospy.loginfo(",K,%s,%s,%s,%s;%s,%s,%s,%s" %(self.K[0][0],self.K[0][1],self.K[0][2],self.K[0][3],self.K[1][0],self.K[1][1],self.K[1][2],self.K[1][3]))
		# rospy.loginfo(",P,%s,%s;%s,%s" %(self.P[0][0],self.P[0][1],self.P[1][0],self.P[1][1]))
		# rospy.loginfo(",A,%s,%s;%s,%s" %(self.A[0][0],self.A[0][1],self.A[1][0],self.A[1][1]))
		# rospy.loginfo(",B,%s;%s" %(self.B[0],self.B[1]))


		if self.count_curves_axle != 0:
			axle = 0.1 * abs(self.phi_tot / self.count_curves_axle )
			corrfactor = abs(self.phi_0) / (abs(self.count_curves_axle) - abs(self.phi_tot) + abs(self.phi_0))
			rospy.loginfo("phi: %s  axle: %s  corr: %s  phi0: %s" %(self.phi_tot, axle, corrfactor, self.phi_0))

		return
		
if __name__ == '__main__':
	node = Sensor_Fusion(node_name='my_sensor_fusion_node')
	rospy.loginfo("sensor_fusion_node is up and running...")
	rospy.spin()