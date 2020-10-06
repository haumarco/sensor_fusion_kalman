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

		self.diff_left = 0
		self.diff_right = 0
		self.last_left_ticks = 0
		self.last_right_ticks = 0
		self.tick_to_meter = np.pi * 0.067 / 135.   # 6.5cm diameter, 135 = #ticks per revolution
		self.z_m = np.zeros((3, 1)) # d_enc, phi_enc, d_cam, phi_cam
		self.last_zm0 = 0
		self.old_z_m0 = 0
		self.wheelbase = 0.103 #0.102 # ueli 1: 0.104, ueli 2: 0.101----1.03
		self.corretion_factor = 1.05 # ueli 1: 1.05, ueli 2: 1.03---- 1
		self.msg_fusion = FusionLanePose()

		self.A = np.array([])
		self.time_last_image = 0
		self.time_last_est = 0
		self.B = np.zeros((2,1))
		self.x = np.zeros((2,1))
		self.P = np.eye(2) #
		self.K = np.zeros((2,3))
		self.Q = 0.5 * np.eye(2) #
		self.q = 5
		self.R = 0.2 * np.eye(3) #
		self.R[0][0] = 0.0025 #0.1   0.0025
		self.R[1][1] = 0.04 #0.05    0.04
		self.R[2][2] = 0.01 #0.2   0.0025
		self.I = np.eye(2)
		self.H = np.array([[0, 1], [1, 0], [0, 1]])
		self.v = 0
		self.omega = 0
		self.first_encoder_meas_l = 0
		self.first_encoder_meas_r = 0

		self.last_msg_camera_time = 0
		self.last_camera_d = 0
		self.last_camera_phi = 0

		self.store_enc_meas = np.zeros((3,1))
		self.ensure_turn = 0
		self.length_left_curve = 0.65
		self.length_right_curve = 0.28
		self.block_turn_l = -self.length_left_curve -self.length_right_curve + 0.05
		self.block_turn_r = -self.length_right_curve - 0.25
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
		self.count = 0
		self.addition_phi = 0
		self.addition_distance = 0
		self.addition_d = 0
		##
		self.distance = 0
		self.old_distance = 0
		self.cs_transform = 0
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
		self.left_dist_vel = 0
		self.right_dist_vel = 0
		self.direction_l = 1
		self.direction_r = 1

		self.ignore_only_cam = 0
		self.turn = 0
		self.recalib_distance = 0

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
			return
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
			return

	def update_encoder_measurement_r(self, msg_encoder_ticks):
		if self.first_encoder_meas_r == 0:
			self.last_right_ticks = msg_encoder_ticks.data
			self.first_encoder_meas_r = 1
			return
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
			return

	def save_encoder_measurements(self, diff_dist, msg_encoder_ticks, alpha):



		# if self.first_encoder_meas == 0:
		# 	self.last_left_ticks = msg_encoder_ticks.left_ticks
		# 	self.last_right_ticks = msg_encoder_ticks.right_ticks
		# 	self.first_encoder_meas = 1
		# 	return
		
		# self.diff_left = msg_encoder_ticks.left_ticks - self.last_left_ticks
		# self.diff_right = msg_encoder_ticks.right_ticks - self.last_right_ticks
		# self.last_left_ticks = msg_encoder_ticks.left_ticks
		# self.last_right_ticks = msg_encoder_ticks.right_ticks
		# # self.diff_left = msg_encoder_ticks.left_ticks
		# # self.diff_right = msg_encoder_ticks.right_ticks

		# alpha = self.tick_to_meter * (self.diff_right - self.diff_left) / self.wheelbase


		# self.z_m[0] += alpha

		# diff_dist = self.tick_to_meter * 0.5 * (self.diff_left + self.diff_right)
		self.distance += diff_dist

		self.save_alpha = np.append(self.save_alpha, alpha)
		self.save_alpha = np.delete(self.save_alpha, 0)
		self.save_timestamp = np.append(self.save_timestamp, msg_encoder_ticks.header.stamp.secs + msg_encoder_ticks.header.stamp.nsecs/1e9)
		self.save_timestamp = np.delete(self.save_timestamp, 0)
		self.save_distance = np.append(self.save_distance, diff_dist)
		self.save_distance = np.delete(self.save_distance, 0)

		if alpha == 0 and self.recalib_status == 0:
			# sinx = x -1/6 x**3
			diff_d = diff_dist * (self.z_m[0] - self.z_m[0]**3 /6)
			self.save_delta_d = np.append(self.save_delta_d, diff_d[0])
			self.save_delta_d = np.delete(self.save_delta_d, 0)


		elif self.recalib_status == 0:
			# cosx = 1 - 1/2 x**2 -> cosa -cosb = 1/2 (-a**2 + b**2)
			diff_d = diff_dist / alpha * (- self.last_zm0**2 + self.z_m[0])/2
			self.save_delta_d = np.append(self.save_delta_d, diff_d[0])
			self.save_delta_d = np.delete(self.save_delta_d, 0)
		else:
			self.save_delta_d = np.append(self.save_delta_d, 0)
			self.save_delta_d = np.delete(self.save_delta_d, 0)

		

		self.last_zm0 = np.copy(self.z_m[0])

		#encoder_time = rospy.get_rostime().secs + rospy.get_rostime().nsecs /1e9
		#rospy.loginfo("[%s] [%s] [%s] %s" %(self.diff_left, self.diff_right, alpha, self.z_m[1]))
		#rospy.loginfo("%s %s" %(msg_encoder_ticks.left_ticks, msg_encoder_ticks.right_ticks))

		#e_states = np.vstack((self.z_m[0], self.z_m[1], encoder_time))
		#self.store_enc_meas = np.hstack((self.store_enc_meas, e_states))
		return


	def curve_detection(self):

		if abs(self.z_m[0]) > 1:
			rospy.loginfo("turn %s" %(self.z_m[0]))
			self.z_m[0] -= np.sign(self.z_m[0]) * np.pi /2
			self.ignore_only_cam = 1
			self.turn += 1


		if abs(self.z_m[0] - self.z_m[2]) > 0.6 and self.ignore_only_cam == 0:
			rospy.loginfo("only cam c: %s  e: %s %s" %(self.z_m[2], self.z_m[0], self.b))
			self.msg_fusion.header.stamp = rospy.get_rostime()
			#rospy.loginfo("%s %s" %(msg_camera_pose.d, msg_camera_pose.phi))
			self.msg_fusion.d = self.z_m[1]
			self.msg_fusion.phi = self.z_m[2]
			self.pub_pose_est.publish(self.msg_fusion)

			self.R

			return(1)
		else:
			self.ignore_only_cam = 0

		return(0)





	def wheel_encoder_coordinate_system_rotation(self):


		if self.right_turn == 1:
			self.z_m[0] -= self.cs_transform
			self.cs_transform = (self.distance - self.block_turn_r) / self.length_right_curve * np.pi #adjust in 0.1m 0.2
			if self.cs_transform > 0.5 * np.pi:
				self.cs_transform = 0.5 * np.pi
				self.z_m[0] += self.cs_transform
				self.cs_transform = 0
				self.right_turn = 0
			else:
				self.z_m[0] += self.cs_transform
				

		if self.left_turn == 1 or self.left_turn == 2:
			self.z_m[0] -= self.cs_transform
			self.cs_transform = -(self.distance - self.block_turn_l) / self.length_left_curve * np.pi #adjust in 0.4m 0.8
			if self.cs_transform < -0.5 * np.pi:
				self.cs_transform = -0.5 * np.pi
				self.z_m[0] += self.cs_transform
				self.cs_transform = 0
				self.left_turn = 0
			else:
				self.z_m[0] += self.cs_transform
	


	def first_calibration(self, phi_cam):
		#self.z_m[0] += 0.015 * msg_camera_pose.d
		self.z_m[0] += 0.015 * phi_cam 
		self.i_first_calibration += 1

		if self.i_first_calibration == 50:
			self.time_last_est = rospy.get_rostime().secs + rospy.get_rostime().nsecs / 1e9
			self.time_last_image = self.time_last_est
			#rospy.loginfo("first_calibration done")



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





	def camera_delay(self, image):
		lin_interpl = 0
		for i in range(0,20):
			if self.save_timestamp[i] > image:
				## if read encoder frequency is low
				if lin_interpl == 0:
					lin_interpl = (image - self.save_timestamp[i-1]) / (self.save_timestamp[i] - self.save_timestamp[i-1])
					self.addition_phi += lin_interpl * (self.save_alpha[i] - self.save_alpha[i-1]) + self.save_alpha[i-1]
					self.addition_distance += lin_interpl * (self.save_distance[i] - self.save_distance[i-1]) + self.save_distance[i-1]
					self.addition_d += lin_interpl * (self.save_delta_d[i] - self.save_delta_d[i-1]) + self.save_delta_d[i-1]
					#rospy.loginfo("%s" %lin_interpl)
				##
				self.addition_phi += self.save_alpha[i]
				self.addition_distance += self.save_distance[i]
				self.addition_d += self.save_delta_d[i]

		self.z_m[0] -= self.addition_phi


	def vel_enc(self, time_now): # !!!
		dt = time_now - self.old_time_now
		if dt < 1:
			self.vel_left = self.left_dist_vel / dt
			self.vel_right = self.right_dist_vel / dt
			self.old_time_now = time_now
			self.left_dist_vel = 0
			self.right_dist_vel = 0
			self.v = (self.vel_left + self.vel_right) / 2
			self.omega = (self.v - self.vel_left) / self.wheelbase / 2
			rospy.loginfo("v %s  %s" %(self.v, self.omega))
		self.old_time_now = time_now
	


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


		###
		#camera_time = msg_camera_pose.header.stamp.secs + msg_camera_pose.header.stamp.nsecs / 1e9
		# while self.store_enc_meas[2][0] < camera_time and self.store_enc_meas.shape[1] > 1:
		# 	self.store_enc_meas = np.delete(self.store_enc_meas, 0, 1)
		# #rospy.loginfo("c-e_d %s  c-e_phi %s" %(msg_camera_pose.d - self.store_enc_meas[0][0], msg_camera_pose.phi - self.store_enc_meas[1][0]))
		# ##rospy.loginfo("c_d %s  c_phi %s" %(msg_camera_pose.d, msg_camera_pose.phi ))

		if self.i_first_calibration  < 50:
			self.first_calibration(msg_camera_pose.phi)
			return

		self.z_m[1] = msg_camera_pose.d
		self.z_m[2] = msg_camera_pose.phi

		time_now = rospy.get_rostime().secs + rospy.get_rostime().nsecs / 1e9
		#self.vel_enc(time_now) #vel_enc
		time_image = msg_camera_pose.header.stamp.secs + msg_camera_pose.header.stamp.nsecs / 1e9 #nu
		delta_t = time_image - self.time_last_est

		#self.camera_delay(time_image)

		if self.curve_detection() == 1:

			return

		## predict part
		

		# camera delay was here...

		# if self.x[1] == 0:
		# 	self.A = np.array([[1, self.v * delta_t * np.sin(self.omega*delta_t)],[0, 1]])
		# elif self.omega == 0:
		# 	self.A = np.array([[1, delta_t * self.v * np.sin(self.x[1]) / self.x[1]],[0, 1]])
		# else:
		# 	self.A = np.array([[1, self.v /self.omega *(np.cos(self.x[1]) - np.cos(self.x[1]+ self.omega * delta_t))],[0, 1]])
		# self.B = np.array([[0], [delta_t]])
		
		# linearized:
		omega = (self.z_m[0] - self.old_z_m0)/delta_t
		self.old_z_m0 = np.copy(self.z_m[0])
		delta_d = self.distance - self.addition_distance - self.old_distance
		self.old_distance = self.distance - self.addition_distance
		
		#rospy.loginfo("%s %s %s" %(self.old_z_m1, delta_d, omega[0]))
		self.Q = self.q * np.array([[(delta_t**3)/3, (delta_t**2)/2],[(delta_t**2)/2, delta_t]])

		#EKF
		# self.A = np.array([[1, delta_d * np.cos(self.x[1])],[0, 1]])
		# self.B = np.array([[0.5 * delta_d * delta_t * np.cos(self.x[1])], [delta_t]])
		# self.x = np.array([[self.x[0] + delta_d * np.sin(self.x[1])],[self.x[1]]]) + np.dot(self.B, omega[0]) ########

		#KF
		self.A = np.array([[1, delta_d],[0, 1]]) # self.v* delta_t = delta_d
		self.B = np.array([[0.5 * delta_d * delta_t], [delta_t]]) # self.v* delta_t**2 = delta_d * delta_t
		self.x = np.dot(self.A, self.x) + np.dot(self.B, omega[0])
		self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

		self.time_last_est = time_image

		self.update()
		return


	def update(self):

		# update part
		S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
		self.K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))

		y = self.z_m - np.dot(self.H, self.x)
		self.x += np.dot(self.K, y)
		J = self.I - np.dot(self.K, self.H)
		self.P = np.dot(J, self.P)

		self.x[1] += self.addition_phi
		self.msg_fusion.header.stamp = rospy.get_rostime()
		self.msg_fusion.d = self.x[0] * 1.5 ##!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		#self.msg_fusion.d = self.z_m[2]
		self.msg_fusion.phi = self.x[1]
		#self.msg_fusion.phi = self.z_m[1]

#		self.msg_fusion.curvature = self.vel_left #!!!; msgs spaeter hinzufuegen
#		self.msg_fusion.curvature_ref = self.vel_right

		self.pub_pose_est.publish(self.msg_fusion)

		
		self.z_m[0] += self.addition_phi
		self.z_m[1] += self.addition_d
		self.z_m[2] += self.addition_phi
		self.recalibration()
		#self.wheel_encoder_coordinate_system_rotation()
		self.addition_phi = 0
		self.addition_d = 0
		self.addition_distance = 0



		# rospy.loginfo("update: %s  %s" %(self.msg_fusion.d, self.msg_fusion.phi))
		# rospy.loginfo("camera: %s  %s" %(self.z_m[2], self.z_m[3]))
		# rospy.loginfo("e %s  %s" %(self.z_m[0], self.z_m[1]))
		# rospy.loginfo("u %s %s c %s %s e %s %s" %(self.msg_fusion.d, self.msg_fusion.phi, self.z_m[2], self.z_m[3], self.z_m[0], self.z_m[1]))
		rospy.loginfo("u %s %s c %s %s e %s %s d [%s]" %(self.msg_fusion.d, self.msg_fusion.phi, self.z_m[1], self.z_m[2], self.z_m[0], self.b, self.distance))
		# rospy.loginfo("c %s %s e %s d %s ticks0 %s slip %s" %(self.z_m[2], self.z_m[3], self.z_m[1], self.distance, self.a, self.b))


		# rospy.loginfo(",K,%s,%s,%s,%s;%s,%s,%s,%s" %(self.K[0][0],self.K[0][1],self.K[0][2],self.K[0][3],self.K[1][0],self.K[1][1],self.K[1][2],self.K[1][3]))
		# rospy.loginfo(",P,%s,%s;%s,%s" %(self.P[0][0],self.P[0][1],self.P[1][0],self.P[1][1]))
		# rospy.loginfo(",A,%s,%s;%s,%s" %(self.A[0][0],self.A[0][1],self.A[1][0],self.A[1][1]))
		# rospy.loginfo(",B,%s;%s" %(self.B[0],self.B[1]))

		#only camera pose
		# self.msg_fusion.header.stamp = rospy.get_rostime()
		# #rospy.loginfo("%s %s" %(msg_camera_pose.d, msg_camera_pose.phi))
		# self.msg_fusion.d = self.z_m[2]
		# self.msg_fusion.phi = self.z_m[3]
		# self.pub_pose_est.publish(self.msg_fusion)
		#rospy.loginfo("%s %s %s %s" %(self.z_m[2], self.z_m[3], self.z_m[1], self.distance)) # d_cam phi_cam phi_enc distance


		return



if __name__ == '__main__':
	node = Sensor_Fusion(node_name='my_sensor_fusion_node')
	rospy.loginfo("sensor_fusion_node is up and running...")
	rospy.spin()
