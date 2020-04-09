#!/usr/bin/env python
import os
import rospy
from duckietown import DTROS
import numpy as np
from duckietown_msgs.msg import LanePose, Twist2DStamped, EncoderTicksStamped, FusionLanePose
from std_msgs.msg import Header



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
		self.B = np.zeros((2,1))
		self.x = np.zeros((2,1))
		self.P = np.eye(2) #
		self.Q = 0.2 * np.eye(2) #
		self.R = 0.2 * np.eye(4) #
		self.I = np.eye(2)
		self.H = np.array([[1, 0], [0, 1], [1, 0], [0, 1]])
		self.v = 0
		self.omega = 0
		self.first_encoder_meas = 0

		self.encoder_state = 0
		self.last_msg_camera_time = 0
		self.last_camera_d = 0
		self.last_camera_phi = 0


		# Publisher
		self.pub_pose_est = rospy.Publisher("~fusion_lane_pose", FusionLanePose, queue_size=1) # for lane_controller_node

		# Subscriber
		self.sub_encoder_ticks =rospy.Subscriber("encoder_ticks_node/encoder_ticks", EncoderTicksStamped, self.update_encoder_measurement, queue_size=1)
		self.sub_camera_pose =rospy.Subscriber("lane_filter_node/lane_pose", LanePose, self.predict, queue_size=1)
		self.sub_controls = rospy.Subscriber("lane_controller_node/car_cmd", Twist2DStamped, self.save_inputs, queue_size=1)


	def save_inputs(self, msg_Twist2DStamped):
		self.v = msg_Twist2DStamped.v
		self.omega = msg_Twist2DStamped.omega


	def update_encoder_measurement(self, msg_encoder_ticks):
		if self.first_encoder_meas == 0:
			self.last_left_ticks = msg_encoder_ticks.left_ticks
			self.last_right_ticks = msg_encoder_ticks.right_ticks
			self.first_encoder_meas = 1
			return
		#gos = rospy.get_rostime().secs
		#gon = rospy.get_rostime().nsecs
		
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
			#rospy.loginfo("a")

		elif self.diff_left == 0 or self.diff_right == 0:
			self.z_m[0] += self.tick_to_meter * (self.diff_left + self.diff_right) / alpha * (np.cos(self.z_m[1] - alpha ) - np.cos(self.z_m[1]))
			#rospy.loginfo("b")

		else:
			#self.z_m[0] -= self.tick_to_meter * 0.5 * self.wheelbase * (self.diff_left + self.diff_right) / (self.diff_right - self.diff_left) * (np.cos(self.z_m[1]) - np.cos(self.old_z_m1))
			self.z_m[0] += self.tick_to_meter * 0.5 * (self.diff_left + self.diff_right) / alpha * (np.cos(self.old_z_m1) - np.cos(self.z_m[1]))
			#rospy.loginfo("d1: %s" %self.z_m[0])
			#rospy.loginfo("c")
		

		self.old_z_m1 = np.copy(self.z_m[1])

		#stop = rospy.get_rostime()
		self.encoder_state = rospy.get_rostime().secs + rospy.get_rostime().nsecs /1e9
		#rospy.loginfo("time encoder update_ %s" % (gos - stop.secs + (gon - stop.nsecs) / 1e9 ))

		rospy.loginfo("encoder d_e:%s phi_e:%s" %(self.z_m[0], self.z_m[1]))




	def predict(self, msg_camera_pose):
		#if msg_camera_pose.d == self.last_camera_d and msg_camera_pose.phi == self.last_camera_phi:
		#	return

		camera_state = msg_camera_pose.header.stamp.secs + msg_camera_pose.header.stamp.nsecs / 1e9
		rospy.loginfo("d_c %s  phi_c %s" %(msg_camera_pose.d, msg_camera_pose.phi))
		rospy.loginfo("delta_d %s  delta_phi %s" %(msg_camera_pose.d - self.z_m[0], msg_camera_pose.phi - self.z_m[1]))
		rospy.loginfo("stamp: %s\n" %(camera_state))
		self.last_msg_camera_time
		#self.last_camera_d = msg_camera_pose.d
		#self.last_camera_phi = msg_camera_pose.phi
		return

		
		# time_image = msg_camera_pose.header.stamp.secs + msg_camera_pose.header.stamp.nsecs / 1e9
		# if time_image == self.time_last_image:
		# 	return
		# delta_t = time_image - self.time_last_image
		# self.A = np.array([[1, self.v * delta_t],[0, 1]])
		# self.B = np.array([[0.5 * self.v * delta_t**2], [delta_t]])
		# self.x = np.dot(self.A, self.x) + np.dot(self.B, self.omega)
		# self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

		# self.z_m[2] = msg_camera_pose.d ##
		# self.z_m[3] = msg_camera_pose.phi ##
		# self.time_last_image = time_image

		# self.update()


	def update(self):
		return
		# S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
		# self.K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
		# y = self.z_m - np.dot(self.H, self.x)
		# self.x += np.dot(self.K, y)
		# J = self.I - np.dot(self.K, self.H)
		# self.P = np.dot(J, self.P)
		# self.msg_fusion.header.stamp = rospy.get_rostime()
		# self.msg_fusion.d = self.x[0]
		# self.msg_fusion.phi = self.x[1]
		# self.pub_pose_est.publish(self.msg_fusion)
		# rospy.loginfo("update d:%s phi:%s" %(self.msg_fusion.d, self.msg_fusion.phi))




if __name__ == '__main__':
	node = Sensor_Fusion(node_name='my_sensor_fusion_node')
	#node.run()
	rospy.loginfo("sensor_fusion_node is up and running...")
	rospy.spin()
