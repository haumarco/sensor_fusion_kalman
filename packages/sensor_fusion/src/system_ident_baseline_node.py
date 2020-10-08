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
		self.wheelbase = 0.1 #0.102 # ueli 1: 0.104, ueli 2: 0.101----1.03
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
		self.turn = 1
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
			return



	def curve_detection(self):

		if abs(self.z_m[0]) > 2.4:
			self.z_m[0] -= np.sign(self.z_m[0]) * np.pi/2
			self.turn += 1


		return



	


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
		self.msg_fusion.header.stamp = rospy.get_rostime()
		#rospy.loginfo("%s %s" %(msg_camera_pose.d, msg_camera_pose.phi))
		self.msg_fusion.d = msg_camera_pose.d
		self.msg_fusion.phi = msg_camera_pose.phi
		self.pub_pose_est.publish(self.msg_fusion)
		#rospy.loginfo("e %s" %(self.z_m[0]))
		# return


		self.curve_detection()

		if self.turn != 0:
			axle = 0.1 * self.b / (self.turn * np.pi/2)
			rospy.loginfo("turns: %s phi_tot: %s baseline: %s" %(self.turn, self.b, axle))

		return



if __name__ == '__main__':
	node = Sensor_Fusion(node_name='my_sensor_fusion_node')
	rospy.loginfo("sensor_fusion_node is up and running...")
	rospy.spin()
