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
		self.tick_to_meter = np.pi * 0.067 / 135.   # 6.7cm diameter, 135 = #ticks per revolution
		self.z_m = np.zeros((3, 1)) # d_enc, phi_enc, d_cam, phi_cam

		self.msg_fusion = FusionLanePose()

		self.first_encoder_meas_l = 0
		self.first_encoder_meas_r = 0

		self.tot_angle = 0
		self.turn = 1

		self.direction_l = 1
		self.direction_r = 1




		# Publisher
		self.pub_pose_est = rospy.Publisher("~fusion_lane_pose", FusionLanePose, queue_size=1) # for lane_controller_node

		# Subscriber
		
		self.sub_camera_pose =rospy.Subscriber("lane_filter_node/lane_pose", LanePose, self.predict, queue_size=1)
		self.sub_encoder_ticks =rospy.Subscriber("left_wheel_encoder_node/tick", WheelEncoderStamped, self.update_encoder_measurement_l, queue_size=1)
		self.sub_encoder_ticks =rospy.Subscriber("right_wheel_encoder_node/tick", WheelEncoderStamped, self.update_encoder_measurement_r, queue_size=1)
		self.sub_encoder_ticks =rospy.Subscriber("wheels_driver_node/wheels_cmd_executed", WheelsCmdStamped, self.save_wheelscmdstamped, queue_size=1)

	def save_wheelscmdstamped(self, msg_wheelscmdstamped):
		self.direction_l = np.sign(msg_wheelscmdstamped.vel_left)
		self.direction_r = np.sign(msg_wheelscmdstamped.vel_right)

	def update_encoder_measurement_l(self, msg_encoder_ticks):
		if self.first_encoder_meas_l == 0:
			self.last_left_ticks = msg_encoder_ticks.data
			self.first_encoder_meas_l = 1
			return
		else:
			diff_left = (msg_encoder_ticks.data - self.last_left_ticks) * self.direction_l
			self.last_left_ticks = msg_encoder_ticks.data
			alpha = self.tick_to_meter * diff_left / self.wheeltrack
			self.z_m[0] -= alpha
			self.tot_angle -= alpha
			return

	def update_encoder_measurement_r(self, msg_encoder_ticks):
		if self.first_encoder_meas_r == 0:
			self.last_right_ticks = msg_encoder_ticks.data
			self.first_encoder_meas_r = 1
			return
		else:
			diff_right = (msg_encoder_ticks.data - self.last_right_ticks) * self.direction_r
			self.last_right_ticks = msg_encoder_ticks.data
			alpha = self.tick_to_meter * diff_right / self.wheeltrack
			self.z_m[0] += alpha
			self.tot_angle += alpha
			return

	def curve_detection(self):

		if abs(self.z_m[0]) > 2.4:
			self.z_m[0] -= np.sign(self.z_m[0]) * np.pi/2
			self.turn += 1

		return


	def predict(self, msg_camera_pose):


		#only camera pose
		self.msg_fusion.header.stamp = rospy.get_rostime()
		self.msg_fusion.d = msg_camera_pose.d
		self.msg_fusion.phi = msg_camera_pose.phi
		self.pub_pose_est.publish(self.msg_fusion)
		

		self.curve_detection()

		if self.turn != 0:
			actual_wheeltrack = self.wheeltrack * self.tot_angle / (self.turn * np.pi/2)
			rospy.loginfo("turns: %s phi_tot: %s baseline: %s" %(self.turn, self.tot_angle, abs(actual_wheeltrack)))

		return




if __name__ == '__main__':
	node = Sensor_Fusion(node_name='my_sensor_fusion_node')
	rospy.loginfo("system_ident_wheeltrack is up and running...")
	rospy.spin()
