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

		self.tick_to_meter = np.pi * 0.067 / 135.   # 6.7cm diameter, 135 = #ticks per revolution

		self.diff_left = 0
		self.diff_right = 0
		self.last_left_ticks = 0
		self.last_right_ticks = 0

		self.phi_enc = 0
		self.d_enc = 0

		self.wheeltrack = 0.101 

		self.msg_fusion = FusionLanePose()

		self.first_encoder_meas_l = 0
		self.first_encoder_meas_r = 0

		self.tot_ticks = 0

		self.direction_l = 1
		self.direction_r = 1



		# Publisher
		self.pub_pose_est = rospy.Publisher("~fusion_lane_pose", FusionLanePose, queue_size=1) # for lane_controller_node

		# Subscriber
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
			#rospy.loginfo("l: %s" %(msg_encoder_ticks.data))
			diff_left = (msg_encoder_ticks.data - self.last_left_ticks) * self.direction_l
			self.last_left_ticks = msg_encoder_ticks.data
			alpha = self.tick_to_meter * diff_left / self.wheeltrack
			self.phi_enc -= alpha
			self.tot_ticks += diff_left
			diff_dist = self.tick_to_meter * 0.5 * diff_left
			self.d_enc += 0.5 * diff_dist * (np.cos(self.phi_enc + alpha) - (np.cos(self.phi_enc))) / alpha
			self.publish()
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
			self.phi_enc += alpha
			self.tot_ticks += diff_right
			diff_dist = self.tick_to_meter * 0.5 * diff_right
			self.d_enc += 0.5 * diff_dist * (np.cos(self.phi_enc - alpha) - (np.cos(self.phi_enc))) / alpha
			self.publish()
			return


	def publish(self):

		self.msg_fusion.header.stamp = rospy.get_rostime()
		self.msg_fusion.d = self.d_enc
		self.msg_fusion.phi = self.phi_enc

		self.pub_pose_est.publish(self.msg_fusion)

		ticks = 0.5 * self.tot_ticks

		if ticks != 0:
			formula = 135 /(np.pi * ticks)
			rospy.loginfo("diameter = %s * < measured_distance_in_m >" %(formula))


		return



if __name__ == '__main__':
	node = Sensor_Fusion(node_name='my_sensor_fusion_node')
	rospy.loginfo("system_ident_diameter is up and running...")
	rospy.spin()

