cmake_minimum_required(VERSION 2.8.3)
project(sensor_fusion)

find_package(catkin REQUIRED COMPONENTS
  rospy
  duckietown_msgs # Every duckietown packages should use this.
  std_msgs
)


catkin_package()