# Preparation:

Before one can run the sensor_fusion_node a couple of things have to be prepared. 

Download the newest newest version of dt-core from the Duckietown GitHub.

    $ git clone https://github.com/duckietown/dt-core.git

Next open the file dt-core/packages/lane_control/src/lane_controller_node.py and change the line from

    self.sub_lane_reading = rospy.Subscriber("~lane_pose",
                                                 LanePose,
                                                 self.cbAllPoses,
                                                 "lane_filter",
                                                 queue_size=1)

to

    self.sub_lane_reading = rospy.Subscriber("sensor_fusion_node/fusion_lane_pose",
                                                 LanePose,
                                                 self.cbAllPoses,
                                                 "lane_filter",
                                                 queue_size=1)

Now go into the dt-core directory and build the image on the Duckiebot with

    $ docker -H DB_NAME.local build -t duckietown/dt-core:daffy-arm32v7 .

Make sure to restart the containers which depend on dt-core.

#

Change the parameters of the maximal velocity and angular velocity of the kinetics_node. At the time I'm writing this, I was not able to permanently change those values with the

    $ rosparam set /DUCKIEBOT_NAME/kinematics_node/vel_max 0.25
    $ rosparam set /DUCKIEBOT_NAME/kinematics_node/omega_max 3.75
    $ rosservice call /DUCKIEBOT_NAME/kinematics_node/save_calibration

therefore one has to change the value manually by ssh into the Duckiebot. This is also the reason why the diameter and wheel track is saved as a parameter in the code and the official paramters are used.
    


# Run:
clone:

    $ git clone https://github.com/haumarco/sensor_fusion_kalman.git
build:

    $ dts devel build -f --arch arm32v7 -H DB_NAME.local
run:

    $ docker -H DB_NAME.local run --name sensor_fusion -it --rm --privileged --net=host duckietown/sensor_fusion_kalman:v1-arm32v7
	or run in background:
    $ docker -H DB_NAME.local run -d --name sensor_fusion -it --rm --privileged --net=host duckietown/sensor_fusion_kalman:v1-arm32v7

# System Identification

### Wheel track:

In the launch.sh file, use the line with system_ident_wheeltrack.launch instead of sensor_fusion.launch.

Place the Duckiebot on a 3x3 loop and make sure it is standing perfectly in the lane. Start the lane-following demo, run it for 3 loops, stop it when Duckiebot is standing perfectly in the lane again. If the Duckiebot is not perfectly in the lane its position can be adjusted by hand. Note the suggested wheel track. Repeat this procecdure 3 times for both directions of the loop. Average the results and change the wheel track parameter.

### Diameter

In the launch.sh file, use the line with system_ident_diameter.launch instead of sensor_fusion.launch.

Place the Duckiebot on the floor and mark its starting position. Start the container, watch the Duckiebot going straight and stop it after a couple of meters. Now measure the traveled distance and calculate your diameter with the provided formula:

diameter = OUTPUT_OF_CODE * <measured_distance_in_m>

This test does not supply a highly accurate measurement and is just there to double check the chosen diameter and ensure that the bot is not slipping by drifing straight.