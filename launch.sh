#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
echo "launch"
# roslaunch sensor_fusion sensor_fusion.launch veh:=$VEHICLE_NAME
# roslaunch sensor_fusion cheap_sensor_fusion.launch veh:=$VEHICLE_NAME
roslaunch sensor_fusion system_ident_baseline.launch veh:=$VEHICLE_NAME
# roslaunch sensor_fusion system_ident_diameter.launch veh:=$VEHICLE_NAME

# roslaunch sensor_fusion test_encoder_new.launch veh:=$VEHICLE_NAME