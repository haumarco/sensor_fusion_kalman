#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
echo "launch"
roslaunch sensor_fusion sensor_fusion.launch veh:=$VEHICLE_NAME
# roslaunch sensor_fusion system_ident_axle.launch veh:=$VEHICLE_NAME
# roslaunch sensor_fusion system_ident_diameter.launch veh:=$VEHICLE_NAME