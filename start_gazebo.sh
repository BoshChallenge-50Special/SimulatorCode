#! /bin/bash
USERNAME = "marco"
cd ~/Documents/BFMC_Simulator/bfmc_workspace/
source devel/setup.bash
export GAZEBO_MODEL_PATH="/home/$USERNAME/Documents/BFMC_Simulator/bfmc_workspace/src/models_pkg:$GAZEBO_MODEL_PATH"
export ROS_PACKAGE_PATH="/home/$USERNAME/Documents/BFMC_Simulator/bfmc_workspace/src:$ROS_PACKAGE_PATH"
#roslaunch sim_pkg map_with_all_objects.launch
roslaunch sim_pkg map_with_car.launch
