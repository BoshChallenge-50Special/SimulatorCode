#! /bin/bash 

cd ~/Documents/BFMC_Simulator/startup_workspace
source ~/Documents/BFMC_Simulator/bfmc_workspace/devel/setup.bash
catkin_make

#Da eseguire ogni volta per avviare la macchina. Il simulatore Gazebo deve essere già partito
source devel/setup.bash
rosrun startup_package controller.py
