cd ~/PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo
source ~/RAT_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/RAT_ws/src/PROJECT_AC2/E2ES/gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/RAT_ws/src/PROJECT_AC2/E2ES/gazebo/models
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/RAT_ws/devel/lib
echo $GAZEBO_RESOURCE_PATH
echo $GAZEBO_MODEL_PATH
echo $GAZEBO_PLUGIN_PATH
# roslaunch e2es rviz.launch &
# sleep 2
roslaunch px4_fast_planner mavros_e2es.launch pause:=false


