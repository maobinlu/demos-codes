source /home/host/ius_uav/ros_ws/devel_isolated/setup.bash
source /home/host/ius_uav/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash /home/host/ius_uav/PX4-Autopilot /home/host/ius_uav/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/host/ius_uav/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/host/ius_uav/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/host/ius_uav/ros_ws/src/px4_sim/models
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/home/host/ius_uav/ros_ws/src/px4_sim/lib
