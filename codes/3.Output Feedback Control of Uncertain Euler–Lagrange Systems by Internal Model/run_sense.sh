sudo chmod 777 /dev/ttyACM0 & sleep 2;
roslaunch realsense2_camera rs_camera.launch & sleep 5
roslaunch ego_planner mavros.launch & sleep 5
roslaunch vins vins_run.launch & sleep 5
roslaunch fsm px4_pos_estimator.launch & sleep 2
wait
