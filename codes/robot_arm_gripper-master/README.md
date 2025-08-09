<!--
 * @Author: xbw-ubuntu 15116921911@example.com
 * @Date: 2023-04-21 20:44:19
 * @LastEditors: xbw-ubuntu 15116921911@example.com
 * @LastEditTime: 2023-04-23 17:45:29
 * @FilePath: /catkin_moveit/src/robot_arm_gripper/README.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->
启动相机和机械臂
~~~
roslaunch real_arm_hand pick_and_palce.launch
~~~

启动机械臂
~~~
roslaunch real_arm_hand real_bringup.launch
~~~

启动相机
~~~
roslaunch real_arm_hand yolo_camera.launch
~~~

启动抓取程序
~~~
rosrun real_arm_hand plan_grasps_step
~~~

发布抓取指令
~~~
rostopic pub arm_to_grasp std_msgs/String "Pick"
rostopic pub arm_to_grasp std_msgs/String "Place"
~~~

打开串口权限

~~~
dmesg | grep ttyS*
sudo chmod 666 /dev/ttyUSB0
cutecom
~~~

valgrind内存调试

```
valgrind --tool=memcheck --leak-check=yes --show-reachable=yes --num-callers=20 --track-fds=yes --track-origins=yes rosrun my_package my_node

```

用于监视 ros节点 my_node