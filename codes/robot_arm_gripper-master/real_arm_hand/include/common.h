/*
 * @Author: xbw-ubuntu 15116921911@example.com
 * @Date: 2022-08-26 16:49:09
 * @LastEditors: yxz_nuc 2410391147@qq.com
 * @LastEditTime: 2023-04-21 16:46:49
 * @FilePath: /catkin_ws/src/real_arm/include/real_arm/common.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef COMMON_H
#define COMMON_H

//所以平时需要用到的头文件都放在这里
#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <unistd.h>
#include <typeinfo>
#include <inttypes.h>
#include <cstdlib>
#include <ctime>


#include <cmath>
#include <cstring>
#include <string.h>
#include <signal.h>
#include <vector>
//#include "serial/serial.h"

//serial通讯
#include <serial/serial.h>
#include <iostream>
#include <cJSON.h>

//资源互斥锁
#include <mutex>


//ros头文件
#include "ros/ros.h"
// #include "moveit/move_group_interface/move_group_interface.h"
#include "std_msgs/UInt16.h"
#include "sensor_msgs/JointState.h"
#include "robot_state_publisher/robot_state_publisher.h"

//动作编程
#include "actionlib/server/action_server.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/server/server_goal_handle.h" 
#include "control_msgs/FollowJointTrajectoryAction.h"

//声明互斥锁
static std::mutex mutex_;

using namespace std;
//using namespace serial;

//常量定义
const double PI = 3.1415926;


#endif // __COMMON__