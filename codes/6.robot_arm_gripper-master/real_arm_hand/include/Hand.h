/*
 * @Author: xbw-ubuntu 15116921911@example.com
 * @Date: 2022-08-26 16:49:09
 * @LastEditors: xbw-ubuntu 15116921911@example.com
 * @LastEditTime: 2023-03-08 17:16:59
 * @FilePath: /catkin_ws/src/real_arm/include/real_arm/Hand.h
 * @Description: 尝试编写action-serve，针对hand部分的controller
 */
#ifndef HAND_H
#define HAND_H

#include "common.h"

enum Handmode
{
	hand_open = 0,
	hand_close = 1,
};

class Hand
{
public:
	// 传入action的名称
	Hand(string name);
	virtual ~Hand();

	// goal回调函数
	void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);


private:
	//-----------节点和定时器配置-----------
	Handmode handmode;				   // 末端模式
	ros::NodeHandle nh_;			   // 句柄实例
	ros::Time time_current, time_prev; // ros系统时间
	ros::Timer timer;				   // ros定时器
	double period;					   // 定时器周期
	//-------------action相关--------------
	// action名称
	string action_name;

	// 定义action服务端实例
	//  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>  as_;
	actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;

	// 反馈实例
	control_msgs::FollowJointTrajectoryFeedback feedback_;

	// 用来反馈action目标的执行情况，客户端由此可以得知服务端是否执行成功了
	control_msgs::FollowJointTrajectoryResult result_;

	// 路径点容器
	vector<trajectory_msgs::JointTrajectoryPoint> waypoints;

	// 关节状态发布者 消息实例
	ros::Publisher joint_pub_;
	sensor_msgs::JointState msg;
};

#endif // ABBROBOT_H