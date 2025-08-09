/*
 * @Author: BIT807s
 * @LastEditors: yxz_nuc 2410391147@qq.com
 * @LastEditTime: 2023-06-06 17:21:32
 * @FilePath: /real_arm_moveit/run/user/1000/gvfs/sftp:host=192.168.1.116,user=nuc/home/nuc/arm_config_ws/src/robot_arm_gripper/real_arm_hand/src/plan_grasps_with_car.cpp
 * @Description: 机械臂pick & place
 * Copyright (c) 2023 by BIT807s, All Rights Reserved.
 */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/String.h>
// yolo发布的物体中心点位置信息msg
#include <rs_yolo/Info.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <plan_grasps_step.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;
bool pick_attempt = false;	   // 是否执行抓取命令
bool place_attempt = false;	   // 是否执行放置命令
bool sub_obj_position = false; // 物体的坐标只订阅一次cup的
static const std::string PLANNING_GROUP = "arm";
static const std::string Gripper_GROUP = "gripper";

// 声明
// moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
// const moveit::core::JointModelGroup *joint_model_group;
moveit::core::RobotStatePtr current_state_arm;
// const moveit::core::JointModelGroup *gripper_joint_model_group;
moveit::core::RobotStatePtr current_state;
moveit::planning_interface::MoveGroupInterface::Plan my_plan;
// bool success = (move_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
bool success;

void camera_callback(const rs_yolo::Info &msg)
{

	if (msg.classification == "bottle")
	{
		// std::cout<< "bottle camera_callback succees!!!" << std::endl;
		//  if(!sub_obj_position)
		//  {
		object_position.x = msg.z / 1000 + 0.120000; // 将yolo mm单位改为m
		object_position.y = -msg.x / 1000;
		object_position.z = -msg.y / 1000;
		object_position.classification = msg.classification;
		object_position.confidence = msg.confidence;
		sub_obj_position = true;
		//   // 打印
		// std::cout<< " X object in camera Frame in callback:" << object_position.x << std::endl;
		// std::cout<< " Y object in camera Frame :" << object_position.y<< std::endl;
		// std::cout<< " Z object in camera Frame :" << object_position.z << std::endl;
		// sleep(5000);// 50s
		// }
	}
}

void grasp_callback(const std_msgs::String &msg)
{
	// arm setup
	const moveit::core::JointModelGroup *joint_model_group = move_group_interface->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	const moveit::core::JointModelGroup *gripper_joint_model_group = gripper_group_interface->getCurrentState()->getJointModelGroup(Gripper_GROUP);

	current_state_arm = move_group_interface->getCurrentState();
	current_state = gripper_group_interface->getCurrentState();
	move_group_interface->setStartState(*move_group_interface->getCurrentState());
	gripper_group_interface->setStartState(*gripper_group_interface->getCurrentState());
	std::cout << " grasp_callback succees!!!" << std::endl;

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	planning_scene_interface.addCollisionObjects(collision_objects);

	// Getting Basic Information
	// ^^^^^^^^^^^^^^^^^^^^^^^^^
	//
	// We can print the name of the reference frame for this robot.
	ROS_INFO_NAMED("pick & place", "Planning frame: %s", move_group_interface->getPlanningFrame().c_str());

	// We can also print the name of the end-effector link for this group.
	ROS_INFO_NAMED("pick & place", "End effector link: %s", move_group_interface->getEndEffectorLink().c_str());

	// We can get a list of all the groups in the robot:
	ROS_INFO_NAMED("pick & place", "Available Planning Groups:");

	std::copy(move_group_interface->getJointModelGroupNames().begin(),
			  move_group_interface->getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

	// ocm.link_name = "link6";
	// ocm.header.frame_id = "base_link";
	// ocm.orientation.x = -0.002361;
	// ocm.orientation.y = -0.709973;
	// ocm.orientation.z = -0.004482;
	// ocm.orientation.w = 0.704211;
	// // ocm.orientation.w = 1.000000;
	// ocm.absolute_x_axis_tolerance = 0.1;
	// ocm.absolute_y_axis_tolerance = 0.1;
	// ocm.absolute_z_axis_tolerance = 0.1;
	// ocm.weight = 1.0;

	if (msg.data == "Pick" )
	{
		// // 添加可碰撞的物体
		// moveit_msgs::AttachedCollisionObject attached_object;
		// attached_object.link_name = "base_link";
		// attached_object.object.header.frame_id = move_group_interface->getPlanningFrame();
		// /* The id of the object */
		// attached_object.object.id = "bottle";

		// /* A default pose */
		// geometry_msgs::Pose pose;
		// pose.position.x = object_position.x;
		// pose.position.y = object_position.y;
		// pose.position.z = object_position.z;
		// pose.orientation.w = 1.0;

		// /* Define a box to be attached */
		// shape_msgs::SolidPrimitive primitive;
		// primitive.type = primitive.BOX;
		// primitive.dimensions.resize(3);
		// primitive.dimensions[0] = 0.05;
		// primitive.dimensions[1] = 0.05;
		// primitive.dimensions[2] = 0.1;

		// attached_object.object.primitives.push_back(primitive);
		// attached_object.object.primitive_poses.push_back(pose);

		// // Note that attaching an object to the robot requires
		// // the corresponding operation to be specified as an ADD operation.
		// attached_object.object.operation = attached_object.object.ADD;

		// // Since we are attaching the object to the robot hand to simulate picking up the object,
		// // we want the collision checker to ignore collisions between the object and the robot hand.
		// attached_object.touch_links = std::vector<std::string>{"Left", "Right", "left_finger", "left_in", "right_finger", "right_in"};

		// planning_scene_interface.applyAttachedCollisionObject(attached_object);

		pick_attempt = true;
		// Visualization
		// ^^^^^^^^^^^^^
		// The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
		// and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
		visual_tools->deleteAllMarkers();
		// visual_tools->loadRemoteControl();
		text_pose.translation().z() = 1.0;
		visual_tools->publishText(text_pose, "object pickup", rvt::WHITE, rvt::XLARGE);
		visual_tools->trigger();
		// visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
		// ros::Duration(3).sleep();
		/////////////////////////////////////////////////////////////////////////////////////
		geometry_msgs::Pose target_pose_pick_pre1;

		// test_constraints.orientation_constraints.push_back(ocm);
		// move_group_interface->setPathConstraints(test_constraints);

		// current_state->copyJointGroupPositions(joint_model_group,target_pose_pick_pre1);
		// 爪子舵机朝下
		// target_pose1.orientation.x = 0.707145;
		// target_pose1.orientation.y = -0.000033;
		// target_pose1.orientation.z = -0.707069;
		// target_pose1.orientation.w = 0.000009;
		// 爪子舵机朝上
		target_pose_pick_pre1.orientation.x = -0.002361;
		target_pose_pick_pre1.orientation.y = -0.709973;
		target_pose_pick_pre1.orientation.z = -0.004482;
		target_pose_pick_pre1.orientation.w = 0.704211;
		// target_pose_pick_pre1.position.x =0.3;
		// target_pose_pick_pre1.position.y = -0.2;
		// target_pose_pick_pre1.position.z = 0.15;

		target_pose_pick_pre1.position.x = object_position.x - 0.050000;
		target_pose_pick_pre1.position.y = object_position.y + 0.000000;
		target_pose_pick_pre1.position.z = object_position.z + 0.050000;
		// target_pose_pick_pre1.position.x = object_position.z - 0.100000;
		// target_pose_pick_pre1.position.y = -object_position.x + 0.000000;
		// target_pose_pick_pre1.position.z = -object_position.y + 0.100000;

		// target_pose_pick_pre1.position.x = 0.21;
		// target_pose_pick_pre1.position.y = 0;
		// target_pose_pick_pre1.position.z = 0.27;
		// 打印
		std::cout << " X object in camera Frame " << target_pose_pick_pre1.position.x << std::endl;
		std::cout << " Y object in camera Frame :" << target_pose_pick_pre1.position.y << std::endl;
		std::cout << " Z object in camera Frame :" << target_pose_pick_pre1.position.z << std::endl;

		move_group_interface->setPoseTarget(target_pose_pick_pre1);
		move_group_interface->setGoalTolerance(0.01);

		//   for (int i = 0; i < 13; i++) {
		//     move_group_interface->plan(my_plan);
		// }
		// success = (move_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		move_group_interface->plan(my_plan);

		// Visualizing plans
		visual_tools->publishAxisLabeled(target_pose_pick_pre1, "pose2");
		visual_tools->publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
		visual_tools->publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
		visual_tools->trigger();

		// success = (move_group_interface->execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		move_group_interface->execute(my_plan);
		// visual_tools->prompt("Pick-pre1 finish ");
		// ros::Duration(5).sleep();
		// move_group_interface->clearPathConstraints();
		std::cout << " Pick-pre1 finish" << std::endl;

		// 打开爪子
		//  ^^^^^^^^^^^^^^^^^///////////////////////////////////////////////////////////////////////
		current_state->copyJointGroupPositions(gripper_joint_model_group, open_gripper);

		open_gripper[0] = 0;
		open_gripper[1] = 0;
		open_gripper[2] = 0;
		open_gripper[3] = 0;
		open_gripper[4] = 0;
		open_gripper[5] = 0;
		gripper_group_interface->setJointValueTarget(open_gripper);
		gripper_group_interface->setMaxVelocityScalingFactor(1.0);	   // 0,05
		gripper_group_interface->setMaxAccelerationScalingFactor(1.0); // 0.05

		// success = (gripper_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		gripper_group_interface->setStartState(*gripper_group_interface->getCurrentState());

		gripper_group_interface->plan(my_plan);
		std::cout << " gripper open plan finish" << std::endl;
		// gripper_group_interface->execute(my_plan);
		// Visualize the plan in RViz
		visual_tools->deleteAllMarkers();
		visual_tools->publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
		visual_tools->publishTrajectoryLine(my_plan.trajectory_, gripper_joint_model_group);
		visual_tools->trigger();

		gripper_group_interface->execute(my_plan);
		// visual_tools->prompt("Gripper open, wait to pick the object");
		std::cout << "Gripper open, wait to pick the object" << std::endl;
		ros::Duration(3).sleep();

		// 平移x轴前进移动到物体位置
		//  ^^^^^^^^^^^^^^^^^
		geometry_msgs::Pose target_pose2;
		// current_state->copyJointGroupPositions(joint_model_group,target_pose2);
		// 爪子舵机朝下
		//  target_pose2.orientation.x = 0.707145;
		//  target_pose2.orientation.y = -0.000033;
		//  target_pose2.orientation.z = -0.707069;
		//  target_pose2.orientation.w = 0.000009;
		//  爪子舵机朝上
		target_pose2.orientation.x = -0.002361;
		target_pose2.orientation.y = -0.709973;
		target_pose2.orientation.z = -0.004482;
		target_pose2.orientation.w = 0.704211;
		target_pose2.position.x = target_pose_pick_pre1.position.x + 0.05;
		target_pose2.position.y = target_pose_pick_pre1.position.y;
		target_pose2.position.z = target_pose_pick_pre1.position.z - 0.05;

		// 打印
		std::cout << " X target_pose2 :" << target_pose2.position.x << std::endl;
		std::cout << " Y target_pose2 :" << target_pose2.position.y << std::endl;
		std::cout << " Z target_pose2 :" << target_pose2.position.z << std::endl;

		move_group_interface->setPoseTarget(target_pose2);
		move_group_interface->setGoalTolerance(0.01);

		// success = (move_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		move_group_interface->setStartState(*move_group_interface->getCurrentState());

		move_group_interface->plan(my_plan);

		visual_tools->publishAxisLabeled(target_pose2, "pose2");
		visual_tools->publishText(target_pose2, "Pose Goal", rvt::WHITE, rvt::XLARGE);
		visual_tools->publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
		visual_tools->trigger();

		move_group_interface->execute(my_plan);
		// visual_tools->prompt("Close to the object");
		std::cout << "Close to the object finish" << std::endl;
		// ros::Duration(1).sleep();

		// 关闭爪子抓取物体
		//  ^^^^^^^^^^^^^^^^^
		current_state->copyJointGroupPositions(gripper_joint_model_group, closed_gripper);

		closed_gripper[0] = 1;
		closed_gripper[1] = -1;
		closed_gripper[2] = 1;
		closed_gripper[3] = 1;
		closed_gripper[4] = -1;
		closed_gripper[5] = -1;

		gripper_group_interface->setJointValueTarget(closed_gripper);
		gripper_group_interface->setMaxVelocityScalingFactor(0.05);
		gripper_group_interface->setMaxAccelerationScalingFactor(0.05);

		// success = (gripper_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		gripper_group_interface->setStartState(*gripper_group_interface->getCurrentState());

		gripper_group_interface->plan(my_plan);

		// Visualize the plan in RViz
		visual_tools->deleteAllMarkers();
		visual_tools->publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
		visual_tools->publishTrajectoryLine(my_plan.trajectory_, gripper_joint_model_group);
		visual_tools->trigger();
		gripper_group_interface->execute(my_plan);
		std::cout << "gripper close finish" << std::endl;
		// visual_tools->prompt("Closed the gripper, pick the object");
		// sleep(50);
		ros::Duration(3).sleep();

		// 进入work状态

		current_state_arm->copyJointGroupPositions(joint_model_group, arm_work);

		arm_work[0] = 0.4;
		arm_work[1] = 0.7145;
		arm_work[2] = 0.7502;
		arm_work[3] = 0;
		arm_work[4] = 0.7461;
		arm_work[5] = 0;
		move_group_interface->setJointValueTarget(arm_work);
		move_group_interface->setMaxVelocityScalingFactor(0.05);
		move_group_interface->setMaxAccelerationScalingFactor(0.05);

		// success = (move_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		move_group_interface->setStartState(*move_group_interface->getCurrentState());
		move_group_interface->plan(my_plan);
		std::cout << "arm_work plan finish" << std::endl;

		// Visualize the plan in RViz
		visual_tools->deleteAllMarkers();
		visual_tools->publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
		visual_tools->publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
		visual_tools->trigger();
		move_group_interface->execute(my_plan);
		// move_group_interface->setStartState(*move_group_interface->getCurrentState());
		// visual_tools->prompt("go to work with the object");
		// ros::Duration(3).sleep();
		arm_finish_status.data = "pick_finish";
		arm_finish_pub.publish(arm_finish_status);
	}
	else if (msg.data == "Place")
	{
		place_attempt = true;
		// 移动到目标的放置地点
		//  ^^^^^^^^^^^^^^^^^/////////////////////////////////////////////////////////////////
		ocm.link_name = "link6";
		ocm.header.frame_id = "base_link";
		ocm.orientation.x = -0.002361;
		ocm.orientation.y = -0.709973;
		ocm.orientation.z = -0.004482;
		ocm.orientation.w = 0.704211;
		// ocm.orientation.w = 1.000000;
		ocm.absolute_x_axis_tolerance = 0.1;
		ocm.absolute_y_axis_tolerance = 0.1;
		ocm.absolute_z_axis_tolerance = 0.1;
		ocm.weight = 1.0;

		// test_constraints.orientation_constraints.push_back(ocm);
		// move_group_interface->setPathConstraints(test_constraints);

		geometry_msgs::Pose target_pose3;
		// current_state->copyJointGroupPositions(joint_model_group, target_pose3);
		// 爪子舵机朝下
		// target_pose3.orientation.x = 0.707145;
		// target_pose3.orientation.y = -0.000033;
		// target_pose3.orientation.z = -0.707069;
		// target_pose3.orientation.w = 0.000009;
		// 爪子舵机朝上
		target_pose3.orientation.x = -0.002361;
		target_pose3.orientation.y = -0.709973;
		target_pose3.orientation.z = -0.004482;
		target_pose3.orientation.w = 0.704211;
		target_pose3.position.x = 0.320000;
		target_pose3.position.y = -0.200000;
		target_pose3.position.z = +0.000000;

		move_group_interface->setPoseTarget(target_pose3);
		move_group_interface->setGoalTolerance(0.01);
		std::cout << " X in Robot Frame :" << target_pose3.position.x << std::endl;
		std::cout << " Y in Robot Frame :" << target_pose3.position.y << std::endl;
		std::cout << " Z in Robot Frame :" << target_pose3.position.z << std::endl;

		// success = (move_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		move_group_interface->plan(my_plan);

		visual_tools->publishAxisLabeled(target_pose3, "pose3");
		visual_tools->publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
		visual_tools->publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
		visual_tools->trigger();

		move_group_interface->execute(my_plan);
		move_group_interface->setStartState(*move_group_interface->getCurrentState());
		// visual_tools->prompt("Move to the placing place");
		// ros::Duration(2).sleep();
		// move_group_interface->clearPathConstraints();
		// 打开爪子放置物体
		//  ^^^^^^^^^^^^^^^^^
		current_state->copyJointGroupPositions(gripper_joint_model_group, open_gripper);

		open_gripper[0] = 0;
		open_gripper[1] = 0;
		open_gripper[2] = 0;
		open_gripper[3] = 0;
		open_gripper[4] = 0;
		open_gripper[5] = 0;
		gripper_group_interface->setJointValueTarget(open_gripper);
		gripper_group_interface->setMaxVelocityScalingFactor(0.05);
		gripper_group_interface->setMaxAccelerationScalingFactor(0.05);

		// success = (gripper_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		gripper_group_interface->setStartState(*gripper_group_interface->getCurrentState());

		gripper_group_interface->plan(my_plan);

		// Visualize the plan in RViz
		visual_tools->deleteAllMarkers();
		visual_tools->publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
		visual_tools->publishTrajectoryLine(my_plan.trajectory_, gripper_joint_model_group);
		visual_tools->trigger();
		gripper_group_interface->execute(my_plan);
		// visual_tools->prompt("Open the gripper to drop the object");
		ros::Duration(3).sleep();

		////机械臂回到home位置
		// ^^^^^^^^^^^^^^^^^/////////////////////////////////////////////////////////
		std::vector<double> arm_home;
		current_state_arm->copyJointGroupPositions(joint_model_group, arm_home);

		arm_home[0] = 0;
		arm_home[1] = 0;
		arm_home[2] = 0;
		arm_home[3] = 0;
		arm_home[4] = 0;
		arm_home[5] = 0;
		move_group_interface->setJointValueTarget(arm_home);
		move_group_interface->setMaxVelocityScalingFactor(0.05);
		move_group_interface->setMaxAccelerationScalingFactor(0.05);

		// success = (move_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		move_group_interface->setStartState(*move_group_interface->getCurrentState());

		move_group_interface->plan(my_plan);

		// Visualize the plan in RViz
		visual_tools->deleteAllMarkers();
		visual_tools->publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
		visual_tools->publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
		visual_tools->trigger();
		move_group_interface->execute(my_plan);
		// visual_tools->prompt("arm go home");
		// ros::Duration(10).sleep();
		// 关闭爪子回到初始状态
		// ^^^^^^^^^^^^^^^^^//////////////////////////////////////////////////
		current_state->copyJointGroupPositions(gripper_joint_model_group, closed_gripper);
		closed_gripper[0] = 0;
		closed_gripper[1] = 0;
		closed_gripper[2] = 0;
		closed_gripper[3] = 0;
		closed_gripper[4] = 0;
		closed_gripper[5] = 0;

		gripper_group_interface->setJointValueTarget(closed_gripper);
		gripper_group_interface->setMaxVelocityScalingFactor(0.05);
		gripper_group_interface->setMaxAccelerationScalingFactor(0.05);

		// success = (gripper_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		gripper_group_interface->setStartState(*gripper_group_interface->getCurrentState());

		gripper_group_interface->plan(my_plan);

		// Visualize the plan in RViz
		visual_tools->deleteAllMarkers();
		visual_tools->publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
		visual_tools->publishTrajectoryLine(my_plan.trajectory_, gripper_joint_model_group);
		visual_tools->trigger();
		gripper_group_interface->execute(my_plan);
		//  visual_tools->prompt("gripper go home");
		// ros::Duration(2).sleep();
		arm_finish_status.data = "place_finish";
		arm_finish_pub.publish(arm_finish_status);
	}
}

void initPlanningGroup()
{

	move_group_interface = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
	gripper_group_interface = new moveit::planning_interface::MoveGroupInterface(Gripper_GROUP);

	visual_tools = new moveit_visual_tools::MoveItVisualTools("base_link");
	// my_plan=new moveit::planning_interface::MoveGroupInterface::Plan ;

	// define a collision object ROS message for the robot to avoid.
	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = move_group_interface->getPlanningFrame();
	collision_object.id = "box1";

	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[primitive.BOX_X] = 0.7;
	primitive.dimensions[primitive.BOX_Y] = 0.7;
	primitive.dimensions[primitive.BOX_Z] = 0.1;

	geometry_msgs::Pose box_pose;
	box_pose.orientation.w = 1.0;
	box_pose.position.x = -0.25;
	box_pose.position.y = 0.00;
	box_pose.position.z = -0.07;

	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(box_pose);
	collision_object.operation = collision_object.ADD;

	// std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(collision_object);

	ROS_INFO_NAMED("pick & place", "Add an object into the world");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "plan_grasps_step");
	ros::NodeHandle node_handle;

	tf::StampedTransform tip_to_base_;
	tf::TransformListener tf_camera_to_robot;

	initPlanningGroup();

	arm_grasp_sub = node_handle.subscribe("arm_to_grasp", 100, grasp_callback);
	arm_finish_pub = node_handle.advertise<std_msgs::String>("arm_to_finish", 1);
	// yolov5物体中心位置坐标q订阅
	grasps_sub_ = node_handle.subscribe("/detect_result_out", 100, camera_callback); // 话题名,queue_size,回调

	// ros::AsyncSpinner spinner(6);// Use 6 threads
	// spinner.start(); // spin() will not return until the node has been shutdown

	ros::MultiThreadedSpinner spinner(8); // Use 4 threads
	spinner.spin();						  // spin() will not return until the node has been shutdown

	//   ros::Rate loop_rate(1);//设置循环频率，10Hz；也可以设为其他频率，如1为1Hz
	// while(ros::ok())//如果roscore还在运行，则ros::ok()为true即进入循环
	// {
	//   //ros::spinOnce();//处理本节点所有的回调函数 !; 此代码执行后，所有回调函数回调一次执行完成
	//   //***********************相机坐标变换***************************************************
	//   //待标定数据
	//   camera_to_base.setX(0);
	//   camera_to_base.setY(0);
	//   camera_to_base.setZ(0);
	//   //相机坐标与/link6的偏移参数,待设置
	//   double camera_shifting[3][1]={0.0,0.0,0.0};
	//   //坐标变换获取
	//   try
	//   {
	//       tf_camera_to_robot.waitForTransform("/base_link", "/link6", ros::Time(0), ros::Duration(50.0));
	//   }
	//   catch (tf::TransformException &ex)
	//   {
	//       ROS_ERROR("[adventure_tf]: (wait) %s", ex.what());
	//       ros::Duration(1.0).sleep();
	//   }
	//   try
	//   {
	//       tf_camera_to_robot.lookupTransform("/base_link", "/link6", ros::Time(0), (tip_to_base_));
	//   }
	//   catch (tf::TransformException &ex)
	//   {
	//       ROS_ERROR("[adventure_tf]: (lookup) %s", ex.what());
	//   }
	//   std::cout<< " main2  success!!!" << std::endl;
	//   // 打印
	//   std::cout<< " X object in camera Frame :" << object_position.x << std::endl;
	//   std::cout<< " Y object in camera Frame :" << object_position.y<< std::endl;
	//   std::cout<< " Z object in camera Frame :" << object_position.z << std::endl;
	//   obj_camera_frame.setX(object_position.x);
	//   obj_camera_frame.setY(object_position.y);
	//   obj_camera_frame.setZ(object_position.z);
	//   //坐标系的转换
	//   obj_robot_frame = tip_to_base_ * obj_camera_frame;
	//   //obj_robot_frame =tip_to_base_* camera_to_base *obj_camera_frame;
	//   // 打印
	//   std::cout<< " X-Co-ordinate in Robot Frame :" << obj_robot_frame.getX() << std::endl;
	//   std::cout<< " Y-Co-ordinate in Robot Frame :" << obj_robot_frame.getY() << std::endl;
	//   std::cout<< " Z-Co-ordinate in Robot Frame :" << obj_robot_frame.getZ() << std::endl;
	//   //***********************************************************************************
	//   loop_rate.sleep();//用于调整循环频率，保证循环频率为10Hz即周期为1s；一般代码执行速度比设定的频率快。

	// }

	// ros::AsyncSpinner spinner(6);
	// spinner.start();
	// ros::spin();
	// ROS_INFO("Done");
	// ros::shutdown();
	return 0;
}
