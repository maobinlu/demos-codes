/*
 * @Author: yxz 2410391147@qq.com
 * @Date: 2023-03-09 16:17:58
 * @LastEditors: yxz_nuc 2410391147@qq.com
 * @LastEditTime: 2023-04-12 10:18:20
 * @FilePath: /arm807_hand_ws/src/real_arm_hand/include/real_arm_hand/plan_grasps_step.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
# define M_PI		3.14159265358979323846	/* pi */
ros::Subscriber grasps_sub_,arm_grasp_sub;
ros::Publisher arm_finish_pub;
rs_yolo::Info object_position;
tf::Vector3 obj_camera_frame, obj_robot_frame,camera_to_base;

Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
moveit_msgs::OrientationConstraint ocm;
moveit_msgs::Constraints test_constraints;

namespace rvt = rviz_visual_tools;
moveit_visual_tools::MoveItVisualTools *visual_tools;

moveit::planning_interface::MoveGroupInterface *move_group_interface;
moveit::planning_interface::MoveGroupInterface *gripper_group_interface;

std::vector<double> closed_gripper;
std::vector<double> open_gripper;

std_msgs::String arm_finish_status;//机械臂是否完成pick or Place过程，传输信息给car



std::vector<moveit_msgs::CollisionObject> collision_objects;


std::vector<double> arm_work;

