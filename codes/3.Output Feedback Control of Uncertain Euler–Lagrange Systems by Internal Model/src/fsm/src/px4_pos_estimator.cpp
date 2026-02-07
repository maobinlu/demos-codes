    /***************************************************************************************************************************
 * px4_pos_estimator.cpp
 *
 *
 * Update Time: 2019.3.10
 *
 * 说明: mavros位置估计程序
 *      1. 订阅激光SLAM (cartorgrapher_ros节点) 发布的位置信息,从laser坐标系转换至NED坐标系
 *      2. 订阅Mocap设备 (vrpn-client-ros节点) 发布的位置信息，从mocap坐标系转换至NED坐标系
 *      3. 订阅飞控发布的位置、速度及欧拉角信息，作对比用
 *      4. 存储飞行数据，实验分析及作图使用
 *      5. 选择激光SLAM或者Mocap设备作为位置来源，发布位置及偏航角(xyz+yaw)给飞控
 *
***************************************************************************************************************************/

//头文件
#include <ros/ros.h>

#include <iostream>
#include <eigen3/Eigen/Eigen>
// #include <fsm/command_acc.h>


#include <fsm/math_utils.h>
#include <fsm/Frame_tf_utils.h>
//msg 头文件
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Range.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>


using namespace std;
//---------------------------------------vicon定位相关------------------------------------------
Eigen::Vector3d pos_drone_mocap;                          //无人机当前位置 (vicon)
Eigen::Quaterniond q_mocap;
Eigen::Vector3d Euler_mocap;                              //无人机当前姿态 (vicon)
//---------------------------------------VINS定位相关------------------------------------------
Eigen::Vector3d pos_drone_VINS;                          //无人机当前位置 (VINS)
Eigen::Quaterniond q_VINS;
Eigen::Vector3d Euler_VINS;                              //无人机当前姿态(VINS)
//---------------------------------------无人机位置及速度--------------------------------------------
Eigen::Vector3d pos_drone_fcu;                           //无人机当前位置 (来自fcu)
Eigen::Vector3d vel_drone_fcu;                           //无人机上一时刻位置 (来自fcu)

Eigen::Quaterniond q_fcu;
Eigen::Vector3d Euler_fcu;                                          //无人机当前欧拉角(来自fcu)
//---------------------------------------发布相关变量--------------------------------------------
geometry_msgs::PoseStamped vision;
// fsm::command_acc READY;
Eigen::Vector3d pos_flag;
bool ready_for_pub = false;
int flag=0;
float get_dt(ros::Time last);                                          
// void printf_info();     
void pose_pub_timer_cb(const ros::TimerEvent& TE);                                                   
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Read the Drone Position from the Mavros Package [Frame: ENU]
    Eigen::Vector3d pos_drone_fcu_enu(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
    
    pos_drone_fcu = pos_drone_fcu_enu;
    // Read the Quaternion from the Mavros Package [Frame: ENU]
    Eigen::Quaterniond q_fcu_enu(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

    q_fcu = q_fcu_enu;

    //Transform the Quaternion to Euler Angles
    Euler_fcu = quaternion_to_euler(q_fcu);
}
void optitrack_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    //位置 -- optitrack系 到 ENU系
    int optitrack_frame = 1; //Frame convention 0: Z-up -- 1: Y-up
    // Read the Drone Position from the Vrpn Package [Frame: Vicon]  (Vicon to ENU frame)
    Eigen::Vector3d pos_drone_mocap_enu(-msg->pose.position.x,msg->pose.position.z,msg->pose.position.y);
    if (flag == 0) {
        pos_flag[0]=-msg->pose.position.x;
        pos_flag[1]=msg->pose.position.z;
        pos_flag[2]=msg->pose.position.y;   
        flag=1;
    }
    pos_drone_mocap = pos_drone_mocap_enu;

    if(optitrack_frame == 0){
        // Read the Quaternion from the Vrpn Package [Frame: Vicon[ENU]]
        Eigen::Quaterniond q_mocap_enu(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
        q_mocap = q_mocap_enu;
    }
    else
    {
        // Read the Quaternion from the Vrpn Package [Frame: Vicon[ENU]]
        Eigen::Quaterniond q_mocap_enu(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.z, msg->pose.orientation.y); //Y-up convention, switch the q2 & q3
        q_mocap = q_mocap_enu;
    }

    // Transform the Quaternion to Euler Angles
    Euler_mocap = quaternion_to_euler(q_mocap);

}
void VINS_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    Eigen::Vector3d pos_drone(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
    pos_drone_VINS = pos_drone;
    Eigen::Quaterniond q_drone(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    q_VINS = q_drone;
    Euler_VINS = quaternion_to_euler(q_VINS);
}
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    Eigen::Vector3d vel_drone_fcu_enu(msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z);

    vel_drone_fcu = vel_drone_fcu_enu;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_pos_estimator");
    ros::NodeHandle nh("~");

    // ros::Publisher ready = nh.advertise<fsm::command_acc>("/px4/ready", 10);

    //VINS
    ros::Subscriber VINS_sub = nh.subscribe<nav_msgs::Odometry>("/vins_fusion/imu_propagate", 10, VINS_cb);

    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pos_cb);
    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 10, vel_cb);
    ros::Subscriber optitrack_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/robofly/pose", 10, optitrack_cb);
    //  发送飞控位姿信息
    ros::Publisher  vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 100);
    ros::Timer pose_pub_timer = nh.createTimer(ros::Duration(1.0/20.0),pose_pub_timer_cb);
    
    
    
    // 频率
    ros::Rate rate(20.0);

    while(ros::ok())
    {
        ros::spinOnce();
        if(ready_for_pub)
        {
          vision_pub.publish(vision);
          ready_for_pub = false;
        //   printf_info();
        }
        rate.sleep();
    }
    return 0;

}

// void pose_pub_timer_cb(const ros::TimerEvent& TE)
// {
//     vision.pose.position.x = pos_drone_mocap[1] - pos_flag[1];
//     vision.pose.position.y = pos_flag[0] - pos_drone_mocap[0];
//     vision.pose.position.z = pos_drone_mocap[2] ;

//     vision.pose.orientation.x = q_mocap.x();
//     vision.pose.orientation.y = q_mocap.y();
//     vision.pose.orientation.z = q_mocap.z();
//     vision.pose.orientation.w = q_mocap.w();
//     vision.header.stamp = TE.current_real;
//     ready_for_pub = true;
// }
void pose_pub_timer_cb(const ros::TimerEvent& TE)
{
    vision.pose.position.x = pos_drone_VINS[0];
    vision.pose.position.y = pos_drone_VINS[1];
    vision.pose.position.z = pos_drone_VINS[2] ;

    vision.pose.orientation.x = q_VINS.x();
    vision.pose.orientation.y = q_VINS.y();
    vision.pose.orientation.z = q_VINS.z();
    vision.pose.orientation.w = q_VINS.w();
    vision.header.stamp = TE.current_real;
    ready_for_pub = true;
}
//获取当前时间 单位：秒
float get_dt(ros::Time last)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-last.sec;
    float currTimenSec = time_now.nsec / 1e9 - last.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

// void printf_info()
// {
//     //固定的浮点显示
//     cout.setf(ios::fixed);
//     //setprecision(n) 设显示小数精度为n位
//     cout<<setprecision(2);
//     //左对齐
//     cout.setf(ios::left);
//     // 强制显示小数点
//     cout.setf(ios::showpoint);
//     // 强制显示符号
//     cout.setf(ios::showpos);
//         cout <<">>>>>>>>>>>>>>>>>>>>>>>>Vicon Info [ENU Frame]<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
//         cout << "Pos_vicon [X Y Z] : " << pos_drone_mocap[1] - pos_flag[1] << " [ m ] "<< pos_flag[0] - pos_drone_mocap[0] <<" [ m ] "<< pos_drone_mocap[2] <<" [ m ] "<<endl;
//         cout << "Euler_vicon [Yaw] : " << Euler_mocap[2] * 180/M_PI<<" [deg]  "<<endl;
//         cout <<">>>>>>>>>>>>>>>>>>>>>>>>FCU_POS_ESTIMATOR<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
//         cout << "Pos_fcu [X Y Z] : " << pos_drone_fcu[0] << " [ m ] "<< pos_drone_fcu[1] <<" [ m ] "<< pos_drone_fcu[2] <<" [ m ] "<<endl;
//         cout << "Vel_fcu [X Y Z] : " << vel_drone_fcu[0] << " [m/s] "<< vel_drone_fcu[1] <<" [m/s] "<< vel_drone_fcu[2] <<" [m/s] "<<endl;
//         cout << "Euler_fcu [Yaw] : " << Euler_fcu[2] * 180/M_PI<<" [deg] "<<endl;
// }
