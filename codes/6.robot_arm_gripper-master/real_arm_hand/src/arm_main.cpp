/*
 * @Author: xbw-ubuntu 15116921911@example.com
 * @Date: 2022-09-14 10:56:12
 * @LastEditors: geyuanliu geyuanliu@gmail.com
 * @LastEditTime: 2023-04-20
 * @FilePath: /robot_arm_gripper/real_arm_hand/src/arm_main.cpp
 * @Description: 机械臂控制主函数
 */
#include "RealRobot.h"
#include "Hand.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "real_arm_node");
    RealRobot robot("arm_controller/follow_joint_trajectory");
    Hand hand("gripper_controller/follow_joint_trajectory");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time time_current = ros::Time::now();
    ros::Time time_prev = time_current;
    double elapsed = 0;
    while (ros::ok())
    {
        time_current = ros::Time::now();
        elapsed = (time_current - time_prev).toSec(); // 发送时间间隔
        if (elapsed * 1000 >= Control_period_ms)
        {
            ;
        }
        else
        {
            usleep(Control_period_ms - elapsed * 1000);
        }
        robot.jointStateUpdate();
        time_prev = time_current;
    }
    cout << "\033[1m\033[32m机械臂程序正在关闭...\033[0m" << endl;
    return 0;
}