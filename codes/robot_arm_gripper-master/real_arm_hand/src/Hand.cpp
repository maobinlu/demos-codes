/*
 * @Author: xbw-ubuntu 15116921911@example.com
 * @Date: 2023-03-08 14:25:31
 * @LastEditors: yxz_nuc 2410391147@qq.com
 * @LastEditTime: 2023-04-23 22:00:26
 * @FilePath: /catkin_ws/src/arm807_9/hard_arm/src/Hand.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "Hand.h"
#include "serial_com.h"
// 这是action的初始化，传入action的名称
/**
 * @breif:初始化一个节点，初始化变量，启动服务端boost::bind(&Hand::executeCB, this,_1)：boost这个是固定用法，就不深究了，它绑定了一个名为Hand::executeCB的函数，Hand是as_所属的类的名称，executeCB（）是其中的一个成员函数，这个函数写在了as_构造函数的第三个参数位，表示它是一个接受action目标的回调函数（函数名可以自己更改），当action客户端请求一个动作时，这个动作被认为是一个目标，传递给action服务端，此时就moveit的action客户端而言，这个目标就是机械臂运动轨迹，服务端接收到目标后，会自动回调executeCB这个函数，而目标（轨迹）会作为参数传递进来。
 * @param {string} name
 * @return {*}
 */
Hand::Hand(string name)
    : as_(nh_, name, boost::bind(&Hand::executeCB, this, _1), false)
{
    // 动作名
    action_name = name;
    ros::NodeHandle nh_private("~");

    // 初始化关节变量
    handmode = hand_open;
    msg.header.frame_id = "";

    // 服务端启动
    as_.start();
}

Hand::~Hand()
{
    // 释放资源
    cout << "机械臂程序已退出，请等待，并关掉电源，以免电机过热。" << endl;
}

// goal回调函数
void Hand::executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
    std::cout << "executeCB" << std::endl;
    
    std::lock_guard<std::mutex> lock(mutex_);

    // cout << "movetrack finish" << endl;
    // int32_t a = goal->trajectory.points[goal->trajectory.points.size() - 1].positions[0];
    // cout << "position"<< a << endl;

    // cout << "\033[1m\033[35mPoints count : \033[0m" << goal->trajectory.points.size() << endl;

    // 数据发送给下位机，下位机用于执行
    double_t k = goal->trajectory.points[goal->trajectory.points.size() - 1].positions[0];
    feedback_.desired.positions.clear();
    cout << "hand goal" << k << endl;
    if (k <= 0.52)
    {
        cmd_hand_mode = 1;
        cout << "hand close" << endl;
    }
    else
    {
        cmd_hand_mode = 0;
        cout << "hand open" << endl;
    }
    set_cmd_hand_mode();
    json_out();
    json_out();
    json_out();
    json_out();
    json_out();
    // 连发5次
    cout << "complete josn_out" << endl;
    feedback_.desired.positions.push_back(k);

    feedback_.actual.positions.push_back(k);
    as_.publishFeedback(feedback_);
    // 动作完成，反馈结果，设置完成状态
    result_.error_code = result_.SUCCESSFUL;
    as_.setSucceeded(result_);

    /*动作未完成
    result_.error_code = result_.GOAL_TOLERANCE_VIOLATED;
    as_.setAborted(result_);
    */
}