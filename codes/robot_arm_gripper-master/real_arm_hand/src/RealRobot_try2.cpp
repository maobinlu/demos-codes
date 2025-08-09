#include "RealRobot.h"
#include "serial_com.h"

// 声明资源互斥锁

// 这是action的初始化，传入action的名称
/**
 * @breif:初始化一个节点，初始化变量，启动服务端boost::bind(&RealRobot::executeCB, this,_1)：boost这个是固定用法，
 * 就不深究了，它绑定了一个名为RealRobot::executeCB的函数，RealRobot是as_所属的类的名称，
 * executeCB（）是其中的一个成员函数，这个函数写在了as_构造函数的第三个参数位，表示它是一个接受action目标的回调函数（函数名可以自己更改），
 * 当action客户端请求一个动作时，这个动作被认为是一个目标，传递给action服务端，此时就moveit的action客户端而言，
 * 这个目标就是机械臂运动轨迹，服务端接收到目标后，会自动回调executeCB这个函数，而目标（轨迹）会作为参数传递进来。
 * @param {string} name
 * @return {*}
 */
RealRobot::RealRobot(string name)
    : as_(nh_, name, boost::bind(&RealRobot::executeCB, this, _1), false), joint_count(7)
{
    // 动作名
    action_name = name;
    ros::NodeHandle nh_private("~");

    // 初始化关节变量
    joint_name.resize(joint_count);
    msg.name.resize(joint_count);
    msg.position.resize(joint_count);
    msg.velocity.resize(joint_count);
    msg.effort.resize(joint_count);
    msg.header.frame_id = "";

    // 对joint_state的中的名称进行配置，是string格式，比较麻烦点
    stringstream ss;
    ss.clear();
    ss.str("");
    for (size_t i = 0; i < joint_count; i++)
    {
        ss << "joint" << i + 1;
        joint_name[i] = ss.str();
        msg.name[i] = joint_name[i];
        ss.clear();
        ss.str("");
    }
    //--------------------------------电机配置初始化-------------------------------
    float N_init[6] = {N51, N50, N50, N51, N50, N1}; // 减速比
    int dir_init[6] = {1, -1, -1, -1, -1, -1};       // 转动方向
    float joints_init[6] = {0, 0, 0, 0, 0, 0};       // 关节初始位置
    float motors_init[6] = {0, 0, 0, 0, 0, 0};       // 电机初始为
    for (int i = 0; i < 6; i++)
    {
        joints[i].N = N_init[i];
        joints[i].same_dir = dir_init[i];
        joints[i].joint_init = joints_init[i];
        joints[i].motor_init = motors_init[i];
    }

    serial_init();

    // 关节发布者初始化，这里的queue_size是1，目前先这样，之后看情况而定
    joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);

    // 定时器启动，定时器周期为0.01秒,之后视情况而定。
    period = 0.01;
    timer = nh_.createTimer(ros::Duration(period), &RealRobot::timerCallback, this);

    // 服务端启动
    as_.start();
}

RealRobot::~RealRobot()
{
    // 释放资源
    cout << "机械臂程序已退出，请等待，并关掉电源，以免电机过热。" << endl;
}

// timer回调函数，用于接收下位机数据
void RealRobot::timerCallback(const ros::TimerEvent &e)
{
    jointStateUpdate();
}

// goal回调函数
void RealRobot::executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
    std::cout << "executeCB" << std::endl;
    // 加锁
    std::lock_guard<std::mutex> lock(mutex_);

    // 数据发送给下位机，下位机用于执行
    trackMove(goal->trajectory);

    // 动作完成，反馈结果，设置完成状态
    result_.error_code = result_.SUCCESSFUL;
    cout << "error_finish:" << error_sum << endl;
    ros::Duration(0.5).sleep();
    as_.setSucceeded(result_);

    // 解锁
    /*动作未完成
    result_.error_code = result_.GOAL_TOLERANCE_VIOLATED;
    as_.setAborted(result_);
    */
}

// 发布者函数
void RealRobot::jointStateUpdate()
{
    // cout << "jointStateUpdate " << endl;
    armRead();
    // joint_angle = (motor_angle - motor_init) / N * dir + joint_init;
    for (int i = 0; i < 6; i++)
    {
        msg.position[i] = (feedback_loc[i] - joints[i].motor_init) / joints[i].N * joints[i].same_dir + joints[i].joint_init;
        msg.velocity[i] = (feedback_spd[i] / joints[i].N);
    }
    msg.position[6] = (1 - feedback_hand_mode) * 1;
    msg.header.stamp = ros::Time::now();
    joint_pub_.publish(msg);
}

/**
 * @brief: 速度设计，对trajectory中的速度进行再设定
 * @return {*}
 */
void RealRobot::speeddesign_smooth(trajectory_msgs::JointTrajectory trajectory)
{
    for (size_t seq = 0; seq < trajectory.points.size(); seq++)
    {
        waypoints.push_back(trajectory.points[seq]);
    }
}

void RealRobot::a_init()
{
    for (size_t i = 0; i < 6; i++)
    {
        a0[i] = 0;
        a1[i] = 0;
        a2[i] = 0;
        a3[i] = 0;
        a4[i] = 0;
        a5[i] = 0;
    }
}

/**
 * @brief 用于生成三次插值函数，默认起始点和目标点处的速度均为0
 * @param point1 起始路径点
 * @param point2 目标路径点
 * @param num    第几关节
 */
void RealRobot::polynomial_three_equaltion(trajectory_msgs::JointTrajectoryPoint point1, trajectory_msgs::JointTrajectoryPoint point2, size_t num)
{
    //
    double_t t12 = (point2.time_from_start - point1.time_from_start).toSec();
    // cout << "s: " << t12 << endl;

    double_t theata_0 = point1.positions[num];
    double_t theata_f = point2.positions[num];

    a0[num] = theata_0;
    a1[num] = 0;
    a2[num] = 3 * (theata_f - theata_0) / pow(t12, 2);
    a3[num] = -2 * (theata_f - theata_0) / pow(t12, 3);
    // cout << "a: " << a0[num] << "||" << a1[num] << "||" << a2[num] << "||" << a3[num] << "||" << a4[num] << "||" << a4[num] << "||" << endl;
    // cout<<"point2: "<<point2.positions[num]<<endl;
    // cout<<"a get : "<<a0[num]+a1[num]*t12+a2[num]*pow(t12,2)+a3[num]*pow(t12,3)+a4[num]*pow(t12,4)+a5[num]*pow(t12,5)<<endl;
}

// 路径执行
void RealRobot::trackMove(trajectory_msgs::JointTrajectory trajectory)
{
    cout << "trackMove start........" << endl;
    feedback_.desired.positions.clear();

    cout << "\033[1m\033[35mPoints count : \033[0m" << trajectory.points.size() << endl;

    // 路径点个数
    int32_t k = trajectory.points.size();

    // 初始化三次插值参数
    a_init();

    // 将路点的终点写入feedback_中

    for (size_t i = 0; i < 6; i++)
    {
        feedback_.desired.positions.push_back(trajectory.points[k - 1].positions[i]);

        // 直接给最后一个点
        cmd_loc[i] = (trajectory.points[k - 1].positions[i] - joints[i].joint_init) * joints[i].N * joints[i].same_dir + joints[i].motor_init;

        // 计算插值函数 这里就直接将第一个点（当前位置）和最后一个点 做三次插值
        polynomial_three_equaltion(trajectory.points[0], trajectory.points[k - 1], i);
    }

    ros::Duration move_time; // 声明时间间隔变量
    ros::Duration one_period;
    double_t waypoints_spd[6] = {0};
    error_sum = 0;
    cout << "test" << endl;
    ros::Time time_begin = ros::Time::now(); // 获得开始规划时间
    while (1)
    {
        move_time = ros::Time::now() - time_begin - trajectory.points[0].time_from_start; // 获得规划过程中用了多少时间
        double_t t = move_time.toSec();
        for (size_t i = 0; i < 6; i++)
        {
            waypoints_spd[i] = a1[i] + 2 * a2[i] * t + 3 * a3[i] * pow(t, 2);

            cmd_spd[i] = waypoints_spd[i] * joints[i].N * joints[i].same_dir;

            feedback_.actual.positions.push_back(msg.position[i]);
            feedback_.actual.velocities.push_back(msg.velocity[i]);
            feedback_.actual.effort.push_back(msg.effort[i]);
            error_sum = abs(msg.position[i] - trajectory.points[k - 1].positions[i]) + error_sum;
            // cout << error_sum << endl;
        }
        // cout << "waypoints_sbd " << waypoints_spd[1] << endl;
        // cout << "point" << trajectory.points[k - 1].positions[2] << endl;
        // cout << "smooth_vel " << a1[1] + 2 * a2[1] * trajectory.points[k - 1].time_from_start.toSec() + 3 * a3[1] * pow(trajectory.points[k - 1].time_from_start.toSec(), 2) << endl;
        // cout<<""<<a2[] + a3[i] * t<<endl;

        set_cmd_loc();
        set_cmd_spd();
        json_out();

        if (error_sum <= 0.005)
        {
            cout << "reach the goal point , error: " << error_sum << endl;
            cout << "joint3 goal " << trajectory.points[k - 1].positions[2] << endl;
            cout << "joint3 realloc" << msg.position[2] << endl;
            break;
        }
        if ((ros::Time::now() - time_begin - trajectory.points[k - 1].time_from_start).toSec() >= 0)
        {
            cout << "move time is so long" << endl;
            cout << "error: " << error_sum << endl;
            break;
        }
        error_sum = 0;

        feedback_.header.stamp = msg.header.stamp;
        as_.publishFeedback(feedback_);

        one_period = ros::Time::now() - time_begin;
        if ((one_period - move_time).toSec() * 1000 >= Control_period_ms)
        {
            cout << "period error " << endl;
            continue;
        }
        // cout << "time " << (one_period - move_time).toSec() * 1000 << endl;

        usleep((one_period - move_time).toSec() * 1000);
    }

    // 路径点执行完毕
    cout << "movetrack finish" << endl;
}

// 数据写入下位机并执行(参考)
void RealRobot::armWrite(trajectory_msgs::JointTrajectoryPoint point)
{
    // 更新数据
    //  motor_angle = (joint_angle - joint_init) * N * dir + motor_init;
    for (int i = 0; i < 6; i++)
    {
        cmd_loc[i] = (point.positions[i] - joints[i].joint_init) * joints[i].N * joints[i].same_dir + joints[i].motor_init;
        cmd_spd[i] = point.velocities[i] * joints[i].N * joints[i].same_dir;
        // host[i + 6] = 512000;
    }
    // cout << "armwrite ok" << host[6] << endl;
    // 赋值
    set_cmd_loc();
    set_cmd_spd();
    json_out();
}

// 从下位机读取数据(参考)
void RealRobot::armRead()
{
    json_input();
}