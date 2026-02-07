#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <conio.h>
#include <Eigen/Eigen>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    local_pos = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kb_ctrl");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                                ("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
                                    ("/mavros/local_position/pose", 1, local_pos_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                   ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                       ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                         ("mavros/set_mode");

    // The setpoint publishing rate MUST be faster than 2Hz.
    ros::Rate rate(50.0);

    // Wait for FCU connection.
    while (ros::ok() && current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
    pose.pose.orientation.w = 1;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request(0);

    while (ros::ok()) {
        if (current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        double dx = 0;
        double dy = 0;
        double dz = 0;
        double dyaw = 0;
        char in_c;
        if (kbhit() && (in_c = getch()) && in_c != EOF) {
            switch (in_c) {
            case '8':    // key up
                dz = 0.04;
                break;
            case '5':    // key down
                dz = -0.04;
                break;
            case '6':    // key right
                dyaw = -0.03;
                break;
            case '4':    // key left
                dyaw = 0.03;
                break;
            case 'w':
                dx = 0.04;
                break;
            case 's':
                dx = -0.04;
                break;
            case 'a':
                dy = 0.04;
                break;
            case 'd':
                dy = -0.04;
                break;
            case 'q':
                return 0;
            default:
                ;
            }
        }

        Eigen::Quaterniond qd(pose.pose.orientation.w,
                             pose.pose.orientation.x,
                             pose.pose.orientation.y,
                             pose.pose.orientation.z);
        Eigen::AngleAxisd drot(dyaw, Eigen::Vector3d(0, 0, 1));
        qd = qd*drot;
        pose.pose.orientation.w = qd.w();
        pose.pose.orientation.x = qd.x();
        pose.pose.orientation.y = qd.y();
        pose.pose.orientation.z = qd.z();

        // Eigen::Quaterniond q(local_pos.pose.orientation.w,
        //                      local_pos.pose.orientation.x,
        //                      local_pos.pose.orientation.y,
        //                      local_pos.pose.orientation.z);
        Eigen::Vector3d dp(dx, dy, dz);
        Eigen::Vector3d dp_body = qd*dp;
        pose.pose.position.x += dp_body.x();
        pose.pose.position.y += dp_body.y();
        pose.pose.position.z += dp_body.z();

        ROS_INFO("setpoint: %.1f, %.1f, %.1f",
                 pose.pose.position.x,
                 pose.pose.position.y,
                 pose.pose.position.z);


        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

