#pragma once

#include <ros/ros.h>

class MissionParam
{
public:
    MissionParam() {}

    void load_param(ros::NodeHandle& nh) {
        nh.getParam("mission_pillar_x_down", mission_pillar_x_down_);
        nh.getParam("mission_pillar_x_up", mission_pillar_x_up_);
        nh.getParam("mission_pillar_y_left", mission_pillar_y_left_);
        nh.getParam("mission_pillar_y_right", mission_pillar_y_right_);

        nh.getParam("mission_tunnel_x_down", mission_tunnel_x_down_);
        nh.getParam("mission_tunnel_x_up", mission_tunnel_x_up_);
        nh.getParam("mission_tunnel_y_left", mission_tunnel_y_left_);
        nh.getParam("mission_tunnel_y_right", mission_tunnel_y_right_);

        nh.getParam("mission_maze_x_up", mission_maze_x_up_);
        nh.getParam("mission_maze_x_down", mission_maze_x_down_);
        nh.getParam("mission_maze_y_left", mission_maze_y_left_);
        nh.getParam("mission_maze_y_right", mission_maze_y_right_);

        nh.getParam("mission_loop_x_up", mission_loop_x_up_);
        nh.getParam("mission_loop_x_down", mission_loop_x_down_);
        nh.getParam("mission_loop_y_left", mission_loop_y_left_);
        nh.getParam("mission_loop_y_right", mission_loop_y_right_);

        nh.getParam("mission_m_loop_x_up", mission_m_loop_x_up_);
        nh.getParam("mission_m_loop_x_down", mission_m_loop_x_down_);
        nh.getParam("mission_m_loop_y_left", mission_m_loop_y_left_);
        nh.getParam("mission_m_loop_y_right", mission_m_loop_y_right_);

        nh.getParam("mission_land_x_up", mission_land_x_up_);
        nh.getParam("mission_land_x_down", mission_land_x_down_);
        nh.getParam("mission_land_y_left", mission_land_y_left_);
        nh.getParam("mission_land_y_right", mission_land_y_right_);
    }

    // mission A fence
    double mission_pillar_x_down_;
    double mission_pillar_x_up_;
    double mission_pillar_y_left_;
    double mission_pillar_y_right_;

    // mission B fence
    double mission_tunnel_x_down_;
    double mission_tunnel_x_up_;
    double mission_tunnel_y_left_;
    double mission_tunnel_y_right_;

    // mission C fence
    double mission_maze_x_up_;
    double mission_maze_x_down_;
    double mission_maze_y_left_;
    double mission_maze_y_right_;

    // mission D fence
    double mission_loop_x_up_;
    double mission_loop_x_down_;
    double mission_loop_y_left_;
    double mission_loop_y_right_;

    // mission E fence
    double mission_m_loop_x_up_;
    double mission_m_loop_x_down_;
    double mission_m_loop_y_left_;
    double mission_m_loop_y_right_;

    // mission F fence
    double mission_land_x_up_;
    double mission_land_x_down_;
    double mission_land_y_left_;
    double mission_land_y_right_;
};
