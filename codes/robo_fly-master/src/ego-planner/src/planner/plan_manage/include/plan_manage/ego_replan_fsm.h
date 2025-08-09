#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <traj_utils/Bspline.h>
#include <traj_utils/MultiBsplines.h>
#include <geometry_msgs/PoseStamped.h>
#include <traj_utils/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

//add mavros .h
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <ius_msgs/robopos.h>
#include "ius_msgs/CirclePoses.h"
#include <visualization_msgs/MarkerArray.h>

using std::vector;

namespace ego_planner
{

  class EGOReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFENCE_PATH = 3
    };

    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    traj_utils::DataDisp data_disp_;
    traj_utils::MultiBsplines multi_bspline_msgs_buf_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    double waypoints_[50][3];
    int waypoint_num_, wp_id_;
    double planning_horizen_, planning_horizen_time_;
    double emergency_time_;
    bool flag_realworld_experiment_;
    bool enable_fail_safe_;
    // bool local_pt_flag_ = false;
    mavros_msgs::State current_state;
    Eigen::Vector3d robo_pos_1;
    /* planning data */
    bool have_trigger_, have_target_, have_odom_, have_new_target_, have_recv_pre_agent_,new_traj_generate_;
    bool have_pause_;
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_;
    Eigen::Vector3d local_target_vel_;                     // local target state
    std::vector<Eigen::Vector3d> wps_;
    std::vector<Eigen::Vector3d> wpv_;
    std::vector<int> wpjud_;
    

    int current_wp_;
    double yaws[12];
    bool flag_escape_emergency_;
    //ADD Mavros param
    // 读取目标点方式的标志位，true为读取参数服务器，false为从话题读取
    bool readGiven = true;
    // robopos_cb函数中是否进行检查的标志位，false表示第一次读取话题，需要检查；true为不需要检查
    bool finish_init = false;
    bool finish_mission = false;
    double num_mid = 0;
    double num_los = 0;
    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_;
    ros::Subscriber waypoint_sub_, odom_sub_, swarm_trajs_sub_, broadcast_bspline_sub_, trigger_sub_;
    ros::Publisher replan_pub_, new_pub_, bspline_pub_, traj_pub_,data_disp_pub_, swarm_trajs_pub_, broadcast_bspline_pub_;
    ros::Subscriber state_sub,robopos_sub,velVisSub;
    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromCurrentTraj(const int trial_times = 1);

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void readGivenWps(const bool readGiven);
    void planNextWaypoint(const Eigen::Vector3d next_wp,const Eigen::Vector3d next_wvel,const Eigen::Vector3d nnext_wp,const Eigen::Vector3d nnext_wvel);
    void getLocalTarget();

    /* ROS functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void checkCollisionCallback(const ros::TimerEvent &e);
    void waypointCallback(const geometry_msgs::PoseStampedPtr &msg);
    void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void robopos_cb(const ius_msgs::CirclePosesConstPtr &msg);
    void velVisMsgCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);

    void swarmTrajsCallback(const traj_utils::MultiBsplinesPtr &msg);
    void BroadcastBsplineCallback(const traj_utils::BsplinePtr &msg);

    bool checkCollision();
    void publishSwarmTrajs(bool startup_pub);

  public:
    EGOReplanFSM(/* args */)
    {
    }
    ~EGOReplanFSM()
    {
    }

    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner

#endif