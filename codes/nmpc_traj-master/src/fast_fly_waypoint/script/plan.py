#! /usr/bin/python3.8

import rospy
import numpy as np

from fast_fly_waypoint.msg import TrackTraj
from geometry_msgs.msg import Point, Vector3, Quaternion, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from airsim_ros.msg import CirclePoses

import os, sys
BASEPATH = os.path.abspath(__file__).split('script', 1)[0]+'script/fast_fly/'
sys.path += [BASEPATH]

from time_optimal_planner import WayPointOpt, cal_Ns
from gates.gates import Gates
from quadrotor import QuadrotorModel_Traj
from std_msgs.msg import Float64,Bool

import time

rospy.init_node("plan")
rospy.loginfo("ROS: Hello")
traj_pub = rospy.Publisher("~track_traj", TrackTraj, tcp_nodelay=True, queue_size=1)
planned_path_pub = rospy.Publisher("planed_path", Path, queue_size=1)
time_factor_pub = rospy.Publisher("/time_factor", Float64, queue_size=1)
stop_pub = rospy.Publisher("/stop_flag", Bool, queue_size=1)

gate = Gates(BASEPATH+"gates/gates_real.yaml")
quad = QuadrotorModel_Traj(BASEPATH+'quad/quad_real.yaml')
Ns = cal_Ns(gate, 0.4, loop=True)
dts = np.array([0.3]*gate._N)
wp_opt = WayPointOpt(quad, gate._N, Ns, loop=True)
wp_opt.define_opt()
wp_opt.define_opt_t()
res = wp_opt.solve_opt([], np.array(gate._pos).flatten(), dts)

def pub_traj(opt_t_res, opt:WayPointOpt):
    # dt_wp = []

    x = opt_t_res['x'].full().flatten()
    traj = TrackTraj()  
    for i in range(opt._wp_num):
        # dt_w = 0.0
        for j in range(opt._Ns[i]):
            idx = opt._N_wp_base[i]+j
            s = x[idx*opt._X_dim: (idx+1)*opt._X_dim]
            pos = Point()
            pos.x = s[0]
            pos.y = s[1]
            pos.z = s[2]
            vel = Vector3()
            vel.x = s[3]
            vel.y = s[4]
            vel.z = s[5]
            quat = Quaternion()
            quat.w = s[6]
            quat.x = s[7]
            quat.y = s[8]
            quat.z = s[9]
            angular = Vector3()
            angular.x = s[10]
            angular.y = s[11]
            angular.z = s[12]
            dt = x[-opt._wp_num+i]
            # dt_w += dt

            traj.position.append(pos)
            traj.velocity.append(vel)
            traj.orientation.append(quat)
            traj.angular.append(angular)
            traj.dt.append(dt)
        # dt_wp.append(dt_w)
    traj_pub.publish(traj)
    # print("dt_wp",dt_wp)


def pub_path_visualization(opt_t_res, opt:WayPointOpt):
    x = opt_t_res['x'].full().flatten()

    msg = Path()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "world"
    for i in range(opt._Herizon):
        pos = PoseStamped()
        pos.header.frame_id = "world"
        pos.pose.position.y = x[i*opt._X_dim+0]
        pos.pose.position.x = x[i*opt._X_dim+1]
        pos.pose.position.z = -x[i*opt._X_dim+2]

        pos.pose.orientation.w = 1
        pos.pose.orientation.y = 0
        pos.pose.orientation.x = 0
        pos.pose.orientation.z = 0
        msg.poses.append(pos)
    planned_path_pub.publish(msg)

pos_uav = np.array([0,0,0])
def odom_cb(msg: PoseStamped):
    global pos_uav
    pos_uav = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

def extend_point(pos,yaw,distance):
    x_internal = pos[0] + distance*np.cos(yaw*np.pi/180)
    y_internal = pos[1] + distance*np.sin(yaw*np.pi/180)
    return [x_internal, y_internal, pos[2]]

closest_index = 0
gates_get_flag = True
extend_poses = []
def gates_update_cb(msg:CirclePoses):
    global pos_uav,closest_index,gates_get_flag,extend_poses

    if gates_get_flag:
        filtered_poses = msg.poses[:8] + msg.poses[12:]
        extend_poses = []
        distance_internal = 5
        for i in range(len(filtered_poses)):
            if i == 4:
                previous_pos = [100,40.0302,-9.6979]
                extend_poses.append(previous_pos)
            elif i == 0:
                previous_pos = [0.5,0.5,-3.0]
                extend_poses.append(previous_pos)
            elif i == 8:
                distance_internal = 25
                previous_pos = extend_point([filtered_poses[i].position.x, filtered_poses[i].position.y, filtered_poses[i].position.z-1.0], 0.0, distance_internal)
                extend_poses.append(previous_pos)
            elif i == 11:
                distance_internal = -17
                previous_pos = extend_point([filtered_poses[i].position.x, filtered_poses[i].position.y, filtered_poses[i].position.z-1.0], filtered_poses[i].yaw, distance_internal)
                extend_poses.append(previous_pos)
            extend_poses.append([filtered_poses[i].position.x, filtered_poses[i].position.y, filtered_poses[i].position.z])
        gates_get_flag = False

    distances = []
    for g_pos in extend_poses:
        distances.append(np.linalg.norm(pos_uav - np.array([g_pos[0],g_pos[1],g_pos[2]])))
    closest_index = np.argmin(distances)

    time_msg = Float64()
    stop_flag = Bool()
    stop_flag.data = True
    # # if closest_index == (len(extend_poses) -3):
    # #     if pos_uav[1] < extend_poses[closest_index][1] and pos_uav[0] < extend_poses[closest_index][0]:
    # #         time_msg.data = 0.7
    # #     else:
    # #         time_msg.data = 0.5
    # # elif closest_index == (len(extend_poses) -4):
    # #     if pos_uav[1] < extend_poses[closest_index][1]:
    # #         time_msg.data = 0.5
    # #     else:
    # #         time_msg.data = 0.7
    if closest_index == 9:
        if pos_uav[0] < extend_poses[closest_index][0]:
            time_msg.data = 0.62
        else:
            time_msg.data = 0.50     
    # # elif closest_index == 4:
    # #     if pos_uav[0] < extend_poses[closest_index][0]:
    # #         time_msg.data = 0.40
    # #     else:
    # #         time_msg.data = 0.60      
    if closest_index == 8 or closest_index == 6 or closest_index == 7 or closest_index == 5 or closest_index == 2 or closest_index == 3:
        time_msg.data = 0.50
    elif closest_index == len(extend_poses)-1:
        stop_flag.data = True
        stop_pub.publish(stop_flag)
    else:
        time_msg.data = 0.62
    # print(time_msg)
    time_factor_pub.publish(time_msg)

    gates = Gates()

    
    if closest_index < 2:
        for i in range(0,5):
            g_pos = extend_poses[i]
            gates.add_gate([g_pos[0],g_pos[1],g_pos[2]], 0.0)
            init_pos_ = np.array([0.0,0.0,0.0, 0,0,0, 1,0,0,0, 0,0,0])
    else:
        for i in range(closest_index-1,closest_index+4):
            if closest_index < len(extend_poses) - 4:
                g_pos = extend_poses[i]
                gates.add_gate([g_pos[0],g_pos[1],g_pos[2]], 0.0)
                init_pos_ = np.array([extend_poses[closest_index-2][0],extend_poses[closest_index-2][1],extend_poses[closest_index-2][2], 0,0,0, 1,0,0,0, 0,0,0])
            else:
                for k in range(len(extend_poses)- 4,len(extend_poses)-1):
                    g_pos = extend_poses[k]
                    gates.add_gate([g_pos[0],g_pos[1],g_pos[2]], 0.0)
                    if k == len(extend_poses)- 2:
                        gates.add_gate([31.099998474121094, 88.08289337158203, -8.616086006164551], 0.0)
                        gates.add_gate([41.099998474121094, 88.08289337158203, -4.616086006164551], 0.0)
                    init_pos_ = np.array([extend_poses[len(extend_poses)- 5][0],extend_poses[len(extend_poses)- 5][1],extend_poses[len(extend_poses)- 5][2], 0,0,0, 1,0,0,0, 0,0,0])
                break
    res_t = wp_opt.solve_opt_t(init_pos_, np.array(gates._pos).flatten())
    pub_traj(res_t, wp_opt)
    pub_path_visualization(res_t, wp_opt)



rospy.Subscriber("/airsim_node/drone_1/circle_poses_gt", CirclePoses, gates_update_cb, queue_size=1)

rospy.Subscriber("~odom", PoseStamped, odom_cb, queue_size=1, tcp_nodelay=True)

# rospy.Timer(rospy.Duration(0.01), timer_cb)

rospy.spin()
rospy.loginfo("ROS: Byby")