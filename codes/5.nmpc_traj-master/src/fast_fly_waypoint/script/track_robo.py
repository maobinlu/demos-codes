#! /usr/bin/python3.8

import numpy as np
import casadi as ca
import time
import math

# ROS
import rospy
from robo_msgs.msg import Traj_pend,Pendulum

from fast_fly_waypoint.msg import TrackTraj  
from airsim_ros.msg import AngleRateThrottle
from airsim_ros.srv import Takeoff, Land
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped,Pose, TwistStamped, Point, Quaternion
from std_msgs.msg import Float64

# 
import os, sys
BASEPATH = os.path.abspath(__file__).split('script', 1)[0]+'script/fast_fly/'
sys.path += [BASEPATH]

from quadrotor import QuadrotorModel_Traj,QuadrotorSimpleModel
from tracker import TrackerPos,TrackerMPC
from trajectory import Trajectory, StateSave
from gates.gates import Gates

rospy.init_node("track")
rospy.loginfo("ROS: Hello")
class Traj_():
    def __init__(self, traj : Traj_pend):
        poss = []
        yaws = []
        ts = []
        for i, pos in enumerate(traj.pos):
            poss.append([pos.x, pos.y, pos.z])
            yaws.append(traj.yaw[i])
            ts.append(traj.time[i])

        self._poss = np.array(poss)
        self._yaws = np.array(yaws)
        self._N = self._poss.shape[0]
        if self._N < 2:
            return
        dir = self._poss[1 : ] - self._poss[ : -1]
        self._dir_norm = np.linalg.norm(dir, axis = 1)
        self._u_dir = dir/self._dir_norm[:, np.newaxis]
        self._ts = np.array(ts)

    def sample(self, pos, dt, N):
        # calculate t0 and idx0
        t0 = 0
        idx0 = 0
        
        pos = np.array(pos)
        dl = np.linalg.norm(self._poss - pos, axis=1)
        min_idx = np.argmin(dl)
        if min_idx == 0:
            idx0 = min_idx
            d_v = pos - self._poss[0]
            u_dir = self._u_dir[0]
            u_t =  np.dot(d_v, u_dir) / self._dir_norm[0]
            if u_t < 0:
                t0 = self._ts[0]
            else:
                t0 = u_t * (self._ts[1] - self._ts[0]) + self._ts[0]
        else:
            idx0 = min_idx - 1
            d_v = pos - self._poss[idx0]
            u_dir = self._u_dir[idx0]
            u_t = np.dot(d_v, u_dir) / self._dir_norm[idx0]
            if u_t > 1:
                idx0 = idx0 + 1
                if idx0 == self._N - 1:
                    t0 = self._ts[-1]
                else:
                    d_v = pos - self._poss[idx0]
                    u_dir = self._u_dir[idx0]
                    u_t = np.dot(d_v, u_dir) / self._dir_norm[idx0]
                    if u_t < 0:
                        t0 = self._ts[idx0]
                    else:
                        t0 = u_t * (self._ts[idx0 + 1] - self._ts[idx0]) + self._ts[idx0]
            else:
                t0 = u_t * (self._ts[idx0 + 1] - self._ts[idx0]) + self._ts[idx0]

        # sample N points
        ts = np.linspace(t0, t0 + dt * (N - 1), N)
        idx = idx0
        poss = []
        yaws = []
        for t in ts:
            while idx + 1 < self._N and t > self._ts[idx + 1]:
                idx += 1
            if idx == self._N - 1:
                poss.append(self._poss[-1])
                yaws.append(self._yaws[-1])
                continue
            u_dir = self._u_dir[idx]
            u_t = (t - self._ts[idx]) / (self._ts[idx + 1] - self._ts[idx])
            poss.append(self._poss[idx] + u_t * self._dir_norm[idx] * u_dir)
            yaws.append(self._yaws[idx] + u_t * (self._yaws[idx + 1] - self._yaws[idx]))
        
        return np.array(poss), np.array(yaws), ts

traj = Trajectory()
Pendulum_pos = Pendulum()
quad =  QuadrotorModel_Traj(BASEPATH+'quad/quad_real.yaml')

tracker = TrackerPos(quad)

# tracker.define_opt()
tracker.load_so(BASEPATH+"generated/robotracker.so")

run_time_log = time.strftime("%Y-%m-%d_%X", time.localtime())
# state_saver = StateSave(BASEPATH+"results/real_flight"+run_time_log+".csv")

imu_data = Imu()
# takeoff_client = rospy.ServiceProxy("/airsim_node/drone_1/takeoff", Takeoff)
ctrl_pub = rospy.Publisher("/airsim_node/drone_1/angle_rate_throttle_frame", AngleRateThrottle, queue_size=1, tcp_nodelay=True)

stop_flag = False

r_x = []
r_y = []
r_z = []
# last_t = time.time()
cnt = 0
time_factor = 0.6
stop_flag = False
pend_flag = False
# pend_pos_flag = False
def stop_cb(msg: Bool):
    global stop_flag
    stop_flag = msg.data

# 存储上一个时间戳
prev_time = None
prev_pos = None
prev_dt = 0.01
pendulum_pos_ = Pendulum_pos.pos
pendulum_ts_ = Pendulum_pos.pred_dt
pendulum_pos_later = Pendulum_pos.pos
pendulum_dir = Pendulum_pos.flag_y
def odom_cb(msg: PoseStamped):
    global prev_time
    global prev_pos
    global pos_data
    global time_factor
    global pend_flag,uav_pos_,pendulum_pos_,pendulum_ts_,pendulum_pos_later,pendulum_dir
    # cnt += 1
    curr_time = float(imu_data.header.stamp.secs) + float(imu_data.header.stamp.nsecs) / 1e9 
    curr_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    pos_data = curr_pos
    # 如果是第一次回调，则将当前时间戳存储为上一个时间戳
    if prev_time is None:
        prev_time = curr_time
        return
    if prev_pos is None:
        prev_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        return
    # 计算时间间隔 dt
    dt = curr_time - prev_time
    if dt == 0.0:
        dt = prev_dt
    # 计算线性速度
    dx = curr_pos[0] - prev_pos[0]
    dy = curr_pos[1] - prev_pos[1]
    dz = curr_pos[2] - prev_pos[2]
    linear_vel_x = dx / dt
    linear_vel_y = dy / dt
    linear_vel_z = dz / dt
    prev_pos = curr_pos
    prev_time = curr_time

    if traj._N != 0:
        # print("track:", cnt)
        q = np.array([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
        v_b = np.array([linear_vel_x,linear_vel_y,linear_vel_z])
        p = curr_pos
        w = np.array([imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z])
        x0 = np.concatenate([p, v_b, q, w])
        r_x.append(msg.pose.position.x)
        r_y.append(-msg.pose.position.y)
        r_z.append(-msg.pose.position.z)

        if stop_flag:
            time_factor = time_factor*0.5
        else:
            time_factor = time_factor

        print("time_factor",time_factor)

        if time_factor == 0 and np.linalg.norm(v_b) < 0.05 and pend_flag == False:
            uav_pos_ = p
            pend_flag = True
            pendulum_dir = Pendulum_pos.flag_y
            pendulum_pos_.x = Pendulum_pos.pos.x + 0.5
            pendulum_pos_.y = Pendulum_pos.pos.y + 0.11*pendulum_dir
            pendulum_pos_.z = Pendulum_pos.pos.z - 0.33
            pendulum_ts_ = Pendulum_pos.pred_dt - 1.5
            # pendulum_pos_.x = Pendulum_pos.pos.x + 0.5
            # pendulum_pos_.y = Pendulum_pos.pos.y + 0.25*pendulum_dir
            # pendulum_pos_.z = Pendulum_pos.pos.z - 0.33
            # pendulum_ts_ = Pendulum_pos.pred_dt - 1.5
            

            # pendulum_pos_later.x = Pendulum_pos.pos.x + 3.0
            # pendulum_pos_later.y = Pendulum_pos.pos.y
            # pendulum_pos_later.z = Pendulum_pos.pos.z - 0.2      


        if pend_flag:
            quad = QuadrotorSimpleModel(BASEPATH+'quad/quad_real.yaml')

            tracker_pend = TrackerMPC(quad)

            # tracker.define_opt()
            tracker_pend.load_so(BASEPATH+"generated/track_mpc.so")

            x0_pend = np.concatenate([p, v_b, q])
            traj_pend = Traj_pend()
            traj_pend.pos.append(Point(uav_pos_[0],uav_pos_[1],uav_pos_[2]))
            traj_pend.yaw.append(0)
            traj_pend.time.append(0)

            if pendulum_pos_.x == 0.5:
                pendulum_dir = Pendulum_pos.flag_y
                pendulum_pos_.x = Pendulum_pos.pos.x + 0.5
                pendulum_pos_.y = Pendulum_pos.pos.y + 0.11*pendulum_dir
                pendulum_pos_.z = Pendulum_pos.pos.z - 0.33
                pendulum_ts_ = Pendulum_pos.pred_dt - 1.5

            traj_pend.pos.append(pendulum_pos_)
            traj_pend.yaw.append(0)
            traj_pend.time.append(pendulum_ts_)

            poss_pend, yaw_pend, ts_pend = Traj_(traj_pend).sample(p, 0.1, 50)
            print("##############################################")
            print(yaw_pend)
            res_pend = tracker_pend.solve(x0_pend, poss_pend.reshape(-1), yaw_pend.reshape(-1))
            x_pend = res_pend['x'].full().flatten()
            Tt = x_pend[tracker_pend._Herizon*10+0]
            wx = x_pend[tracker_pend._Herizon*10+1]
            wy = x_pend[tracker_pend._Herizon*10+2]
            wz = x_pend[tracker_pend._Herizon*10+3]
            # u.type_mask = AttitudeTarget.IGNORE_ATTITUDE
            u = AngleRateThrottle()
            u.rollRate = wx
            u.pitchRate = wy
            u.yawRate = wz
            u.throttle = -Tt/4.0
            ctrl_pub.publish(u)
        else:
            trjp, trjv, trjdt, ploy = traj.sample(tracker._trj_N, x0[:3])

            res = tracker.solve(x0, ploy.reshape(-1), trjdt, time_factor=time_factor)
            
            x = res['x'].full().flatten()
            Tt = 1*(x[tracker._Herizon*13+0]+x[tracker._Herizon*13+1]+x[tracker._Herizon*13+2]+x[tracker._Herizon*13+3])
            u = AngleRateThrottle()
            u.throttle = Tt/4.0
            u.rollRate = x[10]
            u.pitchRate = x[11]
            u.yawRate = x[12]
            ctrl_pub.publish(u)
        # print("u",u.throttle, u.rollRate, u.pitchRate, u.yawRate)
        print("Pendulum_pos",pendulum_pos_)
        print("pendulum_ts_",pendulum_ts_)
        print("dt",dt)


def imu_cb(msg:Imu):
    global imu_data
    imu_data = msg

def time_factor_cb(msg:Float64):
    global time_factor
    time_factor = msg.data
    print("time_factor",time_factor)

def pendulum_cb(msg:Pendulum):
    global Pendulum_pos
    Pendulum_pos = msg

    
def track_traj_cb(msg: TrackTraj):
    global traj,pos_data,time_factor
    traj_tmp = Trajectory()
    pos = []
    vel = []
    quat = []
    angular = []
    dt = []

    for i in range(len(msg.dt)):
        pos.append([msg.position[i].x, msg.position[i].y, msg.position[i].z])
        vel.append([msg.velocity[i].x, msg.velocity[i].y, msg.velocity[i].z])
        quat.append([msg.orientation[i].w, msg.orientation[i].x, msg.orientation[i].y, msg.orientation[i].z])
        angular.append([msg.angular[i].x, msg.angular[i].y, msg.angular[i].z])
        dt.append(msg.dt[i])


    traj_tmp.load_data(np.array(pos), np.array(vel), np.array(quat), np.array(angular), np.array(dt))
    traj = traj_tmp
    print(traj)

rospy.Subscriber("~odom", PoseStamped, odom_cb, queue_size=1, tcp_nodelay=True)
rospy.Subscriber("/airsim_node/drone_1/imu/imu", Imu, imu_cb, queue_size=1, tcp_nodelay=True)
rospy.Subscriber("~track_traj", TrackTraj, track_traj_cb, queue_size=1, tcp_nodelay=True)
rospy.Subscriber("/time_factor", Float64, time_factor_cb, queue_size=1, tcp_nodelay=True)
rospy.Subscriber("/stop_flag", Bool, stop_cb)
rospy.Subscriber("/pendulum_pos", Pendulum, pendulum_cb, queue_size=1, tcp_nodelay=True)


rospy.spin()
rospy.loginfo("ROS: Goodby")

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 绘制轨迹的三维坐标
ax.plot(traj._pos[:, 0], -traj._pos[:, 1], -traj._pos[:, 2])

# 绘制r_x和r_y的三维坐标
ax.plot(r_x, r_y, r_z)

# 设置坐标轴标签
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.show()