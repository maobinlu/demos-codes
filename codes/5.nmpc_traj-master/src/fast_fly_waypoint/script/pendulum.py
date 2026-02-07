#! /usr/bin/python3.8

import rospy
import numpy as np

from geometry_msgs.msg import Point, Vector3, Quaternion, PoseStamped
from nav_msgs.msg import Path
from airsim_ros.msg import CirclePoses
from robo_msgs.msg import Pendulum
import os, sys
from sensor_msgs.msg import Imu
from collections import deque
import statistics

import time

rospy.init_node("pendulum")

pend_pub = rospy.Publisher("/pendulum_pos", Pendulum, tcp_nodelay=True, queue_size=1)
imu_data = Imu()


ymax_pose = None
time_max = 0
ymin_pose = None
time_min = 0
time_next_ = 0.0
pred_pos = None
flag_y = 0
def imu_cb(msg:Imu):
    global imu_data
    imu_data = msg

queue_t = deque(maxlen=20)
def pend_update_cb(msg:CirclePoses):
    msg.header.stamp = imu_data.header.stamp

    global ymax_pose, time_max, ymin_pose, time_min, time_next_,pred_pos,flag_y
    if ymax_pose is None:
        ymax_pose = msg.poses[16]
        time_max = float(msg.header.stamp.secs) + float(msg.header.stamp.nsecs) / 1e9
    else:
        if msg.poses[16].position.y < ymax_pose.position.y:
            ymax_pose = msg.poses[16]
            time_max = float(msg.header.stamp.secs) + float(msg.header.stamp.nsecs) / 1e9

    if ymin_pose is None:
        ymin_pose = msg.poses[16]
        time_min = float(msg.header.stamp.secs) + float(msg.header.stamp.nsecs) / 1e9
    else:
        if msg.poses[16].position.y > ymin_pose.position.y:
            ymin_pose = msg.poses[16]
            time_min = float(msg.header.stamp.secs) + float(msg.header.stamp.nsecs) / 1e9
    pred_pos = ymax_pose
    if abs(msg.poses[16].position.y - ymax_pose.position.y) < 0.05:
        time_max = float(msg.header.stamp.secs) + float(msg.header.stamp.nsecs) / 1e9
    if abs(msg.poses[16].position.y - ymin_pose.position.y) < 0.05:
        time_min = float(msg.header.stamp.secs) + float(msg.header.stamp.nsecs) / 1e9
    
    time_now = float(msg.header.stamp.secs) + float(msg.header.stamp.nsecs) / 1e9 
    time_cycle = abs(time_max - time_min)
    queue_t.append(time_cycle)
    median_t = statistics.median(queue_t)

    if time_min < time_max:
        time_next_ = 2*median_t - (time_now - time_max)
        pred_pos = ymax_pose
        flag_y = 1
        if time_next_ < 3.0:
            time_next_ = 3*median_t - (time_now - time_max)   
            pred_pos = ymin_pose
            flag_y = -1     
    elif time_min > time_max:
        time_next_ = 2*median_t - (time_now - time_min)
        pred_pos = ymin_pose 
        flag_y = -1   
        if time_next_ < 3.0:
            time_next_ = 3*median_t - (time_now - time_min)   
            pred_pos = ymax_pose
            flag_y = 1 
    Pendulum_pred = Pendulum()
    Pendulum_pred.pos.x = pred_pos.position.x
    Pendulum_pred.pos.y = pred_pos.position.y
    Pendulum_pred.pos.z = pred_pos.position.z
    Pendulum_pred.pred_dt = time_next_
    Pendulum_pred.flag_y = flag_y

    pend_pub.publish(Pendulum_pred)
    # print("--max--")
    # print(ymax_pose)
    # print("---min--")
    # print(ymin_pose)
    # print("---next--")
    # print(flag_y)
    # print(msg.poses[16].position)



rospy.Subscriber("/airsim_node/drone_1/circle_poses_gt", CirclePoses, pend_update_cb, queue_size=1)
rospy.Subscriber("/airsim_node/drone_1/imu/imu", Imu, imu_cb, queue_size=1, tcp_nodelay=True)

rospy.spin()





