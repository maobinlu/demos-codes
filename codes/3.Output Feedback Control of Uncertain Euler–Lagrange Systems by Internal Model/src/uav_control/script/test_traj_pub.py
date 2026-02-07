#! /usr/bin/python3.8

import numpy as np
import time

# ROS
import csv
import rospy

from ius_msgs.msg import Trajectory
from geometry_msgs.msg import Point
#
import os, sys

rospy.init_node("test_traj_pub")

BASEPATH = os.path.abspath(__file__).split('script', 1)[0]+'script/function_model/ref_traj/'
sys.path += [BASEPATH]

trajectory_pub = rospy.Publisher("/ius_uav/trajectory", Trajectory, tcp_nodelay=True, queue_size=1)

def traj_pub():
    traj = Trajectory()

    traj.pos.append(Point(0, 0, 0))
    traj.yaw.append(0)
    traj.time.append(0)
    
    traj.pos.append(Point(0, 0, 5))
    traj.yaw.append(1)
    traj.time.append(5)
    
    traj.pos.append(Point(1, 0, 5))
    traj.yaw.append(2)
    traj.time.append(6)
    
    traj.pos.append(Point(2, 0, 5))
    traj.yaw.append(3)
    traj.time.append(7)
    
    trajectory_pub.publish(traj)
   
if __name__ == "__main__":
    
    # timer
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        traj_pub()
        rate.sleep()