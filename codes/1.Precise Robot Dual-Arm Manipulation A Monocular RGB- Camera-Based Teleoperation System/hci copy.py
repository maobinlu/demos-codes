import threading
import math
import numpy as np
from infer_vio_3d import Infer
import modern_robotics as mr
from mr_urdf_loader import loadURDF
from urdfpy import URDF
import arm.serial_com as serial_com
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import rospy
import keyboard


N50 = 51200 * 50 / 3.1415926536 / 2
N51 = 51200 * 51 / 3.1415926536 / 2
N_1 = 51200 * 1 / 3.1415926536 / 2


class Robot:
    def __init__(self):
        rospy.init_node('robot_master')
        self.N_init = [N51, N50, N50, N51, N50, N_1]  # 减速比
        self.dir_init = [-1, -1, 1, -1, -1, -1]       # 转动方向
        self.rviz_dir = [-1, 1, -1, 1, 1, 1]
        self.joints_init = [0, 0, 0, 0, 0, 0]       # 关节初始位置
        self.motors_init = [0, 0, 0, 0, 0, 0]       # 电机初始为
        self.feedback_loc = [0] * 6
        self.feedback_spd = [0] * 6
        self.cmd_loc = [0] * 6
        self.cmd_spd = [0] * 6

        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_callback)
        self.isTeaching = False
        self.isOpen = False
        self.isReplay = False
        self.isPlan = False
        self.teach_point_num = 0
        self.point_k = 0
        self.teach_record = 0

        self.msg = JointState()
        self.msg.name = ['joint' + str(i + 1) for i in range(6)]
        self.msg.position = [0] * 6
        self.msg.velocity = [0] * 6
        self.msg.effort = [0] * 6
        self.msg.header.frame_id = ""
        self.now_loc = [0] * 6
        self.now_spd = [0] * 6
        self.serial_com = serial_com.Serial_com()

        urdf_name = "./arm/urdf/arm.urdf"
        self.MR = loadURDF(urdf_name)
        self.M = np.around(self.MR["M"], 4)
        self.Slist = np.around(self.MR["Slist"], 4)
        self.rev = np.array([[-math.pi, math.pi],
                             [0, math.pi],
                             [-math.pi, 0],
                             [-math.pi, math.pi],
                             [-math.pi/2, math.pi/2],
                             [-math.pi, math.pi]])
        self.M = np.array([[1.,      0.,      0.,      0.2154],
                           [0.,     -1.,      0.,      0.],
                           [0.,     -0.,     -1.,     -0.1365],
                           [0.,      0.,      0.,      1.]])
        self.Slist = np.array([[0.,      0.,      1.,     -0.,     -0.,     -0.,],
                               [0.,      1.,      0.,     -0.,     -0.,      0.03],
                               [0.,      1.,      0.,     -0.,     -0.,      0.2154],
                               [1.,     0.,      0.,      0.,     -0.1365, -0.],
                               [0.,      1.,      0.,      0.1365, -0.,      0.2154],
                               [1.,      0.,      0.,      0.,     -0.1365, -0.]])
        print("M\n", self.M)
        print("Slist\n", self.Slist)

        dict_path_152 = "hci_0v1_152.pth"
        joint_num = 22
        f = [451.477162, 453.875205]  # aloha 电脑参数
        c = [272.213870, 294.493310]
        self.hpe = Infer(dict_path_152, joint_num, f, c)

    def infer(self):
        self.hpe.infer()

    def IK_2_real(self, thetalist):
        thetalist[2] = thetalist[2]+np.deg2rad(162.24)
        thetalist[3] = thetalist[3]-np.deg2rad(72.24)

    def real_2_IK(self, thetalist):
        thetalist[2] = thetalist[2]-np.deg2rad(162.24)
        thetalist[3] = thetalist[3]+np.deg2rad(72.24)

    def hci(self):
        while True:
            # if keyboard.is_pressed('q'):
            #     break
            T_sd = np.array([[1, 0, 0, -0.0180],
                             [0, -1, 0, -0.008],
                             [0,  0, -1, -0.1223],
                             [0,  0,  0,  1]])
            # [[ 0.44401584 -0.85331698  0.27331312  0.00181435]
            # [ 0.63358107  0.51469292  0.5776385   0.83623057]
            # [-0.63358107 -0.08331462  0.76917729  0.05876656]
            # [ 0.          0.          0.          1.        ]]
            # T_sd[:3, 3] = T_sd[:3, 3]+np.array([0, 0, 0.05])
            # [向下，向左，向前]
            T_sd[:3, 3] = T_sd[:3, 3]+np.array([-self.hpe.hand_point[1], self.hpe.hand_point[0], -self.hpe.hand_point[2]]).flatten()/20000
            # T = np.around(np.array([[1, 0, 0, -1.79600459e-02],
            #               [0, -1, 0, -8.01627102e-03],
            #               [0,  0, -1, -1.22345832e-01],
            #               [0,  0,  0,  1.00000000e+00]]), 4)
            thetalist, ik_flag = mr.IKinSpace(self.Slist, self.M, T_sd, np.array([0, 0, 0, 0, 0, 0]), 0.001, 0.001)
            # thetalist, ik_flag = mr.IKinSpace(self.Slist, self.M, T_sd, np.around(np.array(self.msg.position), 4), 0.001, 0.001)
            # thetalist = np.array([0, math.pi/4, -math.pi/4, 0, math.pi/4, 0])
            # if ik_flag:
            #     for x in range(len(thetalist)):
            #         if thetalist[x] <= -self.rev[x][0]:
            #             thetalist[x] %= -self.rev[x][0]
            #         elif thetalist[x] >= self.rev[x][1]:
            #             thetalist[x] %= self.rev[x][1]
            #     solution_found = True
            # else:
            #     solution_found = False
            thetalist[2] = thetalist[2]+np.deg2rad(360-197.76)
            print(ik_flag)
            if ik_flag:
                self.joint_cmd_update(np.around(thetalist, 3), [0.2]*6, 0)

    def __del__(self):
        if rospy.is_shutdown():
            rospy.signal_shutdown('Destructor called')

    def timer_callback(self, event):
        if self.isOpen:
            self.joint_state_update()

    def joint_state_update(self):
        while True:
            self.serial_com.json_input()
            self.feedback_loc = self.serial_com.get_feedback_loc()
            self.feedback_spd = self.serial_com.get_feedback_spd()
            for i in range(6):
                self.now_loc[i] = (self.feedback_loc[i] - self.motors_init[i]) / self.N_init[i] * self.dir_init[i] + self.joints_init[i]
                self.msg.velocity[i] = self.feedback_spd[i] / self.N_init[i]
                self.msg.position[i] = self.now_loc[i] * self.rviz_dir[i]
                self.msg.velocity[i] = self.msg.velocity[i] * self.rviz_dir[i]
            self.msg.header.stamp = rospy.Time.now()
            self.joint_pub.publish(self.msg)
            # print(self.msg.position)
            rospy.sleep(0.01)
            # print(self.msg.position)

    def joint_cmd_update(self, goal_loc, goal_spd, goal_hand_mode):
        for i in range(6):
            self.cmd_loc[i] = int((goal_loc[i] - self.joints_init[i]) * self.N_init[i] * self.dir_init[i] + self.motors_init[i])
            self.cmd_spd[i] = int(goal_spd[i] * self.N_init[i] * self.dir_init[i])
        self.serial_com.set_cmd_loc(self.cmd_loc)
        self.serial_com.set_cmd_spd(self.cmd_spd)
        self.serial_com.set_cmd_hand_mode(goal_hand_mode)
        self.serial_com.json_out()
        self.serial_com.json_out()
        self.serial_com.json_out()

    # Calculates Rotation Matrix given euler angles.


def eulerAnglesToRotationMatrix(theta):

    R_x = np.array([[1,         0,                  0],
                    [0,         math.cos(theta[0]), -math.sin(theta[0])],
                    [0,         math.sin(theta[0]), math.cos(theta[0])]
                    ])

    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])],
                    [0,                     1,      0],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])]
                    ])

    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])

    R = np.dot(R_z, np.dot(R_y, R_x))

    return R


robot = Robot()
# 创建线程
thread1 = threading.Thread(target=robot.infer)
thread2 = threading.Thread(target=robot.hci)
thread3 = threading.Thread(target=robot.joint_state_update)

# 启动线程
thread1.start()
thread2.start()
thread3.start()

# # 等待所有线程完成
# thread1.join()
# thread2.join()
# thread3.join()
