import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import serial_com
from std_msgs.msg import Float32MultiArray
import numpy as np
from ik_cal_mr import ik_robot_mr
from ik_cal_pip import ik_robot_pip
import math


N50 = 51200 * 50 / 3.1415926536 / 2
N51 = 51200 * 51 / 3.1415926536 / 2
N_1 = 51200 * 1 / 3.1415926536 / 2


class Robot:
    def __init__(self):
        rospy.init_node("robot_master", anonymous=True)
        self.N_init = [N51, N50, N50, N51, N50, N_1]  # 减速比
        self.dir_init = [-1, -1, 1, -1, -1, -1]  # 转动方向
        self.rviz_dir = [-1, 1, -1, 1, 1, 1]
        self.joints_init = [0, 0, 0, 0, 0, 0]  # 关节初始位置
        self.motors_init = [0, 0, 0, 0, 0, 0]  # 电机初始为
        self.joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_callback)

        self.isTeaching = False
        self.isOpen = False
        self.isReplay = False
        self.isPlan = False
        self.teach_point_num = 0
        self.point_k = 0
        self.teach_record = 0

        self.msg = JointState()
        self.msg.name = ["joint" + str(i + 1) for i in range(6)]
        self.msg.position = [0] * 6
        self.msg.velocity = [0] * 6
        self.msg.effort = [0] * 6
        self.msg.header.frame_id = ""

        self.now_loc = [0] * 6
        self.now_spd = [0] * 6
        self.cmd_loc = [0] * 6
        self.cmd_spd = [0] * 6
        self.serial_com = serial_com.Serial_com()
        self.ik_cal = ik_robot_pip()
        self.ik_cal_mr = ik_robot_mr()
        self.inital_theta = [0,  math.pi/4, -math.pi/4, 0, 0, 0]
        self.joint_cmd_update(self.inital_theta, [0.1]*6, 0)  # 启动初始位置
        rospy.Subscriber("hand_pose", Float32MultiArray, self.ik_callback, queue_size=1)
        self.T = self.ik_cal.fk_cal([0,  math.pi/4, -math.pi/4, 0, 0, 0])
        # self.ik_cal.work_space2()
        print(self.T)

    def ik_callback(self, data):
        T = self.T.copy()

        # # 固定姿态
        # T[0, 2] = 1
        # T[1, 1] = -1
        # T[2, 0] = 1
        # T[3, 3] = 1

        # 位置跟随 TODO:运动范围的归一化处理
        print(data.data)
        T[0, 3] = (np.around(data.data[0]/1000, 5)-0.3)*0.75+0.25
        T[1, 3] = np.around(data.data[1]/1000, 5)
        T[2, 3] = np.around(data.data[2]/1000, 5)*0.25+0.12
        print(T)

        # mr库逆解算
        ik_ok, thetas = self.ik_cal.ik_cal(T, self.msg.position)
        # ik_ok, thetas = self.ik_cal.ik_cal(T)
        if not ik_ok:
            rospy.loginfo("ik_error")
            return -1
        print(thetas)
        self.joint_cmd_update(thetas, [0.4]*6, 0)

        # mr库逆解算
        # ik_ok, thetas = self.ik_cal.ik_cal(T, self.msg.position)
        # # ik_ok, thetas = self.ik_cal.ik_cal(T)
        # if not ik_ok:
        #     rospy.loginfo("ik_error")
        #     return -1
        # print(thetas)
        # self.joint_cmd_update(thetas, [0.4]*6, 0)

    def __del__(self):
        if rospy.is_shutdown():
            rospy.signal_shutdown("Destructor called")

    def timer_callback(self, event):
        # if self.isOpen:
        self.joint_state_update()

    def joint_state_update(self):
        # print('--------joint_state_update--------')
        self.serial_com.json_input()
        self.feedback_loc = self.serial_com.get_feedback_loc()
        self.feedback_spd = self.serial_com.get_feedback_spd()
        for i in range(6):
            self.now_loc[i] = (
                self.feedback_loc[i] - self.motors_init[i]
            ) / self.N_init[i] * self.dir_init[i] + self.joints_init[i]
            self.msg.velocity[i] = self.feedback_spd[i] / self.N_init[i]
            self.msg.position[i] = self.now_loc[i] * self.rviz_dir[i]
            self.msg.velocity[i] = self.msg.velocity[i] * self.rviz_dir[i]
        self.msg.header.stamp = rospy.Time.now()
        self.joint_pub.publish(self.msg)

        # # print('--------ik_test--------')
        # T = self.ik_cal.fk_cal(np.array([0, math.pi/3, -math.pi/3, 0, 0, 0]))
        # # # T[0][3] = T[0][3] - 0.02
        # T[1][3] = T[1][3] - 0.05
        # # # T[2][3] = T[2][3] + 0.03
        # print('target T')
        # print(np.round(T, 3))
        # ik_ok, theta = self.ik_cal.ik_cal(T)
        # if ik_ok:
        #     print(theta)
        #     self.joint_cmd_update(theta, [0.2]*6, 0)
        # print('execute T')
        # print(np.round(self.ik_cal.fk_cal(theta), 3))
        # theta = [-0.008,  1.047, - 1.047,  1.573, 0, 0]
        # [-0.008,  1.047, -1.047,  1.573,  0.008, -1.573]
        # self.joint_cmd_update(theta, [0.4]*6, 0)
        # print(theta)

        # print(self.msg.position)

    def joint_cmd_update(self, goal_loc, goal_spd, goal_hand_mode):
        for i in range(6):
            self.cmd_loc[i] = int((goal_loc[i] - self.joints_init[i]) * self.N_init[
                i
            ] * self.dir_init[i] + self.motors_init[i])
            self.cmd_spd[i] = int(goal_spd[i] * self.N_init[i] * self.dir_init[i])
        self.serial_com.set_cmd_loc(self.cmd_loc)
        self.serial_com.set_cmd_spd(self.cmd_spd)
        self.serial_com.set_cmd_hand_mode(goal_hand_mode)
        self.serial_com.json_out()
        self.serial_com.json_out()
        self.serial_com.json_out()


if __name__ == "__main__":
    arm = Robot()
    rospy.spin()
