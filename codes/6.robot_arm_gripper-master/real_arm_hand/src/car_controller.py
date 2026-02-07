#!/usr/bin/env python
import rospy
import sys
sys.path.append(r"/home/nuc/arm807_hand_ws/src/robot_arm_gripper/rs_yolo"); 

# from geometry_msgs.msg import PoseStamped
# from enum import Enum
from geometry_msgs.msg import Pose, PoseStamped,Twist
from std_msgs.msg import String
from rs_yolo.msg import Info


class Car_Controller:

    def __init__(self):
        self.arm_status="Init"     #机械臂状态
        self.car_status = 0         #无人车状态
        self.pick_finish=False      #抓取完成状态
        self.place_finish=False     #放置完成状态
        self.pose = Pose()          #目标点位置信息
        self.pub_goal_= rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)#车前往的目标点
        self.pub_grasp_=rospy.Publisher('arm_to_grasp', String, queue_size=1)#机械臂运动
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1, latch=True)#底盘运动控制

        # 设置PID参数
        self.kp = 0.1 
        self.ki = 0.01
        self.kd = 0.001 
        self.prev_error = 0.0 # 上一次的误差
        self.integral = 0.0 # 积分项

    def DroneStatusCB(self,status):
        """
        无人车状态回调函数：
        状态反馈是1时，根据机械臂的状态发布机械臂的移动指令
        机械臂状态变化：Start -> Pick -> Place 
        """
        self.car_status=status.data  

        if self.car_status== 1 and self.arm_status=="Ready":
            print("Start Pick")
            self.arm_status="Pick"
            # 放到InfoCB里判断物体是否在可抓取的范围内，到之后再进行抓取指令的发布
            self.pub_grasp_.publish(self.arm_status)

        if self.car_status== 1 and self.arm_status=="Pick":
            print("Start Place")
            self.arm_status="Place"
            self.pub_grasp_.publish(self.arm_status)

    def ArmFinishCB(self,status):
        """
        机械臂完成状态的回调函数：
        机械臂完成状态：pick_finish -> place_finish -> 无人车回原点
        感觉注释的那段逻辑错了点，但不确定，可以测试的时候看看
        """
        # if status.data=="pick_finish" and not self.pick_finish:
        #     self.pose.position.x=10.0
        #     self.pose.position.y=5.0
        #     self.pubGoal()
        #     self.pick_finish=True
        # elif status.data=="place_finish" and not self.place_finish:
        #     self.pose.position.x=5.0
        #     self.pose.position.y=4.0
        #     self.pubGoal()
        #     self.place_finish=True
        # elif self.pick_finish and self.place_finish:
        #     self.pose.position.x=0.0
        #     self.pose.position.y=0.0
        #     self.pubGoal()
        if  not self.pick_finish:
            self.pose.position.x=10.0
            self.pose.position.y=5.0
            self.pubGoal()
            self.pick_finish=True
        elif status.data=="pick_finish" and self.pick_finish:
            self.pose.position.x=5.0
            self.pose.position.y=4.0
            self.pubGoal()
            self.place_finish=True
        elif status.data=="place_finish" and self.place_finish:
            self.pose.position.x=0.0
            self.pose.position.y=0.0
            self.pubGoal()

    def InfoCB(self, info):
        """
        相机状态的回调函数
        只使用info.x即距离信息
        判断当x不等于0.25时发送指令到小车控制其移动到0.25
        0.25这个距离是大致的估计值，可能需要更改
        """
        if info.classification == "bottle":
            object_position = {
            'x': info.z / 1000 + 0.120000,
            'y': -info.x / 1000,
            'z': -info.y / 1000,
            'classification': info.classification,
            'confidence': info.confidence
            }

            if object_position['x'] != 0.25:
                error = 0.25 - object_position['x']

                # 计算PID控制输出
                self.integral += error
                derivative = error - self.prev_error
                output = self.kp * error + self.ki * self.integral + self.kd * derivative

                # 创建Twist消息，设置线速度
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = output

                # 发布消息到jackal_velocity_controller/cmd_vel话题
                self.pub_cmd_vel.publish(cmd_vel_msg)

                self.prev_error = error

        # 检查object_position的x值是否满足条件后，再将机械臂状态设置为ready状态
        # 这样才可以执行DroneStatusCB中的指令，发布arm_status消息
        self.arm_status=="Ready"
        # self.pub_grasp_.publish(self.arm_status)

    def pubGoal(self):
        """
        发布导航算法的目标点信息
        """
        msg=PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x=self.pose.position.x
        msg.pose.position.y=self.pose.position.y
        msg.pose.position.z=0.0
        msg.pose.orientation.w = 1.0
        self.pub_goal_.publish(msg)


def startNode():
    c = Car_Controller()
    rospy.Subscriber("/planning/state",String, c.DroneStatusCB, queue_size=1)
    rospy.Subscriber("arm_to_finish", String, c.ArmFinishCB, queue_size=1)      #机械臂完成pick或者place后，都会返回到此话题“pick_finish”“place_finish”
    rospy.Subscriber("/detect_result_out", Info, c.InfoCB, queue_size=1)        #相机检测到的物体的类别、坐标、置信度信息，是Info.msg里定义的话题格式

    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('publish_goal')
    startNode()
    print ("Car Controller started") 