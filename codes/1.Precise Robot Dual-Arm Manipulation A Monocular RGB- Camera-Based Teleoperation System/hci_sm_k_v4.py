'''
Author       : station geyuanliu@gmail.com
Date         : 2025-02-13 22:13:35
LastEditors  : station geyuanliu@gmail.com
LastEditTime : 2025-02-13 22:26:20
FilePath     : /hcipose_lite/hci_sm_k_v4.py
Description  : 支持
version      : 更新映射，左右圆形变动yam
notion       : 
'''
#! /usr/bin/env python3
import dis
from turtle import distance
import mediapipe as mp
import matplotlib.pyplot as plt
from sympy import im
import torch
import os
import numpy as np
import cv2
from requests import get
from filter import Filter
from kalman_filter import KalmanFilter
from cam_utils import pixel2cam, bbox_transform, cam2pixel, get_bbox_from_kps
from AITEPose import AITEPose
from AITEPoseV3 import AITEPoseV3
from std_msgs.msg import Float32MultiArray, Bool
import rospy
import pdb
from ultralytics import YOLO
import time
import matplotlib
from piper_msgs.msg import PiperStatusMsg, PosCmd
matplotlib.use('TkAgg')
os.chdir("/home/aloha-pc/interbotix_ws/src/hcipose_lite")

R = np.array([[9.87045566e-01, -1.35264977e-01, -8.62811506e-02],
              [1.34829228e-01,  9.90809061e-01, -1.08850342e-02],
              [8.69605097e-02, -8.89196189e-04,  9.96211363e-01]])
T = np.array([-95.3572745,  -45.94321624,  27.25388518])
scale = 0.9648176310192673

class HPE_3D:
    def __init__(self, dict_path, smoother_weight,filter_weight, joint_num, window_size_s,window_size_f ,f, c, resnet):
        if resnet == 50:
            self.input_h = 256
            self.input_w = 256
            self.model = AITEPoseV3(
                dict_path=dict_path, smooth_path=smoother_weight, resnet=50, joint_num=joint_num, window_size_f=window_size_f,window_size_s=window_size_s, input_h=self.input_h, input_w=self.input_w
            )
        elif resnet == 152:
            self.input_h = 384
            self.input_w = 288
            # self.input_h = 288
            # self.input_w = 384
            self.model = AITEPoseV3(
                dict_path=dict_path, smooth_path=smoother_weight,filter_path=filter_weight, resnet=152, joint_num=joint_num, window_size_f=window_size_f,window_size_s=window_size_s, input_h=self.input_h, input_w=self.input_w
            )
        self.model_det = YOLO("model_weight/yolov8n.pt",task="detect")
        # self.model_det = YOLO("model_weight/yolov8n.engine", task="detect")
        # model_det = YOLO("model_weight/yolov8n.onnx", task="detect")

        self.capture = cv2.VideoCapture(0)  # 使用相机
        if not self.capture.isOpened():
            print("Error: Unable to open video.")
            exit()
        self.width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        # print(self.width, self.height)
        print(self.width, self.height)
        
        self.detector = 'yolo'
        self.detector_stable = 0
        self.anchor_thresh = 1
        self.bbox_anchor = None
        self.f = f
        self.c = c
        self.templist = []
        self.templist_1 = []
        self.templist_2 = []
        self.templist_hpe = []
        self.templist_smooth = []
        self.templist_kf = []
        self.templist_kf2 = []
        self.scorelist = []
        self.bbox_list = []
        self.box_filter = Filter(s=32, n=4, r=1)
        self.hand_point = np.zeros((3, 1))
        self.origin_2d = None
        self.origin_3d = None

        rospy.init_node("hci_node", anonymous=True)
        self.pose_pub = rospy.Publisher("/piper_right/pin_pos_cmd", PosCmd, queue_size=1)
        self.pose_pub2 = rospy.Publisher("/piper_left/pin_pos_cmd", PosCmd, queue_size=1)
        self.hand_gesture_pub = rospy.Publisher("hand_gesture", Bool, queue_size=1)  # 新的发布者


    def infer(self, function=None):
        print("start predict")
        fps = 0
        i = 0
        time_list = []
        img_list = []
    
        while True:
            t1 = time.time()
            ref, frame = self.capture.read()
            if not ref:
                break
            frame_p = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            if self.detector == 'yolo':
                # print('yolo')
                results = self.model_det.predict(frame, save=False, imgsz=320, classes=0, conf=0.3,verbose=False)
                if results[0].boxes.xyxy.numel() == 0:
                    self.start_seq = False
                    self.model.frame_num = 0
                    continue
                else:
                    self.start_seq = True
                bbox = results[0].boxes.xywh[0]  # x,y,w,h
            else:
                # print('anchor')
                bbox = self.bbox_anchor
            if i == 0:
                pre_w = bbox[2]
                pre_h = bbox[3]
                self.box_filter.init_history(bbox)
            bbox = self.box_filter.update(bbox)
            bbox, pre_w, pre_h = bbox_transform(
                bbox.clone(), self.input_h / self.input_w, pre_w, pre_h, self.width, self.height
            )
            coords, scores = self.model.predict(frame_p, bbox)
            if (coords[7, 1] - coords[10, 1]) > (bbox[2] - bbox[0]) * 0.4:
                scores[0, :7, :] = 0
            coords_3d = coords.clone()
            # coords_3d[:, 2] = coords_3d[:, 2] * 2000 + 2000
            # print(coords_3d[16]-coords_3d[0])
            coords_3d[:, 2] = coords_3d[:, 2] - (coords_3d[:,1]-coords_3d[0,1])* ((-4.5e-02)/(-2.7204e+02)) #(-4.4497e-02)/(-2.7204e+02)
            coords_3d[:, 2] = coords_3d[:, 2] * 2000 + 1700 #距相机1.7米
            coords_3d = pixel2cam(coords_3d, self.f, self.c)
            coords_root = coords_3d[0]
            coords_3d = coords_3d - coords_root
            coords_3d_hpe = coords_3d[:17, :]
            coords_3d = coords_3d[:17, :]
            coords_3d = self.model.smooth(coords_3d)

            # convert 3d to 2d
            coords_2d = coords_3d+coords_root
            coords_2d = cam2pixel(coords_2d, self.f, self.c)

            if self.origin_2d is None and self.detector == 'anchor':
                self.origin_2d = coords_2d[0]-coords_2d[7]+coords_2d[14]+(coords_2d[14]-coords_2d[11])/2
            frame_show = self.model.vis_keypoints(frame, coords_2d.cpu(), scores.cpu(), kp_thresh=0.0)

            
            # 绘制坐标系
            if self.detector == 'anchor':
                frame_show = cv2.line(
                    frame_show,
                    (int(self.origin_2d[0]), int(self.origin_2d[1])),
                    (int(self.origin_2d[0]) + 50, int(self.origin_2d[1])),  # X 轴延伸 100 像素
                    (255, 0, 0),  # 蓝色
                    2
                )
                frame_show = cv2.line(
                    frame_show,
                    (int(self.origin_2d[0]), int(self.origin_2d[1])),
                    (int(self.origin_2d[0]), int(self.origin_2d[1])-50),  # Y 轴延伸 100 像素
                    (0, 0, 255),  # 红色
                    2
                )
                frame_show = cv2.circle(
                    frame_show,
                    (int(self.origin_2d[0]), int(self.origin_2d[1])),  # 圆心坐标
                    5,  # 半径
                    (0, 255, 0),  # 绿色
                    -1  # 填充圆
                )
                
            # decetor判断
            if self.detector == 'yolo':  # 如果连续30帧检测到的bbox差值小于阈值，则切换到anchor检测
                if i == 0:
                    bbox_last = bbox
                elif (bbox - bbox_last).abs().mean() < self.anchor_thresh:
                    self.detector_stable = self.detector_stable + 1
                bbox_last = bbox
                if self.detector_stable > 60:
                    self.detector = 'anchor'
                    self.detector_stable = 0
                    self.bbox_anchor = get_bbox_from_kps(coords_2d, self.input_h / self.input_w)
                    
            if self.detector == 'anchor':
                self.all_joints = coords_3d_hpe
                self.all_score = scores
                self.model.frame_num = self.model.frame_num + 1
                
                #确定相对坐标原点
                if self.origin_3d is None:
                    # self.origin_3d = coords_3d[0]-coords_3d[7]+coords_3d[14]+(coords_3d[14]-coords_3d[11])/2
                    self.origin_3d = coords_3d[0]
                    self.origin_3d= self.origin_3d[[2, 0, 1]]
                    self.origin_3d[2] = self.origin_3d[2]*(-1)
                    self.origin_3d[0] = self.origin_3d[0]*(-1)
                    # 原点矫正
                    # self.origin_3d = scale * (self.origin_3d.cpu().numpy() @ R.T) + T
                    # self.origin_3d = torch.tensor(self.origin_3d).to(coords_3d.device)
                    self.origin_3d2 = coords_3d[0]-coords_3d[7]+coords_3d[11]+(coords_3d[11]-coords_3d[14])/2
                    self.origin_3d2= self.origin_3d2[[2, 0, 1]]
                    self.origin_3d2[2] = self.origin_3d2[2]*(-1)
                    self.origin_3d2[0] = self.origin_3d2[0]*(-1)
                    
                
                # coords_3d_16 = coords_3d[16]
                # coords_3d_filt = self.model.filt(coords_3d_16)
                # coords_3d_13 = coords_3d[13]
                # coords_3d_13[0] = coords_3d_13[0]*(-1)                
                # coords_3d_filt2 = self.model.filt2(coords_3d_13) #因为只针对右手进行的训练
                # coords_3d_filt2[0] = coords_3d_filt2[0]*(-1)
                
                coords_3d_16 = coords_3d[16]
                coords_3d_filt = coords_3d[16]
                coords_3d_filt2 = coords_3d[13]
                
                # 坐标系变换
                coords_3d_16 = coords_3d_16[[2, 0, 1]]
                coords_3d_16[2] = coords_3d_16[2]*(-1)
                coords_3d_16[0] = coords_3d_16[0]*(-1)
                
                coords_3d_filt = coords_3d_filt[[2, 0, 1]]
                coords_3d_filt[2] = coords_3d_filt[2]*(-1)
                coords_3d_filt[0] = coords_3d_filt[0]*(-1)
                
                coords_3d_hpe = coords_3d_hpe[16][[2, 0, 1]]
                coords_3d_hpe[2] = coords_3d_hpe[2]*(-1)
                coords_3d_hpe[0] = coords_3d_hpe[0]*(-1)
                
                coords_3d_filt2 = coords_3d_filt2[[2, 0, 1]]
                coords_3d_filt2[2] = coords_3d_filt2[2]*(-1)
                coords_3d_filt2[0] = coords_3d_filt2[0]*(-1)
                
                
                
                # 矫正
                # coords_3d_16 = scale * (coords_3d_16.cpu().numpy() @ R.T) + T
                # coords_3d_16 = torch.tensor(coords_3d_16).to(coords_3d.device)
                # coords_3d_filt = scale * (coords_3d_filt.cpu().numpy() @ R.T) + T
                # coords_3d_filt = torch.tensor(coords_3d_filt).to(coords_3d.device)
                # coords_3d_hpe = scale * (coords_3d_hpe.cpu().numpy() @ R.T) + T
                # coords_3d_hpe = torch.tensor(coords_3d_hpe).to(coords_3d.device)
                
                # coords_3d_correct = coords_3d_16
                
                # 滤波
                # coords_3d_filter = self.model.smooth_filte(coords_3d_correct)  # ----之后只有16关节（手）-----
            
                # 完整处理的数据 xyz变换
                
                self.hand_point = coords_3d_filt -self.origin_3d
                self.templist_kf.append(self.hand_point.cpu().numpy()) # 保存滤波后的数据
                self.hand_point2 = coords_3d_filt2 -self.origin_3d2
                self.templist_kf2.append(self.hand_point2.cpu().numpy()) # 保存滤波后的数据
                self.hand_point_sm= coords_3d_16 -self.origin_3d
                self.templist_smooth.append(self.hand_point_sm.cpu().numpy()) # 保存平滑后的数据
                self.hand_point_hpe = coords_3d_hpe -self.origin_3d
                self.templist_hpe.append(self.hand_point_hpe.cpu().numpy()) # 原始数据
                self.scorelist.append(scores)
                self.bbox_list.append(bbox)

                # 发布交互数据
                out_hand_pose = [self.hand_point[0].cpu().numpy(), self.hand_point[1].cpu().numpy(), self.hand_point[2].cpu().numpy()]
                pub_pose = PosCmd()
                
                out_hand_pose2 = [self.hand_point2[0].cpu().numpy(), self.hand_point2[1].cpu().numpy(), self.hand_point2[2].cpu().numpy()]
                pub_pose2 = PosCmd()
                
                # 正映射
                pub_pose.x = out_hand_pose[0]
                pub_pose.y = out_hand_pose[1]
                pub_pose.z = out_hand_pose[2]

                # pub_pose.data = out_hand_pose
                # spatial_mapping
                pub_pose.x = (pub_pose.x/1000-0.2)/2 +0.325
                pub_pose.y = (pub_pose.y/1000+0.05)*8/3
                pub_pose.z = (pub_pose.z/1000-0.05)*1.2 +0.36
                pub_pose.x = max(0.25, min(0.4, pub_pose.x))  # 限制范围
                pub_pose.y = max(-0.3, min(0.3, pub_pose.y))
                pub_pose.z = max(0.12, min(0.7, pub_pose.z))
                # self.pose_pub.publish(pub_pose)
                
                pub_pose.roll = 0
                pub_pose.yaw = 0
                pub_pose.pitch = 1.57
                #左右球形映射
                # if abs(pub_pose.y) > 0.2:
                pub_pose.yaw = np.arctan2(pub_pose.y,pub_pose.x)

                if pub_pose.z < 0.3:
                    pub_pose.pitch=2.34+(pub_pose.z-0.12)*(1.57-2.34)/(0.3-0.12)
                pub_pose.gripper = 0.07
                self.pose_pub.publish(pub_pose)
                
                #正映射
                pub_pose2.x = out_hand_pose2[0]
                pub_pose2.y = out_hand_pose2[1]
                pub_pose2.z = out_hand_pose2[2]
                pub_pose2.x = (pub_pose2.x/1000-0.2)/2 +0.325
                pub_pose2.y = (pub_pose2.y/1000-0.05)*8/3
                pub_pose2.z = (pub_pose2.z/1000-0.05)*1.2 +0.36
                pub_pose2.x = max(0.25, min(0.4, pub_pose2.x))  # 限制范围
                pub_pose2.y = max(-0.3, min(0.3, pub_pose2.y))
                pub_pose2.z = max(0.12, min(0.7, pub_pose2.z))
                # self.pose_pub.publish(pub_pose)
                
                pub_pose2.roll = 0
                pub_pose2.yaw = 0
                pub_pose2.pitch = 1.57
                #左右球形映射
                # if abs(pub_pose2.y) > 0.2:
                pub_pose2.yaw = np.arctan2(pub_pose2.y,pub_pose2.x)

                if pub_pose2.z < 0.3:
                    pub_pose2.pitch=2.34+(pub_pose2.z-0.12)*(1.57-2.34)/(0.3-0.12)
                pub_pose2.gripper = 0.07
                self.pose_pub2.publish(pub_pose2)
                
            t = time.time() - t1
            time_list.append(time.time())
            fps = 1 / t
            # if fps<30:
            # print('time',i,'fps:',fps)
            frame_show = cv2.putText(
                frame_show,
                "fps=%.2f, t=%.3f ms" % (fps, t * 1000),
                (0, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),  # 红色
                2,
            )
            frame_show = cv2.rectangle(
                frame_show,
                (int(bbox[0]), int(bbox[1])),
                (int(bbox[2]), int(bbox[3])),
                (0, 255, 0),
                2,
            )
            cv2.imshow("frame", frame_show)
            #查看frame_show的shape
            img_list.append(frame_show)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("exit")
                break
            i = i + 1
        # self.capture.release()
        # # 如果文件夹不存在则创建
        
        
        # if not os.path.exists(action_name):
        #     os.makedirs(action_name)
        #     os.makedirs(action_name+'/img')
        # # 保存时间戳
        # for i in range(len(img_list)):
        #     cv2.imwrite(action_name+'/img/'+f'{i}.jpg', img_list[i])
        # np.save(action_name+'/time.npy', time_list, allow_pickle=True)

    def show_trajectory(self, *templist):
        print("show_trajectory")
        fig = plt.figure()
        num_plots = len(templist)

        for i, temp in enumerate(templist):
            ax = fig.add_subplot(2, int(num_plots/2), i + 1, projection="3d")
            n = len(temp)
            hand_points = np.zeros((n, 3))
            for j, coord in enumerate(temp):
                hand_points[j] = coord
            ax.plot(hand_points[:, 0], hand_points[:, 1], hand_points[:, 2], color="blue")
            ax.set_xlabel('X 轴')
            ax.set_ylabel('Y 轴')
            ax.set_zlabel('Z 轴')
            ax.set_title(f'轨迹 {i + 1}')  # 添加标题
            ax.invert_yaxis()
            # 保存列表
            np.save(action_name+f'/hand_trajectory_{i}.npy', hand_points)

        plt.show()
        
    
    def show_trajectory2(self,templist, cat=None, show_num=[0, -1]):
        import math
        fig = plt.figure()
        num_plots = len(templist)
        if cat != None:
            num_plots = num_plots + 1
        len_num = math.ceil(num_plots / 2)
        col_num = math.ceil(num_plots / len_num)
        for i, temp in enumerate(templist):
            ax = fig.add_subplot(len_num, col_num, i + 1, projection="3d")
            n = len(temp)
            hand_points = np.zeros((n, 3))
            for j, coord in enumerate(temp):
                hand_points[j] = coord
            ax.plot(hand_points[:, 0], hand_points[:, 1], hand_points[:, 2], color="blue")
            ax.set_xlabel("X 轴")
            ax.set_ylabel("Y 轴")
            ax.set_zlabel("Z 轴")
            ax.set_title(f"轨迹 {i + 1}")  # 添加标题
            ax.invert_yaxis()
        if cat != None:
            ax = fig.add_subplot(len_num, col_num, num_plots, projection="3d")
            temp1 = templist[cat[0]]
            temp2 = templist[cat[1]]
            n = len(temp1)
            hand_points = np.zeros((n, 3))
            hand_points2 = np.zeros((n, 3))
            for j, coord in enumerate(temp1):
                hand_points[j] = coord
                hand_points2[j] = temp2[j]
            ax.plot(
                hand_points[show_num[0] : show_num[1], 0],
                hand_points[show_num[0] : show_num[1], 1],
                hand_points[show_num[0] : show_num[1], 2],
                color="blue",
            )
            ax.plot(
                hand_points2[show_num[0] : show_num[1], 0],
                hand_points2[show_num[0] : show_num[1], 1],
                hand_points2[show_num[0] : show_num[1], 2],
                color="red",
            )
            ax.set_xlabel("X 轴")
            ax.set_ylabel("Y 轴")
            ax.set_zlabel("Z 轴")
            ax.set_title(f"and")  # 添加标题
            ax.invert_yaxis()
            #设置视角
            # ax.view_init(elev=view[0], azim=view[1])
        # return fig
        plt.show()


# dict_path_50 = "model_weight/best_N-model_48.2892-50-lite.pth"
# dict_path_152 = "model_weight/best_N-model_46.2329-152-lite.pth"
dict_path_50 = 'model_weight/abest_model_50_84.7558-lite.pth'
dict_path_152 = 'model_weight/abest_model_152_80.1034-lite.pth'
# smoother_weight = "model_weight/abest_model_7_45.3471.pth"
filter_weight = "model_weight/amodel_kfnet_10.pth"  

smoother_weight = "model_weight/abest_model_27_45.1142.pth"
# dict_path_50 = 'model_weight/hci_0v1.pth'
# dict_path_152 = 'model_weight/hci_0v1_152.pth'
joint_num = 18
video_name = "ROKI"
video_file = "video"
video_path = os.path.join(video_file, video_name + ".mp4")
output_video_path = os.path.join(video_file, video_name + "_output.mp4")
output_video3D_path = os.path.join(video_file, video_name + "_output3D.mp4")
# f = [451.477162, 453.875205]  # aloha 电脑参数
# c = [272.213870, 294.493310]
# f = [361.459700, 361.475292]  # 外置相机参数
# c = [330.127529, 245.481479]
f = [409.638077, 410.666366]
c = [340.533727, 195.370445]


f = [640.234928-10, 639.647742-12]
c = [310.815142, 259.667876-20]

action_name = 'save_data/ntest2'
if __name__ == "__main__":
    hpe = HPE_3D(dict_path_152, smoother_weight,filter_weight, joint_num, window_size_f=32,window_size_s=27, f=f, c=c, resnet=152)
    hpe.infer()
    hpe.show_trajectory2( [hpe.templist_hpe, hpe.templist_smooth,hpe.templist_kf,hpe.templist_kf2],[1, 2],[100,-1])
    # hpe.model.vis_keypoints_3d(hpe.all_joints.cpu(),hpe.all_score.cpu(),0)
    