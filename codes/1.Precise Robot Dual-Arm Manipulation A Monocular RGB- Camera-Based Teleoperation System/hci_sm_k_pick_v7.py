'''
Author       : station geyuanliu@gmail.com
Date         : 2025-02-13 22:13:35
LastEditors: 解博炜
LastEditTime: 2025-03-12
FilePath: hci_sm_k_pick_v7.py
Description  : 3DHPE+sw滤波+手势识别
version      : v7
-更新映射 左右圆形变动yam 以root为原点
-pick 引入手掌识别
-v7 基于肘、肩和【无手】2D坐标的自适应滤波算法
-v7 抓取判别优化
notion       : 
'''

#! /usr/bin/env python3
import dis
from math import fabs
from turtle import distance
import mediapipe as mp
import matplotlib.pyplot as plt
from sympy import factor, false, im, true
import torch
import os
import numpy as np
import cv2
from requests import get
from filter import Filter,AdaptiveFilter_ref
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
from utils.hand_open import calculate_hand_openness2,HandStateManager
from utils.show_traj import show_joints_comp_web, show_joints_web,show_trajectory2


matplotlib.use('TkAgg')
os.chdir("/home/aloha-pc/interbotix_ws/src/hcipose_lite")

R = np.array(
    [
        [9.87045566e-01, -1.35264977e-01, -8.62811506e-02],
        [1.34829228e-01, 9.90809061e-01, -1.08850342e-02],
        [8.69605097e-02, -8.89196189e-04, 9.96211363e-01],
    ]
)
T = np.array([-95.3572745, -45.94321624, 27.25388518])
scale = 0.9648176310192673


class HPE_3D:
    def __init__(
        self, dict_path, smoother_weight, filter_weight, joint_num, window_size_s, window_size_f, f, c, resnet
    ):
        if resnet == 50:
            self.input_h = 256
            self.input_w = 256
            self.model = AITEPoseV3(
                dict_path=dict_path,
                smooth_path=smoother_weight,
                resnet=50,
                joint_num=joint_num,
                window_size_f=window_size_f,
                window_size_s=window_size_s,
                input_h=self.input_h,
                input_w=self.input_w,
            )
        elif resnet == 152:
            self.input_h = 384
            self.input_w = 288
            # self.input_h = 288
            # self.input_w = 384
            self.model = AITEPoseV3(
                dict_path=dict_path,
                smooth_path=smoother_weight,
                filter_path=filter_weight,
                resnet=152,
                joint_num=joint_num,
                window_size_f=window_size_f,
                window_size_s=window_size_s,
                input_h=self.input_h,
                input_w=self.input_w,
            )
        self.model_det = YOLO("model_weight/yolov8n.pt", task="detect")
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
        self.filter = AdaptiveFilter_ref(s=16,n=3,d=0,r=0.3,n_ref=1,d_ref=2,max_r=0.3,min_r=0,smoothing_factor=0.6,sensitivity=1/4)
        self.filter2 = AdaptiveFilter_ref(s=16,n=3,d=0,r=0.3,n_ref=1,d_ref=2,max_r=0.3,min_r=0,smoothing_factor=0.6,sensitivity=1/4)
        # self.box_filter = Filter(s=32, n=4, r=1)
        self.hand_point = np.zeros((3, 1))
        self.origin_2d = None
        self.origin_3d = None

        rospy.init_node("hci_node", anonymous=True)
        self.pose_pub = rospy.Publisher("/piper_right/pin_pos_cmd", PosCmd, queue_size=1)
        self.pose_pub2 = rospy.Publisher("/piper_left/pin_pos_cmd", PosCmd, queue_size=1)
        
        BaseOptions = mp.tasks.BaseOptions
        HandLandmarker = mp.tasks.vision.HandLandmarker
        HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
        HandLandmarkerResult = mp.tasks.vision.HandLandmarkerResult
        VisionRunningMode = mp.tasks.vision.RunningMode
        self.left_hand_recognizer = HandLandmarker.create_from_options(
            HandLandmarkerOptions(
                base_options=BaseOptions(model_asset_path='model_weight/hand_landmarker.task'),
                running_mode=VisionRunningMode.LIVE_STREAM,
                result_callback=self.left_hand_callback))
        self.right_hand_recognizer = HandLandmarker.create_from_options(
            HandLandmarkerOptions(
                base_options=BaseOptions(model_asset_path='model_weight/hand_landmarker.task'),
                running_mode=VisionRunningMode.LIVE_STREAM,
                result_callback=self.right_hand_callback))
        # self.recognizer = HandLandmarker.create_from_options(options) 
        self.hand_img = false  
        self.hand_ids = {}
        self.gripper=0
        self.gripper2=0
        self.hand_right_manager = HandStateManager(confidence_threshold=0.8,frame_threshold=8)
        self.hand_left_manager = HandStateManager(confidence_threshold=0.8,frame_threshold=8)
        self.state1= []
        # self.state2 =[] #tem
        self.filte_start = False

    import numpy as np

    def right_hand_callback(self, result, output_image: mp.Image, timestamp_ms: int):
        
        if not result.hand_landmarks:
            return  # 未检测到手
        
        # hand_id = self.hand_ids.get(timestamp_ms)
        hand_landmarks = result.hand_landmarks[0]  # 取第一个检测到的手部
        confidence = result.handedness[0][0].score
        if confidence<=0.9:
            return
        
        # is_open, confidence_score, finger_states=calculate_hand_openness2(hand_landmarks, confidence)
        self.gripper = self.hand_right_manager.update(hand_landmarks, confidence)
        print(self.hand_right_manager.get_status_text())
        # print('右手:')
        # if is_open:
        #     print("手掌张开------")
        #     pick = 0.05  # 例如可以设置一个状态
        # else:
        #     print("手掌合拢-")
        #     pick = 0
        # self.gripper = pick
        
        
    def left_hand_callback(self, result, output_image: mp.Image, timestamp_ms: int):
        
        if not result.hand_landmarks:
            return  # 未检测到手
        
        # hand_id = self.hand_ids.get(timestamp_ms)
        hand_landmarks = result.hand_landmarks[0]  # 取第一个检测到的手部
        confidence = result.handedness[0][0].score
        if confidence<=0.9:
            return
        self.gripper2 = self.hand_left_manager.update(hand_landmarks, confidence)
        print(self.hand_left_manager.get_status_text())

        
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
                results = self.model_det.predict(frame, save=False, imgsz=320,
                                                 classes=0, conf=0.3, verbose=False)
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

            bbox, pre_w, pre_h = bbox_transform(
                bbox.clone(), self.input_h / self.input_w, pre_w, pre_h, self.width, self.height
            )
            coords, scores = self.model.predict(frame_p, bbox)
            if (coords[7, 1] - coords[10, 1]) > (bbox[2] - bbox[0]) * 0.4:
                scores[0, :7, :] = 0
            coords_3d = coords.clone()
            # coords_3d[:, 2] = coords_3d[:, 2] * 2000 + 2000
            # print(coords_3d[16]-coords_3d[0])
            coords_3d[:, 2] = coords_3d[:, 2] - (coords_3d[:, 1] - coords_3d[0, 1]) * (
                (-4.5e-02) / (-2.7204e02)
            )  # (-4.4497e-02)/(-2.7204e+02)
            coords_3d[:, 2] = coords_3d[:, 2] * 2000 + 1700  # 距相机1.7米
            coords_3d = pixel2cam(coords_3d, self.f, self.c)
            coords_root = coords_3d[0]
            coords_3d = coords_3d - coords_root
            coords_3d_hpe = coords_3d[:17, :]
            coords_3d = coords_3d[:17, :]
            coords_3d = self.model.smooth(coords_3d)

            # convert 3d to 2d
            coords_2d = coords_3d + coords_root
            coords_2d = cam2pixel(coords_2d, self.f, self.c)

            if self.origin_2d is None and self.detector == 'anchor':
                self.origin_2d = coords_2d[0] - coords_2d[7] + \
                    coords_2d[14] + (coords_2d[14] - coords_2d[11]) / 2
            frame_show = self.model.vis_keypoints(frame, coords_2d.cpu(), scores.cpu(), kp_thresh=0.0)

            # 绘制坐标系
            if self.detector == 'anchor':
                frame_show = cv2.line(
                    frame_show,
                    (int(self.origin_2d[0]), int(self.origin_2d[1])),
                    (int(self.origin_2d[0]) + 50, int(self.origin_2d[1])),  # X 轴延伸 100 像素
                    (255, 0, 0),  # 蓝色
                    2,
                )
                frame_show = cv2.line(
                    frame_show,
                    (int(self.origin_2d[0]), int(self.origin_2d[1])),
                    (int(self.origin_2d[0]), int(self.origin_2d[1]) - 50),  # Y 轴延伸 100 像素
                    (0, 0, 255),  # 红色
                    2,
                )
                frame_show = cv2.circle(
                    frame_show,
                    (int(self.origin_2d[0]), int(self.origin_2d[1])),  # 圆心坐标
                    5,  # 半径
                    (0, 255, 0),  # 绿色
                    -1,  # 填充圆
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

                # 确定相对坐标原点
                if self.origin_3d is None:
                    self.origin_3d = coords_3d[0]
                    self.origin_3d = self.origin_3d[[2, 0, 1]]
                    self.origin_3d[2] = self.origin_3d[2] * (-1)
                    self.origin_3d[0] = self.origin_3d[0] * (-1)
                    # 原点矫正
                    self.origin_3d2 = coords_3d[0]
                    self.origin_3d2 = self.origin_3d2[[2, 0, 1]]
                    self.origin_3d2[2] = self.origin_3d2[2] * (-1)
                    self.origin_3d2[0] = self.origin_3d2[0] * (-1)

                coords_3d_16 = coords_3d[16]
                coords_3d_filt = coords_3d[16]
                coords_3d_filt2 = coords_3d[13]

                # 坐标系变换
                coords_3d_16 = coords_3d_16[[2, 0, 1]]
                coords_3d_16[2] = coords_3d_16[2] * (-1)
                coords_3d_16[0] = coords_3d_16[0] * (-1)

                coords_3d_filt = coords_3d_filt[[2, 0, 1]]
                coords_3d_filt[2] = coords_3d_filt[2] * (-1)
                coords_3d_filt[0] = coords_3d_filt[0] * (-1)

                coords_3d_hpe = coords_3d_hpe[16][[2, 0, 1]]
                coords_3d_hpe[2] = coords_3d_hpe[2] * (-1)
                coords_3d_hpe[0] = coords_3d_hpe[0] * (-1)

                coords_3d_filt2 = coords_3d_filt2[[2, 0, 1]]
                coords_3d_filt2[2] = coords_3d_filt2[2] * (-1)
                coords_3d_filt2[0] = coords_3d_filt2[0] * (-1)
                

                # 完整处理的数据 xyz变换

                self.hand_point = (coords_3d_filt - self.origin_3d).cpu().numpy()
                self.hand_point2 = (coords_3d_filt2 - self.origin_3d2).cpu().numpy()
                
                self.hand_point_sm = coords_3d_16 - self.origin_3d
                self.templist_smooth.append(np.append(self.hand_point_sm.cpu().numpy(),0))  # 保存平滑后的数据
                self.hand_point_hpe = coords_3d_hpe - self.origin_3d
                self.templist_hpe.append(self.hand_point_hpe.cpu().numpy())  # 原始数据
                self.scorelist.append(scores)
                self.bbox_list.append(bbox)
                if not self.filte_start:
                    self.filter.init_history(self.hand_point)
                    self.filte_start = True
                else:
                    coord_ref = (coords_3d[15,:2]-coords_3d[14,:2]).cpu().numpy()
                    coord_ref2 = (coords_3d[12,:2]-coords_3d[11,:2]).cpu().numpy()
                    self.hand_point,factor = self.filter.update(self.hand_point,coord_ref)
                    self.hand_point2,factor2 = self.filter2.update(self.hand_point2,coord_ref2) 
                    self.templist_kf.append(np.append(self.hand_point,factor))  # 保存滤波后的数据
                    self.templist_kf2.append(np.append(self.hand_point2,factor2))  # 保存滤波后的数据
                # 发布交互数据
                out_hand_pose = [
                    self.hand_point[0],
                    self.hand_point[1],
                    self.hand_point[2],
                ]
                pub_pose = PosCmd()

                out_hand_pose2 = [
                    self.hand_point2[0],
                    self.hand_point2[1],
                    self.hand_point2[2],
                ]
                pub_pose2 = PosCmd()

                # 正映射
                pub_pose.x = out_hand_pose[0]
                pub_pose.y = out_hand_pose[1]
                pub_pose.z = out_hand_pose[2]

                # pub_pose.data = out_hand_pose
                # spatial_mapping
                pub_pose.x = (pub_pose.x / 1000 - 0.3) * 0.3 / 0.4 + 0.25
                pub_pose.y = (pub_pose.y / 1000 + 0.3) * 2
                pub_pose.z = (pub_pose.z / 1000 - 0.25) * 0.48 / 0.3 + 0.36
                pub_pose.x = max(0.1, min(0.45, pub_pose.x))  # 限制范围
                pub_pose.y = max(-0.4, min(0.4, pub_pose.y))
                pub_pose.z = max(0.1, min(0.7, pub_pose.z))
                # self.pose_pub.publish(pub_pose)

                pub_pose.roll = 0
                pub_pose.yaw = 0
                pub_pose.pitch = 1.57
                # 左右球形映射
                # if abs(pub_pose.y) > 0.2:
                pub_pose.yaw = np.arctan2(pub_pose.y, pub_pose.x)

                if pub_pose.z < 0.3:
                    pitch_low = (np.sqrt(pub_pose.x**2+pub_pose.y**2)-0.1)*(-0.8)/0.46+3.14
                    pub_pose.pitch = pitch_low + (pub_pose.z - 0.1) * (1.57 - pitch_low) / (0.3 - 0.1)
                    down_limit = 0.15 * np.cos(pub_pose.pitch)
                    pub_pose.z = max(down_limit, min(0.7, pub_pose.z))
                pub_pose.gripper = self.gripper
                self.pose_pub.publish(pub_pose)

                # 正映射
                pub_pose2.x = out_hand_pose2[0]
                pub_pose2.y = out_hand_pose2[1]
                pub_pose2.z = out_hand_pose2[2]
                pub_pose2.x = (pub_pose2.x / 1000 - 0.3) * 0.3 / 0.4 + 0.25
                pub_pose2.y = (pub_pose2.y / 1000 - 0.3) * 2
                pub_pose2.z = (pub_pose2.z / 1000 - 0.25) * 0.48 / 0.3 + 0.36
                pub_pose2.x = max(0.1, min(0.45, pub_pose2.x))  # 限制范围
                pub_pose2.y = max(-0.4, min(0.4, pub_pose2.y))
                pub_pose2.z = max(0.1, min(0.7, pub_pose2.z))
                # self.pose_pub.publish(pub_pose)

                pub_pose2.roll = 0
                pub_pose2.yaw = 0
                pub_pose2.pitch = 1.57
                # 左右球形映射
                # if abs(pub_pose2.y) > 0.2:
                pub_pose2.yaw = np.arctan2(pub_pose2.y, pub_pose2.x)

                if pub_pose2.z < 0.3:
                    pitch_low2 = (np.sqrt(pub_pose2.x**2+pub_pose2.y**2)-0.1)*(-0.8)/0.46+3.14
                    pub_pose2.pitch = pitch_low2 + (pub_pose2.z - 0.1) * (1.57 - pitch_low2) / (0.3 - 0.1)
                    down_limit2 = 0.15 * np.cos(pub_pose2.pitch)
                    pub_pose2.z = max(down_limit2, min(0.7, pub_pose2.z))
                pub_pose2.gripper = self.gripper2
                self.pose_pub2.publish(pub_pose2)
                # 利用mediapip估计手势
                if i % 1 == 0:
                    coords_2d_hand = coords_2d[16]
                    x11, y11, x21, y21 = (
                        int(coords_2d_hand[0] - 50),
                        int(coords_2d_hand[1] - 50),
                        int(coords_2d_hand[0] + 50),
                        int(coords_2d_hand[1] + 50),
                    )
                    hand_img = frame_p[y11:y21, x11:x21]
                    hand_img = np.array(hand_img, dtype=np.uint8)
                    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=hand_img)
                    # print('x',(coords_3d[16,0]-coords_3d[15,0]).norm().item())
                    # print('y',(coords_3d[16,1]-coords_3d[15,1]).norm().item())
                    # if (coords_3d[16,0]-coords_3d[15,0]).item()>90:
                        # self.hand_img = false
                    self.right_hand_recognizer.detect_async(mp_image, 2*i)  
                    self.hand_img = true
                    
                    coords_2d_hand = coords_2d[13]
                    x1, y1, x2, y2 = (
                        int(coords_2d_hand[0] - 50),
                        int(coords_2d_hand[1] - 50),
                        int(coords_2d_hand[0] + 50),
                        int(coords_2d_hand[1] + 50),
                    )
                    hand_img = frame_p[y1:y2, x1:x2]
                    hand_img = np.array(hand_img, dtype=np.uint8)
                    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=hand_img)
                    # self.hand_ids[2*i+1] = 1
                    self.left_hand_recognizer.detect_async(mp_image, 2*i+1)

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
            if self.hand_img:
                frame_show = cv2.rectangle(
                    frame_show,
                    (int(x1), int(y1)),
                    (int(x2), int(y2)),
                    (0, 255, 0),
                )
                frame_show = cv2.rectangle(
                    frame_show,
                    (int(x11), int(y11)),
                    (int(x21), int(y21)),
                    (0, 255, 0),
                )
            cv2.imshow("frame", frame_show)
            # 查看frame_show的shape
            img_list.append(frame_show)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("exit")
                cv2.destroyAllWindows()
                self.right_hand_recognizer.close()
                self.left_hand_recognizer.close()
                break
            i = i + 1
            
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

f = [640.234928 - 10, 639.647742 - 12]
c = [310.815142, 259.667876 - 20]

from utils.show_traj import show_joints_web
action_name = 'save_data/ntest2'
if __name__ == "__main__":
    hpe = HPE_3D(
        dict_path_152,
        smoother_weight,
        filter_weight,
        joint_num,
        window_size_f=32,
        window_size_s=27,
        f=f,
        c=c,
        resnet=152,
    )
    hpe.infer()
    # show_trajectory2([hpe.templist_hpe, hpe.templist_smooth,
    #                      hpe.templist_kf, hpe.templist_kf2], [1, 2], [100, -1])
    
    show_joints_comp_web([np.array(hpe.templist_smooth),np.array(hpe.templist_kf)],7025)
    # hpe.model.vis_keypoints_3d(hpe.all_joints.cpu(),hpe.all_score.cpu(),0)
