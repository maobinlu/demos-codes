import matplotlib.pyplot as plt
import torch
import os
from models.attention_smooth_1v1_lite import as1v1
from models.regression_nf_3d_hci_3vm2_best_lite import RegressFlow3D_hci_3vm2_best_lite
from models.hcipsoe_finnal import RegressFlow3D_cbam
import numpy as np
import cv2
from curses import noecho
from shutil import move
# from sys import orig_argv

from requests import get
from filter import Filter
from cam_utils import pixel2cam, bbox_transform, cam2pixel, get_bbox_from_kps
from AITEPose import AITEPose
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
import rospy
import math
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import torchvision.transforms.functional as F
import torchvision.transforms as transforms
import pdb
from ultralytics import YOLO
import time
import matplotlib
matplotlib.use('TkAgg')

os.chdir("/home/aloha/HCI/src/hcipose_lite")
# /home/aloha/HCI/src/hcipose_lite/hci.py
# from torch2trt import TRTModule
# 加权递推平均滤波


class HPE_3D:
    def __init__(self, dict_path, smoother_weight, joint_num, window_size, f, c, resnet):
        if resnet == 50:
            self.input_h = 256
            self.input_w = 256
            self.model = AITEPose(
                dict_path=dict_path, smooth_path=smoother_weight, resnet=50, joint_num=joint_num, window_size=window_size, input_h=self.input_h, input_w=self.input_w
            )
        elif resnet == 152:
            self.input_h = 384
            self.input_w = 288
            # self.input_h = 288
            # self.input_w = 384
            self.model = AITEPose(
                dict_path=dict_path, smooth_path=smoother_weight, resnet=152, joint_num=joint_num, window_size=window_size, input_h=self.input_h, input_w=self.input_w
            )
        self.model_det = YOLO("model_weight/yolov8n.pt")
        # self.model_det = YOLO("model_weight/yolov8n.engine", task="detect")
        # model_det = YOLO("model_weight/yolov8n.onnx", task="detect")

        self.capture = cv2.VideoCapture(3)  # 使用相机
        if not self.capture.isOpened():
            print("Error: Unable to open video.")
            exit()
        self.width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(self.width, self.height)
        tall0 = time.time()
        # fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # 定义输出视频的编码器
        # self.out = cv2.VideoWriter(output_video_path, fourcc, 30.0, (self.width, self.height))  # 创建输出视频写入对象

        self.detector = 'yolo'
        self.detector_stable = 0
        self.anchor_thresh = 1
        self.bbox_anchor = None
        self.f = f
        self.c = c
        self.templist = []
        self.scorelist = []
        self.bbox_list = []
        self.box_filter = Filter(s=32, n=4, r=1)
        self.pose_filter = Filter(s=64, n=17, d=3, r=1)
        self.hand_point = np.zeros((3, 1))
        self.origin_2d = None
        self.origin_3d = None

        rospy.init_node("hci_node", anonymous=True)
        self.img_pub = rospy.Publisher("cam_img", Image, queue_size=1)
        self.pose_pub = rospy.Publisher("hand_pose", Float32MultiArray, queue_size=1)

    def infer(self, function=None):
        print("start predict")
        fps = 0
        i = 0
        while True:
            t1 = time.time()
            ref, frame = self.capture.read()
            if not ref:
                break
            frame_p = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            if self.detector == 'yolo':
                # print('yolo')
                results = self.model_det.predict(frame, save=False, imgsz=320, classes=0, conf=0.3)
                if results[0].boxes.xyxy.numel() == 0:
                    self.start_seq = False
                    self.frame_num = 0
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
            coords_3d[:, 2] = coords_3d[:, 2] * 2000 + 2000
            # coords_3d[:, 0] = coords[:, 0] * (2000 / (bbox[2] - bbox[0]))
            # coords_3d[:, 1] = coords[:, 1] * (2000 / (bbox[3] - bbox[1]))
            coords_3d = pixel2cam(coords_3d, self.f, self.c)
            coords_root = coords_3d[0]
            coords_3d = coords_3d - coords_root
            # coords_3d = self.model.smooth(coords_3d[:17, :])  # ----至此之后是17关节-----
            coords_3d = coords_3d[:17, :]
            if i == 0:
                self.pose_filter.init_history(coords_3d)
            coords_3d = self.pose_filter.update(coords_3d)

            # convert 3d to 2d
            coords_2d = coords_3d+coords_root
            coords_2d = cam2pixel(coords_2d, self.f, self.c)

            if self.origin_2d is None and self.detector == 'anchor':
                self.origin_2d = coords_2d[0]-coords_2d[7]+coords_2d[14]+(coords_2d[14]-coords_2d[11])/2
            frame_show = self.model.vis_keypoints(frame, coords_2d.cpu(), scores.cpu(), kp_thresh=0.0)
            t = time.time() - t1
            fps = 1 / t
            frame_show = cv2.putText(
                frame_show,
                "fps=%.2f, t=%.3f ms" % (fps, t * 1000),
                (0, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
            )
            frame_show = cv2.rectangle(
                frame_show,
                (int(bbox[0]), int(bbox[1])),
                (int(bbox[2]), int(bbox[3])),
                (0, 255, 0),
                2,
            )

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
            # self.out.write(frame_show)
            cv2.imshow("frame", frame_show)
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
                if i % 30 == 0:  # 每30帧检测一次yolo的结果是否在anchor范围内
                    results = self.model_det.predict(frame, save=False, imgsz=320, classes=0, conf=0.3)
                    if results[0].boxes.xyxy.numel() == 0:
                        self.start_seq = False
                        self.frame_num = 0
                        # break
                    else:
                        self.start_seq = True
                    bbox_y = results[0].boxes.xywh[0]
                    bbox_y, pre_w, pre_h = bbox_transform(
                        bbox_y.clone(), self.input_h / self.input_w, pre_w, pre_h, self.width, self.height
                    )
                    move_x = 0
                    move_y = 0
                    if bbox_y[0] > bbox[0] and bbox_y[1] > bbox[1] and bbox_y[2] < bbox[2] and bbox_y[3] < bbox[3]:
                        pass
                    else:
                        break
                        if bbox_y[0] < bbox[0]:
                            move_x = bbox_y[0]-bbox[0]+move_x
                        if bbox_y[2] > bbox[2]:
                            move_x = bbox_y[2]-bbox[2]+move_x
                        if bbox_y[1] < bbox[1]:
                            move_y = bbox_y[1]-bbox[1]+move_y
                        if bbox_y[3] > bbox[3]:
                            move_y = bbox_y[3]-bbox[3]+move_y
                        self.bbox_anchor = self.bbox_anchor + torch.tensor([move_x, move_y, move_x, move_y])
            if self.detector == 'anchor':
                if self.origin_3d is None:
                    self.origin_3d = coords_3d[0]-coords_3d[7]+coords_3d[14]+(coords_3d[14]-coords_3d[11])/2
                self.hand_point = coords_3d[16] - self.origin_3d
                self.hand_point = self.hand_point[[2, 0, 1]]
                self.hand_point[2] = self.hand_point[2]*(-1)
                self.hand_point[0] = self.hand_point[0]*(-1)
                self.templist.append(self.hand_point)
                self.scorelist.append(scores)
                self.bbox_list.append(bbox)
                pub_img = CvBridge().cv2_to_imgmsg(frame_show, "bgr8")
                self.img_pub.publish(pub_img)
                out_hand_pose = [self.hand_point[0], self.hand_point[1], self.hand_point[2]]
                pub_pose = Float32MultiArray()
                pub_pose.data = out_hand_pose
                self.pose_pub.publish(pub_pose)
            # if function is not None:
            #     function()
            if cv2.waitKey(1) & 0xFF == ord("q"):
                # break
                fig = plt.figure()
                self.ax = fig.add_subplot(111, projection="3d")
                n = len(self.templist)
                hand_points = np.zeros((n, 3))
                for j, coord in enumerate(self.templist):
                    hand_points[j] = coord.cpu().numpy()
                self.ax.plot(hand_points[:, 0], hand_points[:, 1], hand_points[:, 2], color="blue")
                self.ax.set_xlabel('X 轴')
                self.ax.set_ylabel('Y 轴')
                self.ax.set_zlabel('Z 轴')
                self.ax.invert_yaxis()
                plt.savefig('hand_points_plot.png')
                plt.show()
                # print(hand_points)
                print('hand_points_plot.png')
                break
            i = i + 1
        self.capture.release()


# dict_path_50 = "model_weight/best_N-model_48.2892-50-lite.pth"
# dict_path_152 = "model_weight/best_N-model_46.2329-152-lite.pth"
dict_path_50 = 'model_weight/abest_model_50_84.7558-lite.pth'
dict_path_152 = 'model_weight/abest_model_152_80.1034-lite.pth'
smoother_weight = "model_weight/abest_model_7_45.3471.pth"
# dict_path_50 = 'model_weight/hci_0v1.pth'
# dict_path_152 = 'model_weight/hci_0v1_152.pth'
joint_num = 18
video_name = "ROKI"
video_file = "video"
video_path = os.path.join(video_file, video_name + ".mp4")
output_video_path = os.path.join(video_file, video_name + "_output.mp4")
output_video3D_path = os.path.join(video_file, video_name + "_output3D.mp4")
f = [451.477162, 453.875205]  # aloha 电脑参数
c = [272.213870, 294.493310]
if __name__ == "__main__":
    hpe = HPE_3D(dict_path_152, smoother_weight, joint_num, 7, f, c, resnet=152)
    hpe.infer()
