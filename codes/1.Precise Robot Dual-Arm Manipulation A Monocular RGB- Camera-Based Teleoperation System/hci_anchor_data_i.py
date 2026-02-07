from onnx import save
import pandas as pd
from motion_capture import mc_node
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
from kalman_filter import KalmanFilter
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


class HPE_3D:
    def __init__(self, dict_path, smoother_weight, joint_num, window_size, f, c, resnet, name='test'):
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
        self.mc = mc_node("/vrpn_client_node/hand3/pose")
        self.mc_thox = mc_node("/vrpn_client_node/thox/pose")

        self.capture = cv2.VideoCapture(0)  # 使用相机
        if not self.capture.isOpened():
            print("Error: Unable to open video.")
            exit()
        self.width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(self.width, self.height)
        tall0 = time.time()

        self.detector = 'yolo'
        self.detector_stable = 0
        self.anchor_thresh = 1
        self.bbox_anchor = None
        self.f = f
        self.c = c
        self.hand_point = np.zeros((3, 1))
        self.origin_2d = None
        self.origin_3d = None

        # rospy.init_node("hci_node", anonymous=True)
        self.save_hpe = []
        self.save_mc = []
        self.save_img = []
        self.mc_maker_list = []
        self.action_name = name

    def infer(self, function=None):
        if os.path.exists('dataset/'+self.action_name):
            print('data repeat')
            return -1
        print("start predict")
        fps = 0
        i = 0
        collect_n = 0
        data_stable = 0
        hpe_data_last = 0
        mc_data_last = 0
        start_collect = False
        hpe_root_list = np.zeros((30, 3))
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
            bbox, pre_w, pre_h = bbox_transform(
                bbox.clone(), self.input_h / self.input_w, pre_w, pre_h, self.width, self.height
            )
            coords, scores = self.model.predict(frame_p, bbox)
            # print('img', coords[16])
            coords_img = coords
            coords_3d = coords.clone()
            coords_2d = coords[:17, :]
            coords_3d[:, 2] = coords_3d[:, 2] * 2000 + 1700
            # coords_3d[:, 2] = coords_3d[:, 2] * 1300 + 1261
            # print('to cam', coords_3d[16])
            coords_3d = pixel2cam(coords_3d, self.f, self.c)
            coords_root = coords_3d[0]
            coords_o = coords_3d.clone()
            coords_3d = coords_3d - coords_root
            # print('3d', coords_3d[16])
            coords_3d = coords_3d[:17, :]

            # convert 3d to 2d
            # coords_2d = coords_3d+coords_root
            # coords_2d = cam2pixel(coords_2d, self.f, self.c)

            if self.origin_2d is None and self.detector == 'anchor':
                self.origin_2d = coords_2d[0]-coords_2d[7]+coords_2d[14]+(coords_2d[14]-coords_2d[11])/2
            frame_show = self.model.vis_keypoints(frame, coords_2d.cpu(), scores.cpu(), kp_thresh=0.0)
            t = time.time() - t1
            fps = 1 / t
            if start_collect:
                collect_n = collect_n+1
            frame_show = cv2.putText(
                frame_show,
                "fps=%.2f, t=%.3f ms, n=%d" % (fps, t * 1000, collect_n),
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
            if collect_n > 2000:
                print('collect 500 data')
                break
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
            cv2.imshow("frame", frame_show)
            # 获取焦距参数
            # focus = self.capture.get(cv2.CAP_PROP_FOCUS)
            # print(f"Current focus: {focus}")

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
                        print('no human')
                        break
                    else:
                        self.start_seq = True
                    bbox_y = results[0].boxes.xywh[0]
                    bbox_y, pre_w, pre_h = bbox_transform(
                        bbox_y.clone(), self.input_h / self.input_w, pre_w, pre_h, self.width, self.height
                    )
                    if bbox_y[0] > bbox[0] and bbox_y[1] > bbox[1] and bbox_y[2] < bbox[2] and bbox_y[3] < bbox[3]:
                        pass
                    else:
                        print('out bbox')
                        break

                if not self.mc.get_maker and data_stable < 30:
                    data_stable = 0
                    rospy.loginfo(f"marker loss, out stable")
                    # continue

                coords_3d = coords_3d.cpu().numpy()
                if not start_collect:
                    if data_stable < 30:
                        print('hpe:', np.abs(coords_3d[16] - hpe_data_last))
                        print('mc', np.abs(self.mc.current_data - mc_data_last))
                        if np.mean(np.abs(coords_3d[16] - hpe_data_last)) < 15 and np.mean(np.abs(self.mc.current_data - mc_data_last)) < 5:
                            hpe_root_list[data_stable] = coords_3d[16]
                            data_stable = data_stable + 1
                        else:
                            data_stable = 0
                        hpe_data_last = coords_3d[16]
                        mc_data_last = self.mc.current_data
                        print(f'wait stable--{data_stable}')
                        print('------------^^------------')
                        # continue
                    else:
                        start_collect = True
                        root_hpe = np.mean(hpe_root_list, axis=0)
                        root_mc = mc_data_last
                else:
                    save_data_img = coords_img[16]

                    # save_data_hpe = coords_3d[16] - root_hpe
                    save_data_hpe = coords_3d[16]
                    # save_data_hpe = coords_3d[7]  # 用来测试root偏差
                    save_data_hpe = save_data_hpe[[2, 0, 1]]
                    save_data_hpe[2] = save_data_hpe[2]*(-1)
                    save_data_hpe[0] = save_data_hpe[0]*(-1)

                    # save_data_mc = self.mc.current_data - self.mc_thox.current_data
                    # save_data_mc = self.mc.current_data - root_mc
                    # save_data_mc = self.mc_thox.current_data  # 用来测试root偏差
                    # save_data_mc = self.mc.current_data + np.array([1751.5, 77, -889])  # xbw
                    # save_data_mc = self.mc.current_data + np.array([1721, 77, -950])  # lyl
                    save_data_mc = self.mc.current_data + np.array([1719, 70, -898])
                    # save_data_mc[1] = save_data_mc[1]-100  # 弥补身体差
                    save_img = self.model.img
                    self.save_hpe.append(save_data_hpe)
                    self.save_mc.append(save_data_mc)
                    self.mc_maker_list.append(self.mc.get_maker)
                    self.save_img.append(save_img)

                    # # ----------debug------------
                    # coords_3d_d = coords_3d[16][[2, 0, 1]]
                    # coords_3d_d[2] = coords_3d_d[2]*(-1)
                    # coords_3d_d[0] = coords_3d_d[0]*(-1)
                    # print('hpe', coords_3d_d)
                    # print('hpe_img', coords_img[16], 'hpe_root', coords_img[0])
                    # print('hpe_o', coords_o[16], 'hpe_root_o', coords_o[0])

                    # data_mc = (self.mc.current_data-root_mc)[[1, 0, 2]]
                    # data_mc[0] = data_mc[0]*(-1)
                    # root_hpe_d = root_hpe[[2, 0, 1]]
                    # root_hpe_d[2] = root_hpe_d[2]*(-1)
                    # root_hpe_d[0] = root_hpe_d[0]*(-1)
                    # print('mc_d', data_mc+root_hpe_d)
                    # print('-----------^^^^-------------')
            # time.sleep(1 / 10)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("exit")
                break
            i = i + 1

        self.capture.release()
        self.save_data('dataset/'+self.action_name)

    def save_data(self, action_name="action"):
        # precess mc data
        data_mc_np = np.array(self.save_mc)
        data_compensation = self.mc.loss_compensation(data_mc_np, mask=self.mc_maker_list)
        # pdb.set_trace()
        if len(data_compensation) != len(self.save_mc):
            print("data loss! save data fail")
            return

        if not os.path.exists(action_name):
            os.makedirs(action_name)
            os.makedirs(action_name+'/img')

        # 保存时间戳
        for i in range(len(self.save_img)):
            cv2.imwrite(action_name+'/img/'+f'{i}.jpg', cv2.cvtColor(self.save_img[i], cv2.COLOR_BGR2RGB))
        np.save(action_name+"/save_hpe.npy", np.array(self.save_hpe), allow_pickle=True)
        np.save(action_name+"/save_mc.npy", np.array(self.save_mc), allow_pickle=True)
        print("save data success")

    def show_trajectory(self, *templist):
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
        plt.show()


# dict_path_50 = "model_weight/best_N-model_48.2892-50-lite.pth"
# dict_path_152 = "model_weight/best_N-model_46.2329-152-lite.pth"
dict_path_50 = 'model_weight/abest_model_50_84.7558-lite.pth'
dict_path_152 = 'model_weight/abest_model_152_80.1034-lite.pth'
smoother_weight = "model_weight/abest_model_7_45.3471.pth"
# smoother_weight = "model_weight/abest_model_27_45.1142.pth"
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
# f = [337.954931, 334.976569]
# c = [318.347677, 247.342830]

# f = [409.638077, 410.666366]
# f = [654.149079, 650.525797]

# c = [340.533727, 195.370445]
# f = [700.149079, 700.525797]

# f = [728.790107, 712.030997]
# c = [326.391039, 292.631964]

# f = [665.411075, 656.919784]
# c = [324.621320, 278.284158]

# f = [381.9617, 219.7383] #逆解算
# c = [456.8753, 279.79]

f = [640.234928-10, 639.647742-12]
c = [310.815142, 259.667876-20]


if __name__ == "__main__":
    hpe = HPE_3D(dict_path_152, smoother_weight, joint_num, 7, f, c, resnet=152, name='lg5')
    hpe.infer()
    hpe.show_trajectory(hpe.save_mc, hpe.save_hpe)
    # data_hep = np.load('dataset/test5/save_hpe.npy')
    # data_mc = np.load('dataset/test5/save_mc.npy')
    # data_mc = data_mc[:, [1, 0, 2]]
    # data_mc[:, 0] = data_mc[:, 0]*(-1)
    # hpe.show_trajectory(data_hep, data_mc)
