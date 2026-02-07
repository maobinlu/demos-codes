from tkinter import NO
from sympy import N
import torch
import os
from models.attention_smooth_1v1_lite import as1v1
# from models.smooth_correct_1v2 import sc1v2
from models.sw_kfnet import sc5v3
from models.regression_nf_3d_hci_3vm2_best_lite import RegressFlow3D_hci_3vm2_best_lite
from models.layers.regression_nf_3d_cbam_lite import RegressFlow3D_cbam_lite
import numpy as np
import cv2
import time
from ultralytics import YOLO
import pdb
import torchvision.transforms as transforms
import torchvision.transforms.functional as F
from matplotlib.backends.backend_agg import FigureCanvasAgg
from tqdm import trange
import matplotlib.pyplot as plt
import math
import matplotlib

matplotlib.use("Agg")


class AITEPoseV3:
    def __init__(
        self,
        dict_path,
        smooth_path,
        filter_path=None,
        resnet=50,
        joint_num=18,
        window_size_s=27,
        window_size_f=32,
        input_h=256,
        input_w=256,
        mean=[0.485, 0.456, 0.406],
        std=[0.229, 0.224, 0.225],
        cuda_id=0,
    ):
        self.joint_num = joint_num
        self.input_h = input_h
        self.input_w = input_w
        self.window_size_s = window_size_s
        self.window_size_f = window_size_f
        self.device = torch.device(f"cuda:{cuda_id}" if torch.cuda.is_available() else "cpu")
        print("Inference with", "GPU" if torch.cuda.is_available() else "CPU")

        self.model = RegressFlow3D_hci_3vm2_best_lite(joint_num=joint_num, resnet=resnet).to(
            self.device
        )
        # self.model = RegressFlow3D_cbam_lite(joint_num=joint_num, resnet=resnet).to(
        #     self.device
        # )
        self.model.load_state_dict(torch.load(dict_path, map_location=self.device), strict=False)
        self.model.eval()  # Set the model to evaluation mode
        self.smoother = as1v1(window_size=window_size_s).to(self.device)
        self.smoother.load_state_dict(
            torch.load(smooth_path, map_location=self.device), strict=True
        )
        self.smoother.eval()  # Set the model to evaluation mode
        if filter_path is not None:
            self.filter = sc5v3(window_size=window_size_f).to(self.device)
            self.filter.load_state_dict(
                torch.load(filter_path, map_location=self.device), strict=True
            )
            self.filter.eval()  # Set the model to evaluation mode
            self.filter2 = sc5v3(window_size=window_size_f).to(self.device)
            self.filter2.load_state_dict(
                torch.load(filter_path, map_location=self.device), strict=True
            )
            self.filter2.eval()  # Set the model to evaluation mode
        self.skeleton = (
            (1, 0),
            (2, 1),
            (3, 2),  # 2
            (4, 0),
            (5, 4),
            (6, 5),  # 5
            (7, 8),  # 7
            (7, 0),
            (9, 8),
            (10, 9),  # 9
            (11, 7),
            (12, 11),
            (13, 12),  # 12
            (14, 7),
            (15, 14),
            (16, 15),  # 15
        )  # 16

        self.mean = np.array(mean).reshape(1, -1, 1, 1)
        self.std = np.array(std).reshape(1, -1, 1, 1)
        self.pred_seq_s = torch.zeros(1, self.window_size_s, 51).to(self.device)
        self.window_seq_s = torch.zeros(1, self.window_size_s - 1, 51).to(
            self.device
        )
        self.pred_seq_f = torch.zeros(1, self.window_size_f, 3).to(self.device)
        self.window_seq_f = torch.zeros(1, self.window_size_f - 1, 3).to(
            self.device
        )  # TOO 这里只限定17关节
        self.pred_seq_f2 = torch.zeros(1, self.window_size_f, 3).to(self.device)
        self.window_seq_f2 = torch.zeros(1, self.window_size_f - 1, 3).to(
            self.device
        )
        self.start_seq = False
        self.frame_num = 0
        self.state = None
        self.Probability = None
        self.state2 = None
        self.Probability2 = None

    def preprocess(self, img, bbox):
        """对图片进行归一化处理，使用bbox进行切割"""
        bbox = bbox.int()
        img = img.copy()

        # 计算bbox相对于原始图片边界的偏移
        x1, y1, x2, y2 = bbox.tolist()
        self.pad_left = max(-x1, 0)
        self.pad_top = max(-y1, 0)
        pad_right = max(x2 - img.shape[1], 0)
        pad_bottom = max(y2 - img.shape[0], 0)
        if not (self.pad_left == 0 and self.pad_top == 0 and pad_right == 0 and pad_bottom == 0):
            # 填充原始图片
            img = cv2.copyMakeBorder(
                img,
                self.pad_top,
                pad_bottom,
                self.pad_left,
                pad_right,
                cv2.BORDER_CONSTANT,
                value=(0, 0, 0),
            )
            # 调整bbox坐标
            x1 += self.pad_left
            y1 += self.pad_top
            x2 += self.pad_left
            y2 += self.pad_top

        # 切割填充后的图片以及调整后的bbox
        # img = np.expand_dims(img, axis=0)
        input_shape = img.shape
        # assert len(input_shape) == 4, "expect shape like (1, C, H, W)"
        img = img[y1:y2, x1:x2, :]
        self.img = img
        img = np.array([cv2.resize(img, (self.input_w, self.input_h))])
        img = (np.transpose(img, (0, 3, 1, 2)) / 255.0 - self.mean) / self.std
        return torch.from_numpy(img.astype(np.float32)).to(self.device)

    def inference(self, img):
        with torch.no_grad():  # No gradient needed for inference
            output = self.model(img)
        return output["pred_jts"], output["maxvals"]

    def post_process(self, coords, bbox):
        # print('output:', coords[0, 16, :], 'output_o', coords[0, 0, :])
        target_coords = torch.zeros(self.joint_num, 3, requires_grad=False).to(self.device)
        target_coords[:, 0] = (coords[0, :, 0] + 0.5) * (bbox[2] - bbox[0]) + bbox[0]
        target_coords[:, 1] = (coords[0, :, 1] + 0.5) * (bbox[3] - bbox[1]) + bbox[1]
        target_coords[:, 2] = coords[0, :, 2]
        # target_coords[:, 0] = target_coords[:, 0]-self.pad_top
        # target_coords[:, 1] = target_coords[:, 1]-self.pad_left
        # # pdb.set_trace()
        return target_coords

    def vis_keypoints(self, img, kps, scores, kp_thresh=0.4, alpha=1):
        # Convert from plt 0-1 RGBA colors to 0-255 BGR colors for OpenCV.
        cmap = plt.get_cmap("rainbow")
        kps_lines = self.skeleton
        colors = [cmap(i) for i in np.linspace(0, 1, len(kps_lines) + 2)]
        colors = [(c[2] * 255, c[1] * 255, c[0] * 255) for c in colors]

        # Perform the drawing on a copy of the image, to allow for blending.
        kp_mask = np.copy(img)

        # Draw the keypoints.
        for link in range(len(kps_lines)):
            i1 = kps_lines[link][0]
            i2 = kps_lines[link][1]
            p1 = (int(kps[i1, 0]), int(kps[i1, 1]))
            p2 = (int(kps[i2, 0]), int(kps[i2, 1]))
            if scores[0, i1, 0] > kp_thresh and scores[0, i2, 0] > kp_thresh:
                cv2.line(
                    kp_mask,
                    p1,
                    p2,
                    color=colors[link],
                    thickness=2,
                    lineType=cv2.LINE_AA,
                )
            if scores[0, i1, 0] > kp_thresh:
                cv2.circle(
                    kp_mask,
                    p1,
                    radius=3,
                    color=colors[link],
                    thickness=-1,
                    lineType=cv2.LINE_AA,
                )
            if scores[0, i2, 0] > kp_thresh:
                cv2.circle(
                    kp_mask,
                    p2,
                    radius=3,
                    color=colors[link],
                    thickness=-1,
                    lineType=cv2.LINE_AA,
                )

        # Blend the keypoints.
        return cv2.addWeighted(img, 1.0 - alpha, kp_mask, alpha, 0)

    def predict(self, img, bbox):
        img_p = self.preprocess(img, bbox)
        coords, scores = self.inference(img_p)
        coords = self.post_process(coords, bbox)
        return coords, scores

    # def smooth_filte(self, coords):
    #     with torch.no_grad():
    #         print("frame_num:", self.frame_num)
    #         if self.frame_num < self.window_size:
    #             self.pred_seq[:, self.frame_num-1, :] = coords.view(1,3) / 1000
    #             self.window_seq[:, self.frame_num-1, :] = coords.view(1,3) / 1000
    #             return coords  # 只输出17关节
    #         else:
    #             self.pred_seq[:, -1, :] = coords.view(1,3)/ 1000
    #             pro_coords = self.smoother(self.pred_seq, self.window_seq)
    #             self.pred_seq[:, -1, :] = pro_coords
    #             filt_coords,self.state,self.Probability = self.filter(self.pred_seq[:,-self.window_size_f:,:],self.window_seq[:,-self.window_size_f+1:,:],None,self.state,self.Probability)
    #             self.pred_seq[:, :-1, :] = self.pred_seq[:, 1:, :].clone()
    #             self.window_seq[:, :-1, :] = self.window_seq[:, 1:, :].clone()
    #             self.window_seq[:, -1, :] = filt_coords
    #             return filt_coords[0] * 1000

    def smooth(self, coords):
        with torch.no_grad():
            # print("frame_num:", self.frame_num)
            if self.frame_num < self.window_size_s:
                self.pred_seq_s[:, self.frame_num-1, :] = coords.view(1, 51) / 1000
                self.window_seq_s[:, self.frame_num-1, :] = coords.view(1, 51) / 1000
                return coords  # 只输出17关节
            else:
                self.pred_seq_s[:, -1, :] = coords.view(1, 51) / 1000
                pro_coords = self.smoother(self.pred_seq_s, self.window_seq_s)
                self.pred_seq_s[:, :-1, :] = self.pred_seq_s[:, 1:, :].clone()
                self.window_seq_s[:, :-1, :] = self.window_seq_s[:, 1:, :].clone()
                self.window_seq_s[:, -1, :] = pro_coords
                return pro_coords.view(17, 3) * 1000

    def filt(self, coords_hand):
        with torch.no_grad():
            # print("frame_num:", self.frame_num)
            if self.frame_num < self.window_size_f:
                self.pred_seq_f[:, self.frame_num-1, :] = coords_hand / 1000
                self.window_seq_f[:, self.frame_num-1, :] = coords_hand / 1000
                return coords_hand  # 只输出17关节
            else:
                self.pred_seq_f[:, -1, :] = coords_hand / 1000
                filt_coords, self.state, self.Probability = self.filter(self.pred_seq_f, self.window_seq_f, None, self.state, self.Probability)
                self.pred_seq_f[:, :-1, :] = self.pred_seq_f[:, 1:, :].clone()
                self.window_seq_f[:, :-1, :] = self.window_seq_f[:, 1:, :].clone()
                self.window_seq_f[:, -1, :] = filt_coords
                return filt_coords[0] * 1000

    def filt2(self, coords_hand):
        with torch.no_grad():
            # print("frame_num:", self.frame_num)
            if self.frame_num < self.window_size_f:
                self.pred_seq_f2[:, self.frame_num-1, :] = coords_hand / 1000
                self.window_seq_f2[:, self.frame_num-1, :] = coords_hand / 1000
                return coords_hand  # 只输出17关节
            else:
                self.pred_seq_f2[:, -1, :] = coords_hand / 1000
                filt_coords, self.state2, self.Probability2 = self.filter2(self.pred_seq_f2, self.window_seq_f2, None, self.state2, self.Probability2)
                self.pred_seq_f2[:, :-1, :] = self.pred_seq_f2[:, 1:, :].clone()
                self.window_seq_f2[:, :-1, :] = self.window_seq_f2[:, 1:, :].clone()
                self.window_seq_f2[:, -1, :] = filt_coords
                return filt_coords[0] * 1000
    
    def read(self, img_path):
        img = cv2.imread(img_path)
        return img

    def write(self, img_path, img):
        img = cv2.imwrite(img_path, img)

    def vis_keypoints_3d(self, kps_3d, scores, kp_thresh):
        # print("kps_3d:", kps_3d)
        # Set the interactive backend
        kps_3d = kps_3d.cpu().numpy() if isinstance(kps_3d, torch.Tensor) else kps_3d
        # plt.switch_backend("nbAgg")
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        kps_lines_3d = self.skeleton
        kps_3d = kps_3d.copy()
        # kps_3d[:,2] = -kps_3d[:,2]+kps_3d[:,1]*90.169/713.76 
        kps_3d = kps_3d[:,[2, 0, 1]]
        kps_3d[:,2] = kps_3d[:,2]*(-1)
        kps_3d[:,0] = kps_3d[:,0]*(-1)
        # kps_3d[:,0] = kps_3d[:,0]-kps_3d[:,2]*98.169/713.76
        print("kps_3d_hand:", kps_3d[10])
        for link in range(len(kps_lines_3d)):
            i1 = kps_lines_3d[link][0]
            i2 = kps_lines_3d[link][1]
            p1 = kps_3d[i1]
            p2 = kps_3d[i2]
            if link in [0, 1, 2, 13, 14, 15]:
                color = "r"
            else:
                color = "b"
            if scores[0, i1, 0] > kp_thresh and scores[0, i2, 0] > kp_thresh:
                ax.plot(
                    [p1[0], p2[0]],
                    [p1[1], p2[1]],
                    [p1[2], p2[2]],
                    color=color,
                    linewidth=2.5,
                    marker="o",
                    markersize=2.5,
                    alpha=0.9,
                )

        # Set the axis labels
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

        ax.set_xlim(-850, 850)
        ax.set_ylim(-850, 850)
        # ax.set_zlim(-850, 850)
        ax.set_zlim(-900, 900)

        # ax.view_init(elev=10, azim=-85)  # best
        # ax.view_init(elev=10, azim=-40)

        # ax.set_xticklabels([])
        # ax.set_yticklabels([])
        # ax.set_zticklabels([])

        ax.set_box_aspect([1, 1, 1])
        # show
        plt.show()
        return fig, ax

    def getbonelength(self, coords):
        bonelength = torch.zeros(len(self.skeleton))
        for i, (parent, child) in enumerate(self.skeleton):
            bonelength[i] = torch.sqrt(torch.sum((coords[parent] - coords[child]) ** 2))
        return bonelength
