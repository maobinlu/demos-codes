from AITEPose import AITEPose
from cam_utils import pixel2cam, bbox_transform
import torch
import os
import numpy as np
import cv2
import time
from ultralytics import YOLO
import pdb
from matplotlib.backends.backend_agg import FigureCanvasAgg
from tqdm import trange
import matplotlib.pyplot as plt
import AITEPose
import matplotlib
import math
print(f"Running {__name__}.py from: {__file__}")


matplotlib.use("Agg")

SUB_FIG_SIZE = 4
SUB_FIG_UNIT = 100
VIEW_NUM = 1
VIEW_CAMERA = [10, 10]


SKELETON_3D_RADIUS = 1.7  # 骨架半径
SKELETON_AX_DIST = 6.5
FRAME_RATE = 30

SKELRTON_COLOR = ["red", "black"]


# /home/aloha/HCI/src/hcipose_lite/hci.py
# from torch2trt import TRTModule


def plot_joint_trajectory(ax_3d, skeleton, gt):
    # pdb.set_trace()

    skeleton = -skeleton[
        :, :, [2, 0, 1]
    ]  # 将骨架的z坐标放在第一列，x坐标放在第二列，y坐标放在第三列
    gt = -gt[:, :, [2, 0, 1]]
    center = np.mean(skeleton[:, 0, :], axis=0)  # 计算骨架的中心点
    # pdb.set_trace()
    ax_3d.set_xlim3d(
        [center[0] - SKELETON_3D_RADIUS / 2, center[0] + SKELETON_3D_RADIUS / 2]
    )  # 设置x轴的范围为中心点的x坐标减去骨架半径的一半到中心点的x坐标加上骨架半径的一半
    ax_3d.set_ylim3d([center[1] - SKELETON_3D_RADIUS / 2, center[1] + SKELETON_3D_RADIUS / 2])
    ax_3d.set_zlim3d([center[2], center[2] + SKELETON_3D_RADIUS / 2])
    ax_3d.plot3D(
        skeleton[:, 16, 0],
        skeleton[:, 16, 1],
        skeleton[:, 16, 2],
        color=SKELRTON_COLOR[0],
        alpha=0.9,
    )
    ax_3d.plot3D(
        gt[:, 16, 0],
        gt[:, 16, 1],
        gt[:, 16, 2],
        color=SKELRTON_COLOR[1],
        alpha=0.9,
    )
    return


def plot_skeleton(ax_3d, skeleton, color, edges=None, joints=None):
    skeleton = -skeleton[:, [2, 0, 1]]
    center = np.mean(skeleton, axis=0)  # 计算骨架的中心点

    ax_3d.set_xlim3d(
        [center[0] - SKELETON_3D_RADIUS / 2, center[0] + SKELETON_3D_RADIUS / 2]
    )  # 设置x轴的范围为中心点的x坐标减去骨架半径的一半到中心点的x坐标加上骨架半径的一半
    ax_3d.set_ylim3d([center[1] - SKELETON_3D_RADIUS / 2, center[1] + SKELETON_3D_RADIUS / 2])
    ax_3d.set_zlim3d([center[2] - SKELETON_3D_RADIUS / 2, center[2] + SKELETON_3D_RADIUS / 2])

    kps_lines_3d = (
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

    rad_line = [1, 2, 3, 14, 15, 16]

    for link in range(len(kps_lines_3d)):
        i1 = kps_lines_3d[link][0]
        i2 = kps_lines_3d[link][1]
        p1 = skeleton[i1]
        p2 = skeleton[i2]
        k = 0 if i1 in rad_line or i2 in rad_line else 1
        ax_3d.plot3D(
            [p1[0], p2[0]],
            [p1[1], p2[1]],
            [p1[2], p2[2]],
            color=SKELRTON_COLOR[k],
            marker="o",
            markersize=2,
            alpha=0.9,
        )

    return


# 加权递推平均滤波
class Filter:
    def __init__(self, s, n, r=1, d=0):
        """
        Parameters:
            s (int): The size of the window.
            n (int): The number of elements.
            r (float, optional): The decay rate. Defaults to 1.
            d (int, optional): The number of dimensions. Defaults to 0.

        """
        self.weights_sum = 0
        if d == 0:
            self.mask = torch.ones((s, n))
            self.history = torch.ones((s, n))
        else:
            self.mask = torch.ones((s, n, d))
            self.history = torch.ones((s, n, d))
        for i in range(s):
            self.weights_sum += math.exp(-1 * r * i)
            self.mask[i] = self.mask[i] * math.exp(-1 * r * i)

        self.win_size = s

    def init_history(self, item):
        # print(item.device)
        for i in range(self.win_size):
            self.history[i] = item
        self.history = self.history.to(item.device)
        self.mask = self.mask.to(item.device)
        # self.weights_sum=self.weights_sum.to(item.device)
        # print(self.history.device)

    def update(self, item):
        self.history[1:] = self.history[:-1].clone()
        self.history[0] = item
        self.item = self.history * self.mask
        item_out = self.item.sum(dim=0) / self.weights_sum
        return item_out


class HPE_3D:
    def __init__(self, dict_path_152, smooth_path, joint_num, f, c):
        self.model = AITEPose(
            dict_path=dict_path_152,
            smooth_path=smooth_path,
            resnet=152,
            joint_num=joint_num,
            input_h=288,
            input_w=384,
        )
        self.model_det = YOLO("model/weight/yolov8n.pt")
        # model_det = YOLO("model_weight/yolov8n.engine", task="detect")
        # model_det = YOLO("model_weight/yolov8n.onnx", task="detect")
        print("start predict")
        # fourcc = cv2.VideoWriter_fourcc(*"H264")
        self.capture = cv2.VideoCapture(video_path)  # 使用视频
        # self.capture = cv2.VideoCapture(0)  # 使用相机
        if not self.capture.isOpened():
            print("Error: Unable to open video.")
            exit()
        self.width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        tall0 = time.time()
        # 定义输出视频的编码器
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        # 创建输出视频写入对象
        self.out = cv2.VideoWriter(output_video_path, fourcc, 30.0, (self.width, self.height))

        self.templist = []
        self.scorelist = []
        self.bbox_list = []
        self.box_filter = Filter(s=32, n=4, r=1)
        self.pose_filter = Filter(s=64, n=joint_num - 1, d=3, r=1)
        self.hand_point = np.zeros((3, 1))
        # rospy.init_node("hci_node", anonymous=True)

    def infer(self, function=None):
        videoWriter = cv2.VideoWriter(
            os.path.join("vis_output", "vis_output_video_name"),
            cv2.VideoWriter_fourcc(*"mp4v"),
            FRAME_RATE,
            (
                2 * SUB_FIG_UNIT,
                2 * SUB_FIG_UNIT,
            ),
        )
        fps = 0
        self.frame_num = 0
        while True:
            t1 = time.time()
            ref, frame = self.capture.read()
            fig = plt.figure(figsize=(2 * SUB_FIG_UNIT, 2 * SUB_FIG_UNIT))
            plt.subplots_adjust(
                left=None, bottom=None, right=None, top=None, wspace=None, hspace=0.5
            )
            if not ref:
                break

            # 得到bbox
            frame_p = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.model_det.predict(frame, save=False, imgsz=320, classes=0, conf=0.3)
            if results[0].boxes.xyxy.numel() == 0:
                self.start_seq = False
                self.frame_num = 0
                continue
            else:
                self.start_seq = True
            bbox = results[0].boxes.xywh[0]
            if self.frame_num == 0:
                pre_w = bbox[2]
                pre_h = bbox[3]
                self.box_filter.init_history(bbox)
            bbox = self.box_filter.update(bbox)
            # bbox, pre_w, pre_h = bbox_transform(bbox.clone(), 1, pre_w, pre_h, width, height)
            bbox, pre_w, pre_h = bbox_transform(
                bbox.clone(), 288 / 384, pre_w, pre_h, self.width, self.height
            )

            # 开始预测pose
            coords, scores = self.model.predict(frame_p, bbox)
            if (coords[7, 1] - coords[10, 1]) > (bbox[3] - bbox[1]) * 0.4:
                scores[0, :7, :] = 0
            # pdb.set_trace()
            coords_3d = coords.clone()
            coords_3d[:, 2] = coords_3d[:, 2] * 2000 + 2000
            coords_3d[:, 0] = coords[:, 0] * (2000 / (bbox[2] - bbox[0]))
            coords_3d[:, 1] = coords[:, 1] * (2000 / (bbox[3] - bbox[1]))
            # coords_3d = pixel2cam(coords_3d, f, c)
            # pdb.set_trace()
            coords_o = coords_3d[0].clone()
            coords_3d = coords_3d - coords_3d[0]
            coords_pred = coords_3d.clone()
            coords_3d = self.model.smooth(coords_3d)  # ----至此之后是17关节-----
            if self.frame_num == 0:
                self.pose_filter.init_history(coords_3d)
            coords_3d = self.pose_filter.update(coords_3d)
            if self.frame_num == 0:
                pro_data = coords_3d.unsqueeze(0)
                pred_data = coords_pred.unsqueeze(0)
            else:
                pro_data = torch.cat((pro_data, coords_3d.unsqueeze(0)), dim=0)
                pred_data = torch.cat((pred_data, coords_pred.unsqueeze(0)), dim=0)

            # 原画绘制
            coords = coords_3d + coords_o
            coords[:, 0] = coords_3d[:, 0] / (2000 / (bbox[2] - bbox[0]))
            coords[:, 1] = coords_3d[:, 1] / (2000 / (bbox[3] - bbox[1]))
            frame_show = self.model.vis_keypoints(frame, coords.cpu(), scores.cpu(), kp_thresh=0.2)
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

            frame_show_rgb = cv2.cvtColor(frame_show, cv2.COLOR_BGR2RGB)
            ax1 = plt.subplot(2, 2, 1)
            ax1.imshow(frame_show_rgb)  # 使用 RGB 图像
            ax1.axis("off")  # 关闭坐标轴显示
            ax1.set_title("Frame Show")  # 设置标题

            # 绘制右手轨迹
            ax_3dtr = fig.add_subplot(
                2,  # 3行
                2,  # 列数 2*3=6
                2,  # 利用view_i和index来确定位置 1*3+1=4; 1*3+2=5; 1*3+3=6
                projection="3d",  # 3D图
            )
            ax_3dtr.view_init(elev=VIEW_CAMERA[0], azim=VIEW_CAMERA[1])
            ax_3dtr.set_xticklabels([])
            ax_3dtr.set_yticklabels([])
            ax_3dtr.set_zticklabels([])

            ax_3dtr.dist = SKELETON_AX_DIST
            ax_3dtr.set_title("right hand trajectory", fontsize=3 * SUB_FIG_SIZE)
            if self.frame_num > 0:
                plot_joint_trajectory(ax_3dtr, pro_data.cpu().numpy(), pred_data.cpu().numpy())

            # 绘制3D数据pro
            ax_3d1 = fig.add_subplot(
                2,  # 3行
                2,  # 列数 2*3=6
                3,  # 利用view_i和index来确定位置 1*3+1=4; 1*3+2=5; 1*3+3=6
                projection="3d",  # 3D图
            )
            ax_3d1.view_init(elev=VIEW_CAMERA[0], azim=VIEW_CAMERA[1])
            ax_3d1.set_xticklabels([])
            ax_3d1.set_yticklabels([])
            ax_3d1.set_zticklabels([])
            ax_3d1.dist = SKELETON_AX_DIST
            ax_3d1.set_title("AITEPose", fontsize=3 * SUB_FIG_SIZE)
            plot_skeleton(ax_3d1, coords_pred[:17, :].cpu().numpy(), SKELRTON_COLOR)

            # 绘制3D数据pro
            ax_3d = fig.add_subplot(
                2,  # 3行
                2,  # 列数 2*3=6
                4,  # 利用view_i和index来确定位置 1*3+1=4; 1*3+2=5; 1*3+3=6
                projection="3d",  # 3D图
            )
            ax_3d.view_init(elev=VIEW_CAMERA[0], azim=VIEW_CAMERA[1])
            ax_3d.set_xticklabels([])
            ax_3d.set_yticklabels([])
            ax_3d.set_zticklabels([])
            ax_3d.dist = SKELETON_AX_DIST
            ax_3d.set_title("AITEPose+Smoother", fontsize=3 * SUB_FIG_SIZE)
            plot_skeleton(ax_3d, coords_3d.cpu().numpy(), SKELRTON_COLOR)

            canvas = FigureCanvasAgg(plt.gcf())
            canvas.draw()
            final_img = np.array(canvas.renderer.buffer_rgba())[:, :, [2, 1, 0]]
            self.frame_num = self.frame_num + 1
            # plt.savefig("tmp" + str(frame_i) + ".png")
            cv2.imwrite(os.path.join(video_file, "last_frame.png"), final_img)
            videoWriter.write(final_img)
            plt.close()
        videoWriter.release()
        self.capture.release()
        print(
            f"Finish! The video is stored in " + os.path.join("vis_output", "vis_output_video_name")
        )


dict_path_50 = "models/weight/best_N-model_48.2892-50-lite.pth"
dict_path_152 = "models/weight/best_N-model_46.2329-152-lite.pth"
smooth_path = "models/weight/best_model_7_46.6179.pth"
joint_num = 18
video_name = "Baduanjin1"
video_file = "vis_output"
video_path = os.path.join(video_file, video_name + ".mp4")
print(video_path)
output_video_path = os.path.join(video_file, video_name + "_output.mp4")
output_video3D_path = os.path.join(video_file, video_name + "_output3D.mp4")
f = [451.477162, 453.875205]  # aloha 电脑参数
c = [272.213870, 294.493310]
if __name__ == "__main__":
    hpe = HPE_3D(dict_path_152, smooth_path, joint_num, f, c)
    hpe.infer()
