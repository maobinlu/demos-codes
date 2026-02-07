from filter import Filter
from cam_utils import pixel2cam, bbox_transform, cam2pixel
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
import matplotlib.pyplot as plt
import cv2
import numpy as np
from models.hcipsoe_finnal import RegressFlow3D_cbam
from models.regression_nf_3d_hci_3vm2_best_lite import RegressFlow3D_hci_3vm2_best_lite
from models.attention_smooth_1v1_lite import as1v1
import os
import torch

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
        self.f = f
        self.c = c
        print("start predict")
        # fourcc = cv2.VideoWriter_fourcc(*"H264")
        # self.capture = cv2.Videoself.Capture(video_path)
        self.capture = cv2.VideoCapture(0)  # 使用相机
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
        self.pose_filter = Filter(s=16, n=17, d=3, r=1)
        fig = plt.figure()
        self.ax = fig.add_subplot(111, projection="3d")
        self.hand_point = np.zeros((3, 1))

        rospy.init_node("hci_node", anonymous=True)
        self.img_pub = rospy.Publisher("cam_img", Image, queue_size=1)
        self.pose_pub = rospy.Publisher("hand_pose", Float32MultiArray, queue_size=1)

    def infer(self, function=None):
        fps = 0
        i = 0
        while True:
            t1 = time.time()
            ref, frame = self.capture.read()
            if not ref:
                break
            frame_p = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.model_det.predict(frame, save=False, imgsz=320, classes=0, conf=0.3)
            if results[0].boxes.xyxy.numel() == 0:
                self.start_seq = False
                self.frame_num = 0
                continue
            else:
                self.start_seq = True
            bbox = results[0].boxes.xywh[0]
            if i == 0:
                pre_w = bbox[2]
                pre_h = bbox[3]
                self.box_filter.init_history(bbox)
            bbox = self.box_filter.update(bbox)
            bbox, pre_w, pre_h = bbox_transform(
                bbox.clone(), self.input_h / self.input_w, pre_w, pre_h, self.width, self.height
            )
            coords, scores = self.model.predict(frame_p, bbox)
            # if (coords[7, 1] - coords[10, 1]) > (bbox[2] - bbox[0]) * 0.4:
            #     scores[0, :7, :] = 0
            coords_3d = coords.clone()
            coords_3d[:, 2] = coords_3d[:, 2] * 2000 + 2000
            # coords_3d[:, 0] = coords[:, 0] * (2000 / (bbox[2] - bbox[0]))
            # coords_3d[:, 1] = coords[:, 1] * (2000 / (bbox[3] - bbox[1]))
            coords_3d = pixel2cam(coords_3d, self.f, self.c)
            coords_root = coords_3d[0]
            coords_3d = coords_3d - coords_root
            coords_3d = self.model.smooth(coords_3d[:17, :])  # ----至此之后是17关节-----
            # coords_3d = coords_3d[:17, :]
            if i == 0:
                self.pose_filter.init_history(coords_3d)
            coords_3d = self.pose_filter.update(coords_3d)

            # convert 3d to 2d
            coords_2d = coords_3d+coords_root
            coords_2d = cam2pixel(coords_2d, self.f, self.c)

            self.templist.append(coords_3d)
            self.scorelist.append(scores)
            self.bbox_list.append(bbox)
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
            self.out.write(frame_show)
            # 绘制新点
            # ax.scatter(coords_3d[16, 0], coords_3d[16, 1], coords_3d[16, 2], color='blue')
            # if i > 0:
            #     # 连接上一个点和当前点
            #     ax.plot([last_point[0], coords_3d[16, 0]], [last_point[1], coords_3d[16, 0]], [last_point[2], coords_3d[16, 0]], color='blue')
            # last_point = coords_3d[16]
            # plt.draw()
            # plt.pause(0.001)

            # # # 直接在线观看
            # if i == 0:
            #     hand_points_init = coords_3d[16]
            self.hand_point = coords_3d[16] - coords_3d[14]

            cv2.imshow("frame", frame_show)
            pub_img = CvBridge().cv2_to_imgmsg(frame_show, "bgr8")
            self.img_pub.publish(pub_img)
            out_hand_pose = [self.hand_point[0], self.hand_point[1], self.hand_point[2]]
            pub_pose = Float32MultiArray()
            pub_pose.data = out_hand_pose
            self.pose_pub.publish(pub_pose)
            if function is not None:
                function()
            if cv2.waitKey(1) & 0xFF == ord("q"):
                n = len(self.templist)
                hand_points = np.zeros((n, 3))
                for j, coord in enumerate(self.templist):
                    hand_points[j] = coord[16]
                self.ax.plot(hand_points[:, 0], hand_points[:, 1], hand_points[:, 2], color="blue")
                plt.show()

                # bl_all = torch.zeros(len(templist), len(model.skeleton))
                # bbox_all = torch.zeros(len(templist), 4)
                # for j, coord in enumerate(templist):
                #     bl_all[j] = model.getbonelength(coord)
                #     bbox_all[j] = bbox_list[j]
                # # pdb.set_trace()
                # fig, ax = model.vis_keypoints_3d(templist[0], scorelist[0], kp_thresh=0.2)
                # ani = FuncAnimation(
                #     fig,
                #     model.update_animation,
                #     frames=i,
                #     fargs=(fig, ax, templist, scorelist, 0.2),
                #     interval=33.33,
                # )
                # ani.save('./output_video3D_path.mp4')
                # print("human pose estimate complete")
                # print(output_video_path)
                break
            i = i + 1
        self.capture.release()
        self.out.release()


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
