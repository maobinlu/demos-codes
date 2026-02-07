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
import os
import torch
os.chdir('/home/aloha/HCI/src/hcipose_lite')
# /home/aloha/HCI/src/hcipose_lite/hci.py
# from torch2trt import TRTModule


class Pose_inference:
    def __init__(
        self,
        dict_path,
        resnet=50,
        joint_num=18,
        input_h=256,
        input_w=256,
        mean=[0.485, 0.456, 0.406],
        std=[0.229, 0.224, 0.225],
        cuda_id=0,
    ):
        self.joint_num = joint_num
        self.input_h = input_h
        self.input_w = input_w
        self.device = torch.device(f"cuda:{cuda_id}" if torch.cuda.is_available() else "cpu")
        print("Inference with", "GPU" if torch.cuda.is_available() else "CPU")

        self.model = RegressFlow3D_cbam(joint_num=joint_num, resnet=resnet).to(self.device)
        self.model.load_state_dict(
            torch.load(dict_path, map_location=self.device), strict=False
        )
        self.model.eval()  # Set the model to evaluation mode

        # self.model = TRTModule()
        # self.model.load_state_dict(torch.load('finnal_trt.pth'))

        self.transform = transforms.Compose(
            [
                transforms.ToPILImage(),  # Convert numpy array to PILImage
                transforms.Resize((input_h, input_w)),  # Resize image
                transforms.ToTensor(),  # Convert image to tensor
                transforms.Normalize(
                    mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
                ),  # Normalize the image
            ]
        )
        self.skeleton = (
            (1, 0),
            (2, 1),
            (3, 2),
            (4, 0),
            (5, 4),
            (6, 5),
            (7, 0),
            (8, 17),
            (9, 8),
            (10, 9),
            (11, 17),
            (12, 11),
            (13, 12),
            (14, 17),
            (15, 14),
            (16, 15),
            (17, 7),
        )
        self.mean = np.array(mean).reshape(1, -1, 1, 1)
        self.std = np.array(std).reshape(1, -1, 1, 1)

    # def preprocess(self, img, bbox):
    #     """对图片进行归一化处理，使用bbox进行切割"""
    #     bbox = bbox.int()
    #     img = img.copy()
    #     img = np.expand_dims(img, axis=0)
    #     input_shape = img.shape
    #     assert len(input_shape) == 4, "expect shape like (1, C, H, W)"
    #     img = img[:, bbox[1] : bbox[3], bbox[0] : bbox[2], :]
    #     img = np.array([cv2.resize(img[0], (self.input_h, self.input_w))])
    #     img = (np.transpose(img, (0, 3, 1, 2)) / 255.0 - self.mean) / self.std
    #     return torch.from_numpy(img.astype(np.float32)).to(self.device)

    def preprocess(self, img, bbox):
        """对图片进行归一化处理，使用bbox进行切割"""
        bbox = bbox.int()
        img = img.copy()

        # 计算bbox相对于原始图片边界的偏移
        x1, y1, x2, y2 = bbox.tolist()
        pad_left = max(-x1, 0)
        pad_top = max(-y1, 0)
        pad_right = max(x2 - img.shape[1], 0)
        pad_bottom = max(y2 - img.shape[0], 0)
        if not (pad_left == 0 and pad_top == 0 and pad_right == 0 and pad_bottom == 0):
            # 填充原始图片
            img = cv2.copyMakeBorder(
                img,
                pad_top,
                pad_bottom,
                pad_left,
                pad_right,
                cv2.BORDER_CONSTANT,
                value=(0, 0, 0),
            )
            # 调整bbox坐标
            x1 += pad_left
            y1 += pad_top
            x2 += pad_left
            y2 += pad_top

        # 切割填充后的图片以及调整后的bbox
        # img = np.expand_dims(img, axis=0)
        input_shape = img.shape
        # assert len(input_shape) == 4, "expect shape like (1, C, H, W)"
        img = img[y1:y2, x1:x2, :]
        img = np.array([cv2.resize(img, (self.input_w, self.input_h))])
        img = (np.transpose(img, (0, 3, 1, 2)) / 255.0 - self.mean) / self.std
        return torch.from_numpy(img.astype(np.float32)).to(self.device)

    def inference(self, img):
        with torch.no_grad():  # No gradient needed for inference
            output = self.model(img)
        return output["pred_jts"], output["maxvals"]

    def post_process(self, coords, bbox):
        target_coords = torch.zeros(self.joint_num, 3, requires_grad=False).to(
            self.device
        )
        target_coords[:, 0] = (coords[0, :, 0] + 0.5) * (bbox[2] - bbox[0]) + bbox[0]
        target_coords[:, 1] = (coords[0, :, 1] + 0.5) * (bbox[3] - bbox[1]) + bbox[1]
        target_coords[:, 2] = coords[0, :, 2]
        # pdb.set_trace()
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

    def read(self, img_path):
        img = cv2.imread(img_path)
        return img

    def write(self, img_path, img):
        img = cv2.imwrite(img_path, img)

    def vis_keypoints_3d(self, kps_3d, scores, kp_thresh):
        # Set the interactive backend
        kps_3d = kps_3d.cpu().numpy() if isinstance(kps_3d, torch.Tensor) else kps_3d
        plt.switch_backend("nbAgg")
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        kps_lines_3d = self.skeleton
        kps_3d_copy = kps_3d.copy()
        kps_3d[:, 0] = kps_3d_copy[:, 0]
        kps_3d[:, 1] = kps_3d_copy[:, 2]
        kps_3d[:, 2] = -kps_3d_copy[:, 1]
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
        ax.view_init(elev=10, azim=-40)

        ax.set_xticklabels([])
        ax.set_yticklabels([])
        ax.set_zticklabels([])

        ax.set_box_aspect([1, 1, 1])
        return fig, ax

    def update_animation(self, frame, fig, ax, kps_3d_list, scores_list, kp_thresh):
        ax.clear()
        kps_lines_3d = self.skeleton
        kps_3d = kps_3d_list[frame].cpu().numpy()
        kps_3d_copy = kps_3d.copy()
        kps_3d[:, 0] = kps_3d_copy[:, 0]
        kps_3d[:, 1] = kps_3d_copy[:, 2]
        kps_3d[:, 2] = -kps_3d_copy[:, 1]
        for link in range(len(kps_lines_3d)):
            i1 = kps_lines_3d[link][0]
            i2 = kps_lines_3d[link][1]
            p1 = kps_3d[i1]
            p2 = kps_3d[i2]
            if link in [0, 1, 2, 13, 14, 15]:
                color = "r"
            else:
                color = "b"
            if (
                scores_list[frame][0, i1, 0] > kp_thresh
                and scores_list[frame][0, i2, 0] > kp_thresh
            ):
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
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_xlim(-850, 850)
        ax.set_ylim(-850, 850)
        ax.set_zlim(-900, 900)
        # ax.view_init(elev=10, azim=-85)  # best
        ax.view_init(elev=10, azim=-40)
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        ax.set_zticklabels([])
        ax.set_box_aspect([1, 1, 1])
        return fig, ax

    def getbonelength(self, coords):
        bonelength = torch.zeros(len(self.skeleton))
        for i, (parent, child) in enumerate(self.skeleton):
            bonelength[i] = torch.sqrt(torch.sum((coords[parent] - coords[child]) ** 2))
        return bonelength


def pixel2cam(pixel_coord, f, c):
    x = (pixel_coord[:, 0] - c[0]) / f[0] * pixel_coord[:, 2]
    y = (pixel_coord[:, 1] - c[1]) / f[1] * pixel_coord[:, 2]
    z = pixel_coord[:, 2]
    cam_coord = np.concatenate((x[:, None], y[:, None], z[:, None]), 1)
    return torch.tensor(cam_coord)


def bbox_transform(bbox, ratio, pre_w, pre_h, img_w, img_h):
    x, y, w, h = bbox
    l_w = max(w, pre_w)
    l_h = max(h, pre_h)
    l = max(l_w, l_h / ratio)
    # l = max(w, h / ratio)
    y1 = y - l * ratio / 2
    y2 = y + l * ratio / 2
    x1 = x - l / 2
    x2 = x + l / 2
    # pdb.set_trace()
    # x1 = torch.clamp(x1, min=0)
    # y1 = torch.clamp(y1, min=0)
    # x2 = torch.clamp(x2, max=img_w)
    # y2 = torch.clamp(y2, max=img_h)
    return torch.stack([x1.floor(), y1.floor(), x2.ceil(), y2.ceil()]).to(bbox.device), l, l * ratio


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
        for i in range(self.win_size):
            self.history[i] = item
        self.history.to(item.device)

    def update(self, item):
        self.history[1:] = self.history[:-1].clone()
        self.history[0] = item
        self.item = self.history * self.mask
        item_out = self.item.sum(dim=0) / self.weights_sum
        return item_out


class Infer:
    def __init__(self, dict_path_152, joint_num, f, c):
        self.model = Pose_inference(dict_path=dict_path_152, resnet=152, joint_num=joint_num, input_h=288, input_w=384)
        self.model_det = YOLO("model_weight/yolov8n.pt")
        # model_det = YOLO("model_weight/yolov8n.engine", task="detect")
        # model_det = YOLO("model_weight/yolov8n.onnx", task="detect")
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
        self.pose_filter = Filter(s=64, n=joint_num, d=3, r=1)
        fig = plt.figure()
        self.ax = fig.add_subplot(111, projection='3d')
        self.hand_point = np.zeros((3, 1))

        rospy.init_node('hci_node', anonymous=True)
        self.img_pub = rospy.Publisher('cam_img', Image, queue_size=1)
        self.pose_pub = rospy.Publisher('hand_pose', Float32MultiArray, queue_size=1)

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
                continue
            bbox = results[0].boxes.xywh[0]
            if i == 0:
                pre_w = bbox[2]
                pre_h = bbox[3]
                self.box_filter.init_history(bbox)
            bbox = self.box_filter.update(bbox)
            # bbox, pre_w, pre_h = bbox_transform(bbox.clone(), 1, pre_w, pre_h, width, height)
            bbox, pre_w, pre_h = bbox_transform(bbox.clone(), 288/384, pre_w, pre_h, self.width, self.height)
            coords, scores = self.model.predict(frame_p, bbox)
            if (coords[7, 1]-coords[10, 1]) > (bbox[3]-bbox[1])*0.4:
                scores[0, :7, :] = 0
            # pdb.set_trace()
            if i == 0:
                self.pose_filter.init_history(coords)
            coords = self.pose_filter.update(coords)
            coords_3d = coords.clone()
            coords_3d[:, 2] = coords_3d[:, 2] * 2000 + 2000
            # coords_3d[:, 0] = coords[:, 0] * (2000 / (bbox[2] - bbox[0]))
            # coords_3d[:, 1] = coords[:, 1] * (2000 / (bbox[3] - bbox[1]))
            coords_3d = pixel2cam(coords_3d, f, c)
            coords_3d = coords_3d - coords_3d[0]
            self.templist.append(coords_3d)
            self.scorelist.append(scores)
            self.bbox_list.append(bbox)
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
            self.hand_point = coords_3d[16]-coords_3d[14]

            cv2.imshow('frame', frame_show)
            pub_img = CvBridge().cv2_to_imgmsg(frame_show, "bgr8")
            self.img_pub.publish(pub_img)
            out_hand_pose = [self.hand_point[0], self.hand_point[1], self.hand_point[2]]
            pub_pose = Float32MultiArray()
            pub_pose.data = out_hand_pose
            self.pose_pub.publish(pub_pose)
            if function is not None:
                function()
            if cv2.waitKey(1) & 0xFF == ord('q'):
                n = len(self.templist)
                hand_points = np.zeros((n, 3))
                for j, coord in enumerate(self.templist):
                    hand_points[j] = coord[16]
                self.ax.plot(hand_points[:, 0], hand_points[:, 1], hand_points[:, 2], color='blue')
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


dict_path_50 = "model_weight/hci_0v1.pth"
dict_path_152 = "model_weight/hci_0v1_152.pth"
joint_num = 22
video_name = 'ROKI'
video_file = 'video'
video_path = os.path.join(video_file, video_name+'.mp4')
output_video_path = os.path.join(video_file, video_name+'_output.mp4')
output_video3D_path = os.path.join(video_file, video_name+'_output3D.mp4')
f = [451.477162, 453.875205]  # aloha 电脑参数
c = [272.213870, 294.493310]
if __name__ == "__main__":
    hpe = Infer(dict_path_152, joint_num, f, c)
    hpe.infer()
