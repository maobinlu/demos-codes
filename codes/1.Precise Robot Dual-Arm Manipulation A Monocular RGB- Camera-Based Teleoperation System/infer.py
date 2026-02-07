import torch
# from torch2trt import TRTModule
from models.hcipsoe_finnal import RegressFlow3D_cbam
import numpy as np
import cv2
import matplotlib.pyplot as plt
import time
from ultralytics import YOLO
import pdb
import os
import torchvision.transforms as transforms
import torchvision.transforms.functional as F
# pdb.set_trace()


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
    ):
        self.joint_num = joint_num
        self.input_h = input_h
        self.input_w = input_w
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print("Inference with", "GPU" if torch.cuda.is_available() else "CPU")

        self.model = RegressFlow3D_cbam(resnet=resnet, joint_num=joint_num).to(self.device)
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
    #     img = img[bbox[1]:bbox[3], bbox[0]:bbox[2]]
    #     img = cv2.resize(img, (self.input_w, self.input_h))
    #     img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    #     img = np.transpose(img, (2, 0, 1))  # Change data layout from HWC to CHW
    #     img = np.expand_dims(img, axis=0)  # Add batch dimension to match the mean/std shape
    #     img = (img - self.mean) / self.std  # Now img is (1, C, H, W) and can be broadcast with mean and std
    #     return torch.from_numpy(img).to(self.device)

    # tensor
    # def preprocess(self, img, bbox):
    #     # x1, y1, x2, y2 = bbox.unbind(0)
    #     bbox = bbox.int()
    #     x1, y1, x2, y2 = bbox.tolist()
    #     img_cropped = img[x1:x2, y1:y2]
    #     img_tensor = self.transform(img_cropped).to(self.device)
    #     return img_tensor.unsqueeze(0)  # Add a batch dimension if needed

    # def preprocess(self, img, bbox):
    #     img = cv2.resize(img[bbox[1]:bbox[3], bbox[0]:bbox[2]], (self.input_w, self.input_h))
    #     img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    #     img = (img - self.mean) / self.std
    #     img = img.transpose(2, 0, 1)  # Change data layout from HWC to CHW
    #     return torch.from_numpy(img[None]).to(self.device)

    def preprocess(self, img, bbox):
        """对图片进行归一化处理，使用bbox进行切割"""
        bbox = bbox.int()
        img = img.copy()
        img = np.expand_dims(img, axis=0)
        input_shape = img.shape
        assert len(input_shape) == 4, "expect shape like (1, C, H, W)"
        img = img[:, bbox[1]: bbox[3], bbox[0]: bbox[2], :]
        img = np.array([cv2.resize(img[0], (self.input_h, self.input_w))])
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

    def vis_keypoints_3d(self, kps_3d, save_path=None):
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
        color = "b"
        for link in range(len(kps_lines_3d)):
            i1 = kps_lines_3d[link][0]
            i2 = kps_lines_3d[link][1]
            p1 = kps_3d[i1]
            p2 = kps_3d[i2]
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

        ax.view_init(elev=10, azim=-85)  # best
        # ax.view_init(elev=10, azim=-60)

        # ax.set_xticklabels([])
        # ax.set_yticklabels([])
        # ax.set_zticklabels([])

        ax.set_box_aspect([1, 1, 1])
        # ax.set_axis_off()
        # ax.grid(False)

        # Set the view angle
        # ax.view_init(elev=90, azim=90)
        if save_path is not None:
            # Save the figure as a PNG file
            plt.savefig(save_path, format="png", dpi=500)
            plt.close(fig)
        else:
            # Show the plot
            plt.show()


# 取bbox的长宽最大值，并变换bbox的长宽一致,bbox中心保持不变
def get_max_bbox(bbox):
    x1, y1, x2, y2 = bbox
    w, h = x2 - x1, y2 - y1
    if w > h:
        y1 = y1 - (w - h) // 2
        y2 = y2 + (w - h) // 2
    else:
        x1 = x1 - (h - w) // 2
        x2 = x2 + (h - w) // 2
    return [x1, y1, x2, y2]


def get_max_bbox_ratio(bbox, ratio):
    x1, y1, x2, y2 = bbox.unbind(0)
    w, h = x2 - x1, y2 - y1
    if w > h:
        adjust = ((w - h) * ratio) / 2
        y1 -= adjust
        y2 += adjust
    else:
        adjust = ((h - w) / ratio) / 2
        x1 -= adjust
        x2 += adjust
    return torch.stack([x1, y1, x2, y2])


# 调整bbox为指定的长宽比，bbox中心不变
def resize_bbox(bbox, ratio):
    x1, y1, x2, y2 = bbox
    w, h = x2 - x1, y2 - y1
    x1 = x1 - (w * ratio) // 2
    x2 = x2 + (w * ratio) // 2
    y1 = y1 - (h * ratio) // 2
    y2 = y2 + (h * ratio) // 2
    return [x1, y1, x2, y2]


def bbox_transform(bbox, ratio, img_w, img_h):
    x1, y1, x2, y2 = bbox
    w, h = x2 - x1, y2 - y1
    if w > h:
        adjust = ((w - h) * ratio) / 2
        y1 -= adjust
        y2 += adjust
    else:
        adjust = ((h - w) / ratio) / 2
        x1 -= adjust
        x2 += adjust
    x1 = torch.clamp(x1, min=0)
    y1 = torch.clamp(y1, min=0)
    x2 = torch.clamp(x2, max=img_w)
    y2 = torch.clamp(y2, max=img_h)
    return torch.stack([x1.floor(), y1.floor(), x2.ceil(), y2.ceil()])


def adjust_bbox(bbox, img_w, img_h):
    x1, y1, x2, y2 = bbox.unbind(0)
    x1 = torch.clamp(x1, min=0)
    y1 = torch.clamp(y1, min=0)
    x2 = torch.clamp(x2, max=img_w)
    y2 = torch.clamp(y2, max=img_h)
    return torch.stack([x1.floor(), y1.floor(), x2.ceil(), y2.ceil()])


def pixel2cam(pixel_coord, f, c):
    x = (pixel_coord[:, 0].cpu() - c[0]) / f[0] * pixel_coord[:, 2].cpu()
    y = (pixel_coord[:, 1].cpu() - c[1]) / f[1] * pixel_coord[:, 2].cpu()
    z = pixel_coord[:, 2].cpu()
    cam_coord = np.concatenate((x[:, None].cpu(), y[:, None].cpu(), z[:, None].cpu()), 1)
    return torch.tensor(cam_coord)


dict_path_50 = "finnal.pth"
dict_path = "final_152_288_384.pth"
video_path = ""
output_video_path = ""

# if __name__ == "__main__":
#     model = Pose_inference(dict_path)
#     capture = cv2.VideoCapture(video_path)
#     width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
#     height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
#     # 定义输出视频的编码器
#     fourcc = cv2.VideoWriter_fourcc(*'mp4v')
#     # 创建输出视频写入对象
#     out = cv2.VideoWriter(output_video_path, fourcc, 30.0, (width, height))
#     fps = 0
#     while True:
#         t1 = time.time()
#         ref, frame = capture.read()
#         if not ref:
#             break
#         frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#         coords, scorse = model.predict(frame, bbox=[])
#         frame_show = model.vis_keypoints(frame, coords, scorse)
#         t = time.time() - t1
#         fps = 1 / t
#         frame_show = cv2.putText(
#             frame_show,
#             "fps= %.2f, t=%.3 ms" % (fps, t * 1000),
#             (0, 40),
#             cv2.FONT_HERSHEY_SIMPLEX,
#             1,
#             (0, 255, 0),
#             2,
#         )
#         out.write(frame_show)
#     capture.release()
#     out.release()
#     print('human pose estimate complete')

# img_path = './data/MPI-INF-3DHP/mpi_inf_3dhp_test_set/test_preprocess/images/TS1_000001.jpg'
# bbox = [537, 348, 1486, 1437]
# img_path = './data/mpii/images/009303037.jpg'
img_path = "./data/mpii/images/005495787.jpg"
# img_path = "./data/mpii/images/009770728.jpg"
# img_path = "./data/mpii/images/000950620.jpg"  # 1
# img_path = "./data/mpii/images/000001163.jpg"  #
# img_path = "./data/mpii/images/000003072.jpg"  #
# img_path = "./data/mpii/images/000563131.jpg"  #
# img_path = "./data/mpii/images/000644944.jpg"  #
# img_path = "./data/mpii/images/001009041.jpg"  #
# img_path = "./data/mpii/images/001363985.jpg"  # 2

path_list = [
    "./data/mpii/images/005495787.jpg",
    "./data/mpii/images/009770728.jpg",
    "./data/mpii/images/000950620.jpg",
    "./data/mpii/images/000001163.jpg",
    "./data/mpii/images/000003072.jpg",
    "./data/mpii/images/000563131.jpg",
    "./data/mpii/images/000644944.jpg",
    "./data/mpii/images/001009041.jpg",
    "./data/mpii/images/001363985.jpg",
    "./data/mpii/images/009303037.jpg",
    "./data/mpii/images/005495787.jpg",
    "./data/mpii/images/009770728.jpg",
    "./data/mpii/images/000950620.jpg",
    "./data/mpii/images/000001163.jpg",
    "./data/mpii/images/000003072.jpg",
    "./data/mpii/images/000563131.jpg",
    "./data/mpii/images/000644944.jpg",
    "./data/mpii/images/001009041.jpg",
    "./data/mpii/images/001363985.jpg",
    "./data/mpii/images/009303037.jpg",
]


path_list = ['test.jpg']
# img_path = './data/mpii/images/016704178.jpg'


img_id = int(os.path.splitext(os.path.basename(img_path))[0])
num = 0
f = np.array([[451.477162], [453.875205]])
c = np.array([[272.213870], [294.493310]])
"""处理单张图片"""
if __name__ == "__main__":
    model = Pose_inference(dict_path=dict_path_50, resnet=50, input_h=256, input_w=256)
    # model_det = YOLO("yolov8n.pt")
    # model_det.export(
    #     format="engine",
    #     dynamic=True,
    #     int8=True,
    #     imgsz=480,
    # )
    model_det = YOLO("yolov8n.engine", task="detect")
    print("start predict")
    tall0 = time.time()
    for img_path in path_list:
        frame = cv2.imread(img_path)
        frame_p = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        t0 = time.time()
        results = model_det.predict(
            img_path, save=False, imgsz=320, classes=0, conf=0.3
        )
        tyolo = time.time()
        bbox = results[0].boxes.xyxy[0]
        # bbox = get_max_bbox_ratio(bbox.clone(), 1)
        # bbox = adjust_bbox(bbox, frame.shape[1], frame.shape[0])
        bbox = bbox_transform(bbox.clone(), 1, frame.shape[1], frame.shape[0])
        tbbox = time.time()
        coords, scorse = model.predict(frame_p, bbox)
        tm = time.time()
        # Convert coords to NumPy array only if necessary for CPU operations
        coords_3d = coords.clone()
        coords_3d[:, 2] = coords_3d[:, 2] * 2000 + 2000
        # coords_3d[:, :2] = coords[:, :2] * (2000 / (bbox[2] - bbox[0]))
        coords_3d = pixel2cam(coords_3d, f, c)
        coords_3d = coords_3d - coords_3d[0]
        tout = time.time()
        # print(
        #     f"yolo={(tyolo-t0)* 1000:.3f}, bbox={(tbbox-tyolo)* 1000:.3f}, predict={(tm-tbbox)* 1000:.3f}, process={(tout-tm)* 1000:.3f}, all={(tout-t0)* 1000:.3f} ms"
        # )
    print(time.time()-tall0)
    fps = 1 / (tout - t0)  # fps = 1 / tout
    frame_show = model.vis_keypoints(frame, coords.cpu(), scorse.cpu())
    frame_show = cv2.putText(
        frame_show,
        f"fps= {fps:.2f}, t={(tout - t0) * 1000:.3f} ms",
        (0, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0),
        2,
    )
    bbox = bbox.int()
    x1, y1, x2, y2 = bbox.tolist()
    cv2.rectangle(frame_show, (x1, y1), (x2, y2), (0, 255, 0), 2)
    # cv2.imshow("frame", frame_show)
    cv2.imwrite(f".test_2d.jpg", frame_show)
    model.vis_keypoints_3d(coords_3d, f"test_3d.jpg")

    pdb.set_trace()
    print("human pose estimate complete")
