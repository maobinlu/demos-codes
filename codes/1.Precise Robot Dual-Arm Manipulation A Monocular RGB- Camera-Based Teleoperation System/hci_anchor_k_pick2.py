import dis
from turtle import distance
from mediapipe.tasks.python import vision
from mediapipe.tasks import python
import mediapipe as mp
import matplotlib.pyplot as plt
import torch
import os
from models.attention_smooth_1v1_lite import as1v1
from models.regression_nf_3d_hci_3vm2_best_lite import RegressFlow3D_hci_3vm2_best_lite
from models.hcipsoe_finnal import RegressFlow3D_cbam
import numpy as np
import cv2
from requests import get
from filter import Filter
from kalman_filter import KalmanFilter
from cam_utils import pixel2cam, bbox_transform, cam2pixel, get_bbox_from_kps
from AITEPose import AITEPose
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray, Bool
from sensor_msgs.msg import Image
import rospy
import pdb
from ultralytics import YOLO
import time
import matplotlib
matplotlib.use('TkAgg')
os.chdir("/home/aloha/interbotix_ws/src/hcipose_lite")


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

        self.capture = cv2.VideoCapture(0)  # 使用相机
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
        self.templist_1 = []
        self.templist_2 = []
        self.templist_hpe = []
        self.templist_smooth = []
        self.templist_nkf = []
        self.scorelist = []
        self.bbox_list = []
        self.box_filter = Filter(s=32, n=4, r=1)
        self.pose_filter = Filter(s=64, n=17, d=3, r=1)
        self.kf_x_r = KalmanFilter(dim_x=3, dim_z=1, var=0.02)
        self.kf_y_r = KalmanFilter(dim_x=3, dim_z=1, var=0.02)
        self.kf_z_r = KalmanFilter(dim_x=3, dim_z=1, var=0.02)
        self.kf_x_l = KalmanFilter(dim_x=3, dim_z=1, var=0.02)
        self.kf_y_l = KalmanFilter(dim_x=3, dim_z=1, var=0.02)
        self.kf_z_l = KalmanFilter(dim_x=3, dim_z=1, var=0.02)
        self.hand_point_r = np.zeros((3, 1))
        self.hand_point_l = np.zeros((3, 1))
        self.origin_2d_r = None
        self.origin_3d_r = None
        self.origin_2d_l = None
        self.origin_3d_l = None

        rospy.init_node("hci_node", anonymous=True)
        self.pose_pub = rospy.Publisher("hand_pose", Float32MultiArray, queue_size=1)
        self.hand_gesture_pub_r = rospy.Publisher("hand_gesture_r", Bool, queue_size=1)  # 新的发布者
        self.hand_gesture_pub_l = rospy.Publisher("hand_gesture_l", Bool, queue_size=1)  # 新的发布者

        # mediapipe手势识别
        BaseOptions = mp.tasks.BaseOptions
        GestureRecognizer = mp.tasks.vision.GestureRecognizer
        GestureRecognizerOptions = mp.tasks.vision.GestureRecognizerOptions
        GestureRecognizerResult = mp.tasks.vision.GestureRecognizerResult
        VisionRunningMode = mp.tasks.vision.RunningMode
        options = GestureRecognizerOptions(
            base_options=BaseOptions(model_asset_path='model_weight/gesture_recognizer.task'),
            running_mode=VisionRunningMode.LIVE_STREAM,
            result_callback=self.gesture_callback_r)
        self.recognizer_r = GestureRecognizer.create_from_options(options)

        options = GestureRecognizerOptions(
            base_options=BaseOptions(model_asset_path='model_weight/gesture_recognizer.task'),
            running_mode=VisionRunningMode.LIVE_STREAM,
            result_callback=self.gesture_callback_l)
        self.recognizer_l = GestureRecognizer.create_from_options(options)
        self.pick_r = False
        self.pick_num_r = 0
        self.pick_l = False
        self.pick_num_l = 0

    def gesture_callback_r(self, result, output_image, timestamp_ms):
        top_gesture = result.gestures
        points = result.hand_landmarks
        if len(top_gesture) == 0:
            return
        finger_tips = np.array([[points[0][4].x, points[0][4].y, points[0][4].z],
                                [points[0][8].x, points[0][8].y, points[0][8].z],
                                [points[0][12].x, points[0][12].y, points[0][12].z],
                                [points[0][16].x, points[0][16].y, points[0][16].z],
                                [points[0][20].x, points[0][20].y, points[0][20].z]])
        finger_jonts = np.array([[points[0][3].x, points[0][3].y, points[0][3].z],
                                [points[0][6].x, points[0][6].y, points[0][6].z],
                                [points[0][10].x, points[0][10].y, points[0][10].z],
                                [points[0][14].x, points[0][14].y, points[0][14].z],
                                [points[0][18].x, points[0][18].y, points[0][18].z]])
        palm_base = np.array([points[0][0].x, points[0][0].y, points[0][0].z])
        distances_tips = np.array([np.linalg.norm(tip - palm_base) for tip in finger_tips])
        distances_jonts = np.array([np.linalg.norm(jont - palm_base) for jont in finger_jonts])
        print(distances_tips.mean())
        print(distances_jonts.mean())
        # print(related_l)
        print("sate: pick" if self.pick_r else "sate: free")
        if distances_tips.mean() < 0.08 or distances_tips.mean() < distances_jonts.mean():  # 阈值可以根据需要调整
            print("deceted pick")

            if self.pick_r == False:
                self.pick_num_r += 1
            else:
                self.pick_num_r = 0

            if self.pick_num_r == 3:
                print("from free to pcik")
                self.hand_gesture_pub_r.publish(True)  # 合
                self.pick_r = True
                self.pick_num_r = 0
        else:
            print("deceted free")
            if self.pick_r == True:
                self.pick_num_r += 1
            else:
                self.pick_num_r = 0
            if self.pick_num_r == 3:
                print("from pick to free")
                self.hand_gesture_pub_r.publish(False)
                self.pick_r = False
                self.pick_num_r = 0

    def gesture_callback_l(self, result, output_image, timestamp_ms):
        top_gesture = result.gestures
        points = result.hand_landmarks
        if len(top_gesture) == 0:
            return
        finger_tips = np.array([[points[0][4].x, points[0][4].y, points[0][4].z],
                                [points[0][8].x, points[0][8].y, points[0][8].z],
                                [points[0][12].x, points[0][12].y, points[0][12].z],
                                [points[0][16].x, points[0][16].y, points[0][16].z],
                                [points[0][20].x, points[0][20].y, points[0][20].z]])
        finger_jonts = np.array([[points[0][3].x, points[0][3].y, points[0][3].z],
                                [points[0][6].x, points[0][6].y, points[0][6].z],
                                [points[0][10].x, points[0][10].y, points[0][10].z],
                                [points[0][14].x, points[0][14].y, points[0][14].z],
                                [points[0][18].x, points[0][18].y, points[0][18].z]])
        palm_base = np.array([points[0][0].x, points[0][0].y, points[0][0].z])
        distances_tips = np.array([np.linalg.norm(tip - palm_base) for tip in finger_tips])
        distances_jonts = np.array([np.linalg.norm(jont - palm_base) for jont in finger_jonts])
        print(distances_tips.mean())
        print(distances_jonts.mean())
        # print(related_l)
        print("sate: pick" if self.pick_l else "sate: free")
        if distances_tips.mean() < 0.08 or distances_tips.mean() < distances_jonts.mean():  # 阈值可以根据需要调整
            print("deceted pick")

            if self.pick_l == False:
                self.pick_num_l += 1
            else:
                self.pick_num_l = 0

            if self.pick_num_l == 3:
                print("from free to pcik")
                self.hand_gesture_pub_l.publish(True)  # 合
                self.pick_l = True
                self.pick_num_l = 0
        else:
            print("deceted free")
            if self.pick_l == True:
                self.pick_num_l += 1
            else:
                self.pick_num_l = 0
            if self.pick_num_l == 3:
                print("from pick to free")
                self.hand_gesture_pub_l.publish(False)
                self.pick_l = False
                self.pick_num_l = 0

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
            self.coords_3d_hpe = coords_3d[:17, :]
            coords_3d = self.model.smooth(coords_3d[:17, :])  # ----至此之后是17关节-----
            # coords_3d = coords_3d[:17, :]
            self.coords_3d_smooth = coords_3d
            if i == 0:
                self.pose_filter.init_history(coords_3d)
            coords_3d = self.pose_filter.update(coords_3d)
            # self.coords_3d_filter = coords_3d

            # convert 3d to 2d
            coords_2d = coords_3d+coords_root
            coords_2d = cam2pixel(coords_2d, self.f, self.c)

            if self.origin_2d_r is None and self.detector == 'anchor':
                self.origin_2d_r = coords_2d[0]-coords_2d[7]+coords_2d[14]+(coords_2d[14]-coords_2d[11])/2
            if self.origin_2d_l is None and self.detector == 'anchor':
                self.origin_2d_l = coords_2d[0]-coords_2d[7]+coords_2d[11]+(coords_2d[11]-coords_2d[14])/2

            frame_show = self.model.vis_keypoints(frame, coords_2d.cpu(), scores.cpu(), kp_thresh=0.0)

            # 绘制坐标系
            if self.detector == 'anchor':
                frame_show = cv2.line(
                    frame_show,
                    (int(self.origin_2d_r[0]), int(self.origin_2d_r[1])),
                    (int(self.origin_2d_r[0]) + 50, int(self.origin_2d_r[1])),  # X 轴延伸 100 像素
                    (255, 0, 0),  # 蓝色
                    2
                )
                frame_show = cv2.line(
                    frame_show,
                    (int(self.origin_2d_r[0]), int(self.origin_2d_r[1])),
                    (int(self.origin_2d_r[0]), int(self.origin_2d_r[1])-50),  # Y 轴延伸 100 像素
                    (0, 0, 255),  # 红色
                    2
                )
                frame_show = cv2.circle(
                    frame_show,
                    (int(self.origin_2d_r[0]), int(self.origin_2d_r[1])),  # 圆心坐标
                    5,  # 半径
                    (0, 255, 0),  # 绿色
                    -1  # 填充圆
                )
                frame_show = cv2.line(
                    frame_show,
                    (int(self.origin_2d_l[0]), int(self.origin_2d_l[1])),
                    (int(self.origin_2d_l[0]) + 50, int(self.origin_2d_l[1])),  # X 轴延伸 100 像素
                    (255, 0, 0),  # 蓝色
                    2
                )
                frame_show = cv2.line(
                    frame_show,
                    (int(self.origin_2d_l[0]), int(self.origin_2d_l[1])),
                    (int(self.origin_2d_l[0]), int(self.origin_2d_l[1])-50),  # Y 轴延伸 100 像素
                    (0, 0, 255),  # 红色
                    2
                )
                frame_show = cv2.circle(
                    frame_show,
                    (int(self.origin_2d_l[0]), int(self.origin_2d_l[1])),  # 圆心坐标
                    5,  # 半径
                    (0, 255, 0),  # 绿色
                    -1  # 填充圆
                )
            # self.out.write(frame_show)

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
                    move_x = 0
                    move_y = 0
                    if bbox_y[0] > bbox[0] and bbox_y[1] > bbox[1] and bbox_y[2] < bbox[2] and bbox_y[3] < bbox[3]:
                        pass
                    else:
                        print('out bbox')
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

                if self.origin_3d_r is None:
                    self.origin_3d_r = coords_3d[0]-coords_3d[7]+coords_3d[14]+(coords_3d[14]-coords_3d[11])/2
                if self.origin_3d_l is None:
                    self.origin_3d_l = coords_3d[0]-coords_3d[7]+coords_3d[11]+(coords_3d[11]-coords_3d[14])/2
                # 完整处理的数据
                self.hand_point_r = coords_3d[16] - self.origin_3d_r
                self.hand_point_r = self.hand_point_r[[0, 2, 1]]
                self.hand_point_r[2] = self.hand_point_r[2]*(-1)
                self.templist_nkf.append(self.hand_point_r.cpu().numpy())

                self.hand_point_l = coords_3d[13] - self.origin_3d_l
                self.hand_point_l = self.hand_point_l[[0, 2, 1]]
                self.hand_point_l[2] = self.hand_point_l[2]*(-1)
                self.hand_point_l[0] = self.hand_point_l[0]*(-1)
                self.hand_point_l[1] = self.hand_point_l[1]*(-1)

                # 原始hpe数据
                self.hand_point_r_hpe = self.coords_3d_hpe[16]-self.origin_3d_r
                self.hand_point_r_hpe = self.hand_point_r_hpe[[2, 0, 1]]
                self.hand_point_r_hpe[2] = self.hand_point_r_hpe[2]*(-1)
                self.hand_point_r_hpe[0] = self.hand_point_r_hpe[0]*(-1)

                # 经过smooth的数据
                self.hand_point_r_smooth = self.coords_3d_smooth[16]-self.origin_3d_r
                self.hand_point_r_smooth = self.hand_point_r_smooth[[2, 0, 1]]
                self.hand_point_r_smooth[2] = self.hand_point_r_smooth[2]*(-1)
                self.hand_point_r_smooth[0] = self.hand_point_r_smooth[0]*(-1)

                # kalman filter的数据
                self.hand_point_r = self.hand_point_r.cpu().numpy()
                self.kf_y_r.predict()
                self.hand_point_r[1] = self.kf_y_r.update(self.hand_point_r[1])
                self.kf_z_r.predict()
                self.hand_point_r[2] = self.kf_z_r.update(self.hand_point_r[2])
                self.kf_x_r.predict()
                self.hand_point_r[0] = self.kf_x_r.update(self.hand_point_r[0])

                self.hand_point_l = self.hand_point_l.cpu().numpy()
                self.kf_y_l.predict()
                self.hand_point_l[1] = self.kf_y_l.update(self.hand_point_l[1])
                self.kf_z_l.predict()
                self.hand_point_l[2] = self.kf_z_l.update(self.hand_point_l[2])
                self.kf_x_l.predict()
                self.hand_point_l[0] = self.kf_x_l.update(self.hand_point_l[0])

                out_hand_pose = [self.hand_point_r[0], self.hand_point_r[1], self.hand_point_r[2], self.hand_point_l[0], self.hand_point_l[1], self.hand_point_l[2]]
                pub_pose = Float32MultiArray()
                pub_pose.data = out_hand_pose
                self.pose_pub.publish(pub_pose)

                self.templist.append(self.hand_point_r)
                self.templist_1.append(self.hand_point_l)
                self.templist_hpe.append(self.hand_point_r_hpe.cpu().numpy())
                self.templist_smooth.append(self.hand_point_r_smooth.cpu().numpy())

                self.scorelist.append(scores)
                self.bbox_list.append(bbox)

                # 利用mediapip估计手势
                if i % 10 == 0:
                    # coords_2d_hand_r = (coords_2d[16] - coords_2d[15])/(3) + coords_2d[16]
                    coords_2d_hand_r = coords_2d[16]
                    x1, y1, x2, y2 = int(coords_2d_hand_r[0]-100), int(coords_2d_hand_r[1]-100), int(coords_2d_hand_r[0]+100), int(coords_2d_hand_r[1]+100)
                    hand_img = frame_p[y1:y2, x1:x2]
                    hand_img = np.array(hand_img, dtype=np.uint8)
                    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=hand_img)
                    self.recognizer_r.recognize_async(mp_image, i)

                    coords_2d_hand_l = coords_2d[13]
                    x1, y1, x2, y2 = int(coords_2d_hand_l[0]-100), int(coords_2d_hand_l[1]-100), int(coords_2d_hand_l[0]+100), int(coords_2d_hand_l[1]+100)
                    hand_img = frame_p[y1:y2, x1:x2]
                    hand_img = np.array(hand_img, dtype=np.uint8)
                    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=hand_img)
                    self.recognizer_l.recognize_async(mp_image, i)
            t = time.time() - t1
            time_list.append(time.time())
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
            cv2.imshow("frame", frame_show)
            img_list.append(frame_show)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("exit")
                break
            i = i + 1
        self.capture.release()
        # 如果文件夹不存在则创建
        if not os.path.exists(action_name):
            os.makedirs(action_name)
            os.makedirs(action_name+'/img')
        # 保存时间戳
        for i in range(len(img_list)):
            cv2.imwrite(action_name+'/img/'+f'{i}.jpg', img_list[i])
        np.save(action_name+'/time.npy', time_list, allow_pickle=True)

    def show_trajectory(self, *templist):
        fig = plt.figure()
        num_plots = len(templist)

        for i, temp in enumerate(templist):
            ax = fig.add_subplot(2, int(num_plots/2), i + 1, projection="3d")
            n = len(temp)
            hand_point_rs = np.zeros((n, 3))
            for j, coord in enumerate(temp):
                hand_point_rs[j] = coord
            ax.plot(hand_point_rs[:, 0], hand_point_rs[:, 1], hand_point_rs[:, 2], color="blue")
            ax.set_xlabel('X 轴')
            ax.set_ylabel('Y 轴')
            ax.set_zlabel('Z 轴')
            ax.set_title(f'轨迹 {i + 1}')  # 添加标题
            # ax.invert_yaxis()
            # 保存列表
            np.save(action_name+f'/hand_trajectory_{i}.npy', hand_point_rs)

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
f = [451.477162, 453.875205]  # aloha 电脑参数
c = [272.213870, 294.493310]
action_name = 'save_data/ntest2'
if __name__ == "__main__":
    hpe = HPE_3D(dict_path_152, smoother_weight, joint_num, 7, f, c, resnet=152)
    hpe.infer()
    hpe.show_trajectory(hpe.templist, hpe.templist_1, hpe.templist_nkf,  hpe.templist_smooth)
