#! /usr/bin/env python3
import dis
from re import A
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
os.chdir("/home/aloha-pc/interbotix_ws/src/hcipose_lite")
from piper_msgs.msg import PiperStatusMsg, PosCmd

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
        # self.capture.set(cv2.CAP_PROP_FOCUS, 250)
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
        self.pose_filter = Filter(s=64, n=17, d=3, r=0.9)
        self.kf_x = KalmanFilter(dim_x=3, dim_z=1, var=0.02)
        self.kf_x_2 = KalmanFilter(dim_x=3, dim_z=1, var=20)
        self.kf_x_3 = KalmanFilter(dim_x=3, dim_z=1, var=0.2)
        self.kf_y = KalmanFilter(dim_x=3, dim_z=1, var=0.02)
        self.kf_z = KalmanFilter(dim_x=3, dim_z=1, var=0.02)
        self.hand_point = np.zeros((3, 1))
        self.origin_2d = None
        self.origin_3d = None

        rospy.init_node("hci_node", anonymous=True)
        # self.pose_pub = rospy.Publisher("hand_pose", Float32MultiArray, queue_size=1)
        self.pose_pub = rospy.Publisher("/piper_right/pos_cmd", PosCmd, queue_size=1)
        self.hand_gesture_pub = rospy.Publisher("hand_gesture", Bool, queue_size=1)  # 新的发布者

        # mediapipe手势识别
        BaseOptions = mp.tasks.BaseOptions
        GestureRecognizer = mp.tasks.vision.GestureRecognizer
        GestureRecognizerOptions = mp.tasks.vision.GestureRecognizerOptions
        GestureRecognizerResult = mp.tasks.vision.GestureRecognizerResult
        VisionRunningMode = mp.tasks.vision.RunningMode
        options = GestureRecognizerOptions(
            base_options=BaseOptions(model_asset_path='model_weight/gesture_recognizer.task'),
            running_mode=VisionRunningMode.LIVE_STREAM,
            result_callback=self.gesture_callback)
        self.recognizer = GestureRecognizer.create_from_options(options)
        self.pick = False
        self.pick_num = 0

    def gesture_callback(self, result, output_image, timestamp_ms):
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
        # related_l = np.linalg.norm(np.array([points[0][5].x, points[0][5].y, points[0][5].z]) - np.array([points[0][17].x, points[0][17].y, points[0][17].z]))
        # print(finger_tips)
        # print(palm_base)
        # distances = [np.linalg.norm(tip - root) for tip, root in zip(finger_tips, finger_roots)]
        distances_tips = np.array([np.linalg.norm(tip - palm_base) for tip in finger_tips])
        distances_jonts = np.array([np.linalg.norm(jont - palm_base) for jont in finger_jonts])
        # print(distances_tips.mean())
        # print(distances_jonts.mean())
        # print(related_l)
        print("sate: pick" if self.pick else "sate: free")
        if distances_tips.mean() < 0.08 or distances_tips.mean() < distances_jonts.mean():  # 阈值可以根据需要调整
            print("deceted pick")

            if self.pick == False:
                self.pick_num += 1
            else:
                self.pick_num = 0

            if self.pick_num == 3:
                print("from free to pcik")
                self.hand_gesture_pub.publish(True)  # 合
                self.pick = True
                self.pick_num = 0
        else:
            print("deceted free")
            if self.pick == True:
                self.pick_num += 1
            else:
                self.pick_num = 0
            if self.pick_num == 3:
                print("from pick to free")
                self.hand_gesture_pub.publish(False)
                self.pick = False
                self.pick_num = 0

    def infer(self, function=None):
        print("start predict")
        fps = 0
        i = 0
        time_list = []
        img_list = []
        
        # gt_list = np.load('save_data/x1/save_mc.npy', allow_pickle=True)
        # for k in range(len(gt_list)):
        #     gt_list[k] = gt_list[k][[1, 0, 2]]
        #     gt_list[k][ 0] = gt_list[k][0] * (-1)
        # self.gt_list = []
        
        while True:
            t1 = time.time()
            # self.capture = cv2.VideoCapture(f'save_data/x1/img/{i}.jpg')
            # if i > 300:
            #     break
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
            # print(coords_3d[14])
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
                    results = self.model_det.predict(frame, save=False, imgsz=320, classes=0, conf=0.3,verbose=False)
                    if results[0].boxes.xyxy.numel() == 0:
                        self.start_seq = False
                        self.model.frame_num = 0
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

                self.model.frame_num = self.model.frame_num + 1
                if self.origin_3d is None:
                    self.origin_3d = coords_3d[0]-coords_3d[7]+coords_3d[14]+(coords_3d[14]-coords_3d[11])/2
                # print(self.bbox_anchor)
                # 完整处理的数据
                self.hand_point = coords_3d[16] - self.origin_3d
                self.hand_point = self.hand_point[[2, 0, 1]]
                self.hand_point[2] = self.hand_point[2]*(-1)
                self.hand_point[0] = self.hand_point[0]*(-1)
                self.templist_nkf.append(self.hand_point.cpu().numpy())
                
                # 原始hpe数据
                self.hand_point_hpe = self.coords_3d_hpe[16]-self.origin_3d
                self.hand_point_hpe = self.hand_point_hpe[[2, 0, 1]]
                self.hand_point_hpe[2] = self.hand_point_hpe[2]*(-1)
                self.hand_point_hpe[0] = self.hand_point_hpe[0]*(-1)

                # 经过smooth的数据
                self.hand_point_smooth = self.coords_3d_smooth[16]-self.origin_3d
                self.hand_point_smooth = self.hand_point_smooth[[2, 0, 1]]
                self.hand_point_smooth[2] = self.hand_point_smooth[2]*(-1)
                self.hand_point_smooth[0] = self.hand_point_smooth[0]*(-1)

                # kalman filter的数据
                self.hand_point = self.hand_point.cpu().numpy()
                self.kf_y.predict()
                self.hand_point[1] = self.kf_y.update(self.hand_point[1])
                self.kf_z.predict()
                self.hand_point[2] = self.kf_z.update(self.hand_point[2])

                self.kf_x.predict()
                self.kf_x_2.predict()
                self.kf_x_3.predict()
                self.hand_point_1 = self.hand_point
                self.hand_point_1[0] = self.kf_x_2.update(self.hand_point[0])
                self.hand_point_2 = self.hand_point
                self.hand_point_2[0] = self.kf_x_3.update(self.hand_point[0])
                self.hand_point[0] = self.kf_x.update(self.hand_point[0])

                self.templist.append(self.hand_point)
                self.templist_1.append(self.hand_point_1)
                self.templist_2.append(self.hand_point_2)
                self.templist_hpe.append(self.hand_point_hpe.cpu().numpy())
                self.templist_smooth.append(self.hand_point_smooth.cpu().numpy())

                self.scorelist.append(scores)
                self.bbox_list.append(bbox)
                
                # out_hand_pose = [self.hand_point[0], self.hand_point[1], self.hand_point[2]]
                # pub_pose = Float32MultiArray()
                # pub_pose.data = out_hand_pose
                # self.pose_pub.publish(pub_pose)
                
                out_hand_pose = [self.hand_point_smooth[0], self.hand_point_smooth[1], self.hand_point_smooth[2]]
                pub_pose = PosCmd()
                
                # 正映射
                # pub_pose.x = out_hand_pose[0]
                # pub_pose.y = out_hand_pose[1]
                # pub_pose.z = out_hand_pose[2]
                # pub_pose.roll = 0
                # pub_pose.pitch = 1.57
                # pub_pose.yaw = 0
                # # pub_pose.data = out_hand_pose
                # # spatial_mapping
                # pub_pose.x = pub_pose.x/1000 *4/5+0.05
                # pub_pose.y = pub_pose.y/1000*3/4
                # pub_pose.z = pub_pose.z/1000*5/8 +0.45
                # self.pose_pub.publish(pub_pose)
                
                # 侧映射
                pub_pose.x = out_hand_pose[1]
                pub_pose.y = -out_hand_pose[0]
                pub_pose.z = out_hand_pose[2]
                pub_pose.roll = 0
                pub_pose.pitch = 1.57
                pub_pose.yaw = 0
                pub_pose.x = pub_pose.x/1000*1/2+0.25
                pub_pose.y = pub_pose.y/1000 + 0.25
                # pub_pose.z = pub_pose.z/1000*5/8 +0.45 
                #抓取映射
                pub_pose.z = pub_pose.z/1000 +0.4
                if pub_pose.z < 0.108:
                    pub_pose.z = 0.108
                if pub_pose.z < 0.4:
                    pub_pose.pitch=1.57+(0.4-pub_pose.z)*0.83/0.3
                    # print('pitch',pub_pose.pitch)
                self.pose_pub.publish(pub_pose)
                # rospy.sleep(0.01) #

                # 利用mediapip估计手势
                # if i % 10 == 0:
                #     # coords_2d_hand = (coords_2d[16] - coords_2d[15])/(3) + coords_2d[16]
                #     coords_2d_hand = coords_2d[16]
                #     x1, y1, x2, y2 = int(coords_2d_hand[0]-100), int(coords_2d_hand[1]-100), int(coords_2d_hand[0]+100), int(coords_2d_hand[1]+100)
                #     hand_img = frame_p[y1:y2, x1:x2]
                #     hand_img = np.array(hand_img, dtype=np.uint8)
                #     mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=hand_img)
                #     self.recognizer.recognize_async(mp_image, i)
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

# f = [409.638077, 410.666366]
# c = [340.533727, 195.370445]
 
f = [640.234928-10, 639.647742-12]
# c = [310.815142, 259.667876-20]
c = [320,240]
action_name = 'save_data/ntest2'
if __name__ == "__main__":
    hpe = HPE_3D(dict_path_152, smoother_weight, joint_num, 27, f, c, resnet=152)
    # hpe = HPE_3D(dict_path_50, smoother_weight, joint_num, 27, f, c, resnet=50)
    hpe.infer()
    hpe.show_trajectory2([hpe.templist_hpe,hpe.templist_nkf],[0,1],[0,200])
