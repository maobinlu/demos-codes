'''
by yzh 2022.2.13
'''
import roslib
import rospy
from std_msgs.msg import Header
from std_msgs.msg import String
from geometry_msgs.msg import Point
from rs_yolo.msg import Info

import sys
sys.path.append(r"/home/nuc/arm807_hand_ws/src/robot_arm_gripper/rs_yolo/scripts"); 
sys.path.append(r"/home/nuc/arm807_hand_ws/src/robot_arm_gripper/rs_yolo/scripts/weights"); 

# sys.path.append(r"./scripts")

from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image,CameraInfo
import pyrealsense2 as rs


# 导入依赖
import random
from utils.torch_utils import select_device, load_classifier, time_sync
from utils.general import (
    check_img_size, non_max_suppression, apply_classifier, scale_coords,
    xyxy2xywh, strip_optimizer, set_logging)
from utils.datasets import LoadStreams, LoadImages, letterbox
from models.experimental import attempt_load
import torch.backends.cudnn as cudnn
import torch

import pyrealsense2 as rs
import math
import yaml
import argparse
import os
import time
import numpy as np
#import sys

import cv2

intrinsics=None
color_image=None
depth_image=None

class YoloV5:
    def __init__(self, yolov5_yaml_path='/home/nuc/arm807_hand_ws/src/robot_arm_gripper/rs_yolo/scripts/config/yolov5x.yaml'):
        '''初始化'''
        # 载入配置文件
        with open(yolov5_yaml_path, 'r', encoding='utf-8') as f:
            self.yolov5 = yaml.load(f.read(), Loader=yaml.SafeLoader)
        # 随机生成每个类别的颜色
        self.colors = [[np.random.randint(0, 255) for _ in range(
            3)] for class_id in range(self.yolov5['class_num'])]
        # 模型初始化
        self.init_model()

    @torch.no_grad()
    def init_model(self):
        '''模型初始化'''
        # 设置日志输出
        set_logging()
        # 选择计算设备
        device = select_device(self.yolov5['device'])
        # 如果是GPU则使用半精度浮点数 F16
        is_half = device.type != 'cpu'
        # 载入模型
        model = attempt_load(
            self.yolov5['weight'], map_location=device)  # 载入全精度浮点数的模型
        input_size = check_img_size(
            self.yolov5['input_size'], s=model.stride.max())  # 检查模型的尺寸
        if is_half:
            model.half()  # 将模型转换为半精度
        # 设置BenchMark，加速固定图像的尺寸的推理
        cudnn.benchmark = True  # set True to speed up constant image size inference
        # 图像缓冲区初始化
        img_torch = torch.zeros(
            (1, 3, self.yolov5['input_size'], self.yolov5['input_size']), device=device)  # init img
        # 创建模型
        # run once
        _ = model(img_torch.half()
                  if is_half else img) if device.type != 'cpu' else None
        self.is_half = is_half  # 是否开启半精度
        self.device = device  # 计算设备
        self.model = model  # Yolov5模型
        self.img_torch = img_torch  # 图像缓冲区

    def preprocessing(self, img):
        '''图像预处理'''
        # 图像缩放
        # 注: auto一定要设置为False -> 图像的宽高不同
        img_resize = letterbox(img, new_shape=(
            self.yolov5['input_size'], self.yolov5['input_size']), auto=False)[0]
        # print("img resize shape: {}".format(img_resize.shape))
        # 增加一个维度
        img_arr = np.stack([img_resize], 0)
        # 图像转换 (Convert) BGR格式转换为RGB
        # 转换为 bs x 3 x 416 x
        # 0(图像i), 1(row行), 2(列), 3(RGB三通道)
        # ---> 0, 3, 1, 2
        # BGR to RGB, to bsx3x416x416
        img_arr = img_arr[:, :, :, ::-1].transpose(0, 3, 1, 2)
        # 数值归一化
        # img_arr =  img_arr.astype(np.float32) / 255.0
        # 将数组在内存的存放地址变成连续的(一维)， 行优先
        # 将一个内存不连续存储的数组转换为内存连续存储的数组，使得运行速度更快
        # https://zhuanlan.zhihu.com/p/59767914
        img_arr = np.ascontiguousarray(img_arr)
        return img_arr

    @torch.no_grad()
    def detect(self, img, canvas=None, view_img=True):
        '''模型预测'''
        # 图像预处理
        img_resize = self.preprocessing(img)  # 图像缩放
        self.img_torch = torch.from_numpy(img_resize).to(self.device)  # 图像格式转换
        self.img_torch = self.img_torch.half(
        ) if self.is_half else self.img_torch.float()  # 格式转换 uint8-> 浮点数
        self.img_torch /= 255.0  # 图像归一化
        if self.img_torch.ndimension() == 3:
            self.img_torch = self.img_torch.unsqueeze(0)
        # 模型推理
        t1 = time_sync()
        pred = self.model(self.img_torch, augment=False)[0]
        # pred = self.model_trt(self.img_torch, augment=False)[0]
        # NMS 非极大值抑制
        pred = non_max_suppression(pred, self.yolov5['threshold']['confidence'],
                                   self.yolov5['threshold']['iou'], classes=None, agnostic=False)
        t2 = time_sync()
        # print("推理时间: inference period = {}".format(t2 - t1))
        # 获取检测结果
        det = pred[0]
        gain_whwh = torch.tensor(img.shape)[[1, 0, 1, 0]]  # [w, h, w, h]

        if view_img and canvas is None:
            canvas = np.copy(img)
        xyxy_list = []
        conf_list = []
        class_id_list = []
        if det is not None and len(det):
            # 画面中存在目标对象
            # 将坐标信息恢复到原始图像的尺寸
            det[:, :4] = scale_coords(
                img_resize.shape[2:], det[:, :4], img.shape).round()
            for *xyxy, conf, class_id in reversed(det):
                class_id = int(class_id)
                xyxy_list.append(xyxy)
                conf_list.append(conf)
                class_id_list.append(class_id)
                if view_img:
                    # 绘制矩形框与标签
                    label = '%s %.2f' % (
                        self.yolov5['class_name'][class_id], conf)
                    print(label.split()[0])
                    if label.split()[0] in ['cup','bottle']:
                        self.plot_one_box(
                        xyxy, canvas, label=label, color=self.colors[class_id], line_thickness=2)
        return canvas, class_id_list, xyxy_list, conf_list

    def plot_one_box(self, x, img, color=None, label=None, line_thickness=None):
        ''''绘制矩形框+标签'''
        tl = line_thickness or round(
            0.002 * (img.shape[0] + img.shape[1]) / 2) + 1  # line/font thickness
        color = color or [random.randint(0, 255) for _ in range(3)]
        c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
        cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
        if label:
            tf = max(tl - 1, 1)  # font thickness
            t_size = cv2.getTextSize(
                label, 0, fontScale=tl / 3, thickness=tf)[0]
            c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
            cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
            cv2.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 3,
                        [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)

def publish_image(real_x, real_y, real_z, classification, confidence):
    detect_result=Info()
    rate = rospy.Rate(30)
    header = Header(stamp=rospy.Time.now())
    header.frame_id = 'map'

    detect_result.x = real_x
    detect_result.y = real_y
    detect_result.z = real_z
    detect_result.classification = classification
    detect_result.confidence = confidence

    pub.publish(detect_result)
    rate.sleep()
    # rospy.loginfo("x:%.3f,  y:%.3f,  z:%.3f,  classification:%s: ", detect_result.x, detect_result.y,  detect_result.z, detect_result.classification)


# PyTorch
# YoloV5-PyTorch
bridge = CvBridge()

# def callback(depth_data, color_data, depth_info, color_info):
#     try:
#         depth_image = bridge.imgmsg_to_cv2(depth_data, "passthrough")
#         color_image = bridge.imgmsg_to_cv2(color_data, "bgr8")
#         depth_camera_info = bridge.imgmsg_to_cv2(depth_info, "32FC1")
#         color_camera_info = bridge.imgmsg_to_cv2(color_info, "32FC1")
#     except CvBridgeError as e:
#         print(e)

#     depth_image = depth_image.astype(float)
#     depth_image /= 256.0
#     depth_image = cv2.convertScaleAbs(depth_image)

#     # intr=[ 848x480  p[425.593 243.186]  f[605.373 604.997]  Inverse Brown Conrady [0 0 0 0 0] ]
#     # depth_intrin=[ 848x480  p[425.593 243.186]  f[605.373 604.997]  Inverse Brown Conrady [0 0 0 0 0] ]
#     # aligned_depth_frame="<pyrealsense2.frame Z16 #231 @1679388082715.406250> "
#     cv2.imshow("Depth Image", depth_image)
#     cv2.imshow("Color Image", color_image)
#     cv2.waitKey(1)

#     return depth_intrin, color_image, depth_image, aligned_depth_frame

def imageColorCallback(color_data):
    try:
        global color_image
        color_image = bridge.imgmsg_to_cv2(color_data, "bgr8")
        
        # print("imageColorCallback finish")
    except CvBridgeError as e:
        print(e)

def imageDepthCallback(depth_data):
    try:
        global depth_image 
        depth_image = bridge.imgmsg_to_cv2(depth_data, depth_data.encoding)
       # print("depth_image_cvbridge")
       # print(depth_data.encoding)
        # pick one pixel among all the pixels with the closest range:
        indices = np.array(np.where(depth_image == depth_image[depth_image > 0].min()))[:,0]
        pix = (indices[1], indices[0])
        line = '\rDepth at pixel(%3d, %3d): %7.1f(mm).' % (pix[0], pix[1], depth_image[pix[1], pix[0]])

        # depth_image = depth_image.astype(float)
        # depth_image /= 256.0
        # depth_image = cv2.convertScaleAbs(depth_image)
        
        # print("imageDepthCallback finish")
    except CvBridgeError as e:
        print(e)
        return 
    except ValueError as e:
        return 
           
def imageDepthInfoCallback(cameraInfo):
    global intrinsics
    try:
        if intrinsics:
            return
        intrinsics = rs.intrinsics()
        intrinsics.width = cameraInfo.width
        intrinsics.height = cameraInfo.height
        intrinsics.ppx = cameraInfo.K[2]
        intrinsics.ppy = cameraInfo.K[5]
        intrinsics.fx = cameraInfo.K[0]
        intrinsics.fy = cameraInfo.K[4]
        if cameraInfo.distortion_model == 'plumb_bob':
            intrinsics.model = rs.distortion.brown_conrady
        elif cameraInfo.distortion_model == 'equidistant':
            intrinsics.model = rs.distortion.kannala_brandt4
        intrinsics.coeffs = [i for i in cameraInfo.D]

        # print("imageDepthInfoCallback finish")
    except CvBridgeError as e:
        print(e)
        return    

if __name__ == '__main__':
    rospy.init_node('ros_yolo')

    pub = rospy.Publisher("/detect_result_out", Info, queue_size=10)
    depth_sub=rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image,imageDepthCallback)
    color_sub=rospy.Subscriber("/camera/color/image_raw",Image,imageColorCallback)
    depth_info_sub = rospy.Subscriber("/camera/aligned_depth_to_color/camera_info",CameraInfo,imageDepthInfoCallback)
    #color_info_sub = rospy.Subscriber("/camera/color/camera_info",CameraInfo)    

    
    print("[INFO] YoloV5目标检测-程序启动")
    print("[INFO] 开始YoloV5模型加载")
    # YOLOV5模型配置文件(YAML格式)的路径 yolov5_yaml_path
    model = YoloV5(yolov5_yaml_path='/home/nuc/arm807_hand_ws/src/robot_arm_gripper/rs_yolo/scripts/config/yolov5x.yaml')


    while True:
        # Wait for a coherent pair of frames: depth and color
        # intr, depth_intrin, color_image, depth_image, aligned_depth_frame = callback()  # 获取对齐的图像与相机内参
        intr=intrinsics
        depth_intrin=intrinsics
    
        # if not depth_image.any() or not color_image.any():
        #     continue
        # Convert images to numpy arrays
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        #print("depth image type")
        #print(type(depth_image))
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
            depth_image, alpha=0.03), cv2.COLORMAP_JET)
        # Stack both images horizontally
        images = np.hstack((color_image, depth_colormap))
        
        # Show images

        t_start = time.time()  # 开始计时
        # YoloV5 目标检测
        canvas, class_id_list, xyxy_list, conf_list = model.detect(
            color_image)

        t_end = time.time()  # 结束计时\
        #canvas = np.hstack((canvas, depth_colormap))
        #print(class_id_list)

        camera_xyz_list=[]
        if xyxy_list:
            for i in range(len(xyxy_list)):
                ux = int((xyxy_list[i][0]+xyxy_list[i][2])/2)  # 计算像素坐标系的x
                uy = int((xyxy_list[i][1]+xyxy_list[i][3])/2)  # 计算像素坐标系的y
                dis = depth_image[uy, ux]
                #dis = aligned_depth_frame.get_distance(ux, uy)
                camera_xyz = rs.rs2_deproject_pixel_to_point(
                    depth_intrin, (ux, uy), dis)  # 计算相机坐标系的xyz
                camera_xyz = np.round(np.array(camera_xyz), 3)  # 转成3位小数
                camera_xyz = camera_xyz.tolist()
                cv2.circle(canvas, (ux,uy), 4, (255, 255, 255), 5)#标出中心点
                cv2.putText(canvas, str(camera_xyz), (ux+20, uy+10), 0, 1,
                            [225, 255, 255], thickness=1, lineType=cv2.LINE_AA)#标出坐标
                camera_xyz_list.append(camera_xyz)
        print(camera_xyz_list)


        for i in range(len(camera_xyz_list)):
            publish_image(camera_xyz_list[i][0], camera_xyz_list[i][1], camera_xyz_list[i][2], model.yolov5['class_name'][class_id_list[i]], conf_list[i])
            #publish_image(camera_xyz_list[i][0], camera_xyz_list[i][1], camera_xyz_list[i][2])
        
        
        # 添加fps显示
        fps = int(1.0 / (t_end - t_start))
        cv2.putText(canvas, text="FPS: {}".format(fps), org=(50, 50),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, thickness=1,
                    lineType=cv2.LINE_AA, color=(0, 0, 0))
        cv2.namedWindow('detection', flags=cv2.WINDOW_NORMAL |
                        cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
        cv2.imshow('detection', canvas)
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

    rospy.spin()
