
from turtle import title
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sympy import N
import torch
from kalman_filter import KalmanFilter
from filter import Filter
import pdb
import torch
import math


def show_trajectory(templist, cat=None, show_num=[0, -1]):
    fig = plt.figure()
    num_plots = len(templist)
    if cat != None:
        num_plots = num_plots+1
    len_num = math.ceil(num_plots/2)
    col_num = math.ceil(num_plots/len_num)
    for i, temp in enumerate(templist):
        ax = fig.add_subplot(len_num, col_num, i + 1, projection="3d")
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
        ax.plot(hand_points[show_num[0]:show_num[1], 0], hand_points[show_num[0]:show_num[1], 1], hand_points[show_num[0]:show_num[1], 2], color="blue")
        ax.plot(hand_points2[show_num[0]:show_num[1], 0], hand_points2[show_num[0]:show_num[1], 1], hand_points2[show_num[0]:show_num[1], 2], color="red")
        ax.set_xlabel('X 轴')
        ax.set_ylabel('Y 轴')
        ax.set_zlabel('Z 轴')
        ax.set_title(f'and')  # 添加标题
        ax.invert_yaxis()
    plt.show(block=False)
    plt.show()  # 稍作暂停，确保窗口显示


def error_analysis(temp_pred, temp_gt):
    fig = plt.figure(2)
    titles = {0: 'x', 1: 'y', 2: 'z'}
    for i in range(3):
        ax = fig.add_subplot(2, 3, i + 1)
        ax.plot(temp_pred[:, i], temp_gt[:, i]-temp_pred[:, i],  color="blue")
        ax.set_title(titles[i])  # 添加标题
    for i in range(3):
        min = int(np.min(temp_pred[:, i]))
        max = int(np.max(temp_pred[:, i]))
        error_dis = np.zeros((max-min, 2))
        for j in range(min, max, 1):
            indx = np.where((j <= temp_pred[:, i]) & (temp_pred[:, i] <= j+1))
            error = np.mean(temp_gt-temp_pred, axis=1)
            errors = error[indx]
            error_dis[j-min] = [j, np.mean(errors)]
        ax = fig.add_subplot(2, 3, i + 4)
        ax.plot(error_dis[:, 0], error_dis[:, 1],  color="blue")
        ax.set_title(titles[i]+'mean')
    plt.show(block=False)
    plt.show()  # 稍作暂停，确保窗口显示


def error_frequence(temp_pred, temp_gt):
    fig = plt.figure(2)
    titles = {0: 'x', 1: 'y', 2: 'z'}
    t = np.arange(0, len(temp_pred), 1)
    sample_freq = 1
    freq1 = sample_freq*t/len(temp_pred)/2        # 单边谱的频率轴
    # 展示xyz轴误差分量的频谱分析
    for i in range(3):
        ax = fig.add_subplot(3, 1, i + 1)
        error = temp_gt[:, i]-temp_pred[:, i]
        # 执行快速傅里叶变换（FFT）
        E_f = np.fft.fft(error-np.mean(error))  # 计算傅里叶变换
        frequencies = np.fft.fftfreq(len(t), 1)  # 计算对应的频率轴
        # 取傅里叶变换的正频率部分
        positive_frequencies = frequencies[:len(frequencies)]
        magnitude = np.abs(E_f)[:len(E_f)]  # 计算频谱的幅度（模）
        # 频谱图
        ax.plot(freq1, magnitude,  color="blue")

        gt = temp_gt[:, i]
        E_f = np.fft.fft(gt-np.mean(gt))  # 计算傅里叶变换
        frequencies = np.fft.fftfreq(len(t), 1)  # 计算对应的频率轴
        # 取傅里叶变换的正频率部分
        positive_frequencies = frequencies[:len(frequencies)]
        magnitude = np.abs(E_f)[:len(E_f)]  # 计算频谱的幅度（模）
        # 频谱图
        ax.plot(freq1, magnitude,  color="red")

        gt = temp_pred[:, i]
        E_f = np.fft.fft(gt-np.mean(gt))  # 计算傅里叶变换
        frequencies = np.fft.fftfreq(len(t), 1)  # 计算对应的频率轴
        # 取傅里叶变换的正频率部分
        positive_frequencies = frequencies[:len(frequencies)]
        magnitude = np.abs(E_f)[:len(E_f)]  # 计算频谱的幅度（模）
        # 频谱图
        ax.plot(freq1, magnitude,  color="green")
        plt.ylim(0, 4000)
    plt.show()


# 展示误差距离，输入为预测值和真实值
def show_error(temp_pred, temp_gt):
    fig = plt.figure(2)
    titles = {0: 'x', 1: 'y', 2: 'z'}
    ax = fig.add_subplot(4, 1, 1)
    error = np.sqrt(np.sum((temp_pred-temp_gt)**2, axis=1))
    t = np.arange(0, len(error), 1)
    ax.plot(t, error,  color="blue")
    ax.set_title(f'error_norm:{np.mean(error)}')  # 添加标题
    for i in range(3):
        ax = fig.add_subplot(4, 1, i + 2)
        ax.plot(t, temp_gt[:, i]-temp_pred[:, i],  color="blue")
        ax.set_title('error:'+titles[i])
    plt.show()


R = np.array([[9.87045566e-01, -1.35264977e-01, -8.62811506e-02],
              [1.34829228e-01,  9.90809061e-01, -1.08850342e-02],
              [8.69605097e-02, -8.89196189e-04,  9.96211363e-01]])
T = np.array([-95.3572745,  -45.94321624,  27.25388518])
scale = 0.9648176310192673

if __name__ == "__main__":
    kf_x = KalmanFilter(dim_x=3, dim_z=1, var=20)
    kf_y = KalmanFilter(dim_x=3, dim_z=1, var=0.02)
    kf_z = KalmanFilter(dim_x=3, dim_z=1, var=0.02)
    pose_filter = Filter(s=64, n=1, d=3, r=1)
    data_hpe = np.load('dataset/x1/save_hpe.npy')
    data_mc = np.load('dataset/x1/save_mc.npy')
    data_mc = data_mc[:, [1, 0, 2]]
    data_mc[:, 0] = data_mc[:, 0]*(-1)
    data_hpe = scale * (data_hpe @ R.T) + T
    data_hpe_k = data_hpe.copy()
    # pdb.set_trace()
    for i in range(len(data_hpe)):
        if i == 0:
            pose_filter.init_history(torch.tensor(data_hpe[i]))
        data_hpe_k[i] = (pose_filter.update(torch.tensor(data_hpe[i]))).numpy()
        kf_x.predict()
        data_hpe_k[i, 0] = kf_x.update(data_hpe_k[i, 0])
        kf_y.predict()
        data_hpe_k[i, 1] = kf_y.update(data_hpe_k[i, 1])
        kf_z.predict()
        data_hpe_k[i, 2] = kf_z.update(data_hpe_k[i, 2])
    # pdb.set_trace()
    show_trajectory([data_hpe, data_hpe_k, data_mc], [0, 2], [0, 1500])
    show_error(data_hpe, data_mc)
    step = 10
    data_hpe_v = data_hpe_k[step:, :] - data_hpe_k[:-step, :]
    data_mc_v = data_mc[step:, :] - data_mc[:-step, :]
    data_hpe_a = data_hpe_v[step:, :] - data_hpe_v[:-step, :]
    data_mc_a = data_mc_v[step:, :] - data_mc_v[:-step, :]
    print(np.argmax(data_hpe_a, axis=0))
    show_error(data_hpe_v, data_mc_v)
    show_error(data_hpe_a, data_mc_a)
    # error_frequence(data_hpe, data_mc)
