
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.__config__ import show
from sympy import N
import torch
from kalman_filter import KalmanFilter
from filter import Filter
import pdb
import torch
import math
from dataset_process import show_trajectory, show_error


def procrustes_analysis(X, Y):
    # 去中心化
    mu_X = np.mean(X, axis=0)
    mu_Y = np.mean(Y, axis=0)
    X_centered = X - mu_X
    Y_centered = Y - mu_Y

    # SVD求解旋转矩阵
    H = Y_centered.T @ X_centered
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # 确保旋转矩阵没有反射
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    # 平移向量
    t = mu_X - R @ mu_Y

    # 缩放因子
    scale = np.trace(H) / np.sum(np.square(Y_centered))

    return R, t, scale


def compute_mean_transform(R_list, T_list, scale_list):
    # 计算缩放因子的均值
    scale_mean = np.mean(scale_list)

    # 计算平移向量的均值
    T_mean = np.mean(T_list, axis=0)

    # 计算旋转矩阵的均值
    R_avg = np.mean(R_list, axis=0)  # 逐元素均值
    U, _, Vt = np.linalg.svd(R_avg)
    R_mean = U @ Vt  # 强制正交化

    # 确保旋转矩阵的行列式为1（防止反射矩阵出现）
    if np.linalg.det(R_mean) < 0:
        U[:, -1] *= -1
        R_mean = U @ Vt

    return R_mean, T_mean, scale_mean


if __name__ == "__main__":
    act_dict = {0: 'x0', 1: 'x1', 2: 'x2', 3: 'x3', 4: 'x4', 5: 'lg0', 6: 'lg1', 7: 'lg2', 8: 'lg3', 9: 'lg4'}
    act_test_dict = {0: 'x5', 1: 'lg5', 2: 'l0', 3: 'l1', 4: 'l2'}
    R_list = []
    T_list = []
    scale_list = []
    for i in range(10):
        act = act_dict[i]
        kf_x = KalmanFilter(dim_x=3, dim_z=1, var=20)
        kf_y = KalmanFilter(dim_x=3, dim_z=1, var=0.02)
        kf_z = KalmanFilter(dim_x=3, dim_z=1, var=0.02)
        pose_filter = Filter(s=64, n=1, d=3, r=1)
        data_hpe = np.load(f'dataset/{act}/save_hpe.npy')
        data_mc = np.load(f'dataset/{act}/save_mc.npy')
        data_mc = data_mc[:, [1, 0, 2]]
        data_mc[:, 0] = data_mc[:, 0]*(-1)
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
        R, t, scale = procrustes_analysis(data_mc, data_hpe_k)  # zheng
        data_hpe_t = scale * (data_hpe_k @ R.T) + t
        print(R, t, scale)
        error = np.sqrt(np.sum((data_hpe_t-data_mc)**2, axis=1))
        print(f'error_norm-{act}:{np.mean(error)}')
        R_list.append(R)
        T_list.append(t)
        scale_list.append(scale)
    R_mean, T_mean, scale_mean = compute_mean_transform(R_list, T_list, scale_list)
    print('calibration result:')
    print(R_mean, T_mean, scale_mean)
    for i in range(5):
        act = act_test_dict[i]
        data_hpe = np.load(f'dataset/{act}/save_hpe.npy')
        data_mc = np.load(f'dataset/{act}/save_mc.npy')
        data_mc = data_mc[:, [1, 0, 2]]
        data_mc[:, 0] = data_mc[:, 0]*(-1)
        data_hpe_k = data_hpe.copy()
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
        data_hpe_t = scale_mean * (data_hpe_k @ R_mean.T) + T_mean
        error = np.sqrt(np.sum((data_hpe_t-data_mc)**2, axis=1))
        print(f'error_norm-{act}:{np.mean(error)}')

    # show_trajectory([data_hpe, data_hpe_k, data_hpe_t, data_mc], [2, 3], -1)
    # show_error(data_hpe_t, data_mc)
    # show_error(data_hpe_k, data_mc)

# result:
# R = np.array([[9.87045566e-01, -1.35264977e-01, -8.62811506e-02],
#               [1.34829228e-01,  9.90809061e-01, -1.08850342e-02],
#               [8.69605097e-02, -8.89196189e-04,  9.96211363e-01]])
# T = np.array([-95.3572745,  -45.94321624,  27.25388518])
# scale = 0.9648176310192673
