import re
import numpy as np


class KalmanFilter:
    '''处理一维坐标，建模认为加速度导数固定'''

    def __init__(self, dim_x=3, dim_z=1, var=0.1):
        self.dim_x = dim_x  # 状态向量维度
        self.dim_z = dim_z  # 观测向量维度

        self.x = np.zeros((dim_x, 1))  # 状态向量
        self.P = np.eye(dim_x)  # 状态协方差矩阵
        self.A = np.array([[1, 0.03, 0], [0, 1, 0.03], [0, 0, 1]])  # 状态转移矩阵
        self.E = np.array([[0], [0], [1]])  # 状态矩阵常数项，加速度导数为1
        self.H = np.array([[1, 0, 0]])  # 观测矩阵 3x1
        self.R = np.eye(dim_z)*var  # 观测噪声协方差矩阵
        self.Q = np.eye(dim_x)  # 过程噪声协方差矩阵

    def predict(self):
        self.x = np.dot(self.A, self.x)+self.E  # X(k|k-1) = AX(k-1|k-1) + BU(k) + W(k),BU(k) = 0
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q  # P(k|k-1) = AP(k-1|k-1)A' + Q(k) ,A=[[1,1],[0,1]]

    def update(self, z):
        y = z - np.dot(self.H, self.x)
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))  # Kg(k)=P(k|k-1)H'/[HP(k|k-1)H' + R],H=1
        self.x = self.x + np.dot(K, y)  # X(k|k) = X(k|k-1) + Kg(k)[Z(k) - HX(k|k-1)]
        self.P = self.P - np.dot(np.dot(K, self.H), self.P)  # P(k|k) = (1 - Kg(k)H)P(k|k-1)
        return self.x[0, 0]
