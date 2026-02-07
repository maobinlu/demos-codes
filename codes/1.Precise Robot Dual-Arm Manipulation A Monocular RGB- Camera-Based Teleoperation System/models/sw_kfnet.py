import numpy as np
import torch
import torch.nn as nn
import pdb
import torch.nn.functional as func


class KalmanFilter(nn.Module):
    """处理一维坐标，建模认为加速度导数固定"""

    def __init__(self, window_size, dim_x=3, dim_z=3):
        super(KalmanFilter, self).__init__()
        self.dim_x = dim_x  # 状态向量维度
        self.dim_z = dim_z  # 观测向量维度
        self.window_size = window_size

        self.A_ele_num = dim_x * (dim_x + 1) // 2
        self.R_ele_num = dim_z * (dim_z + 1) // 2

        self.H = torch.zeros(dim_z, dim_x)  # 观测矩阵 3x1
        self.H[0, 0] = 1
        self.H[1, 1] = 1
        self.H[2, 2] = 1

        self.GRU_xt = nn.GRU((window_size-1) * dim_z, dim_z * 5, batch_first=True)
        self.fc_xt = nn.Sequential(
            nn.Linear(dim_z * 5, dim_z),
            nn.Tanh(),
        )

        self.GRU_P = nn.GRU((window_size-1) * dim_z, dim_z * 5, batch_first=True)
        self.fc_P = nn.Sequential(
            nn.Linear(dim_z * 5, dim_z),
            nn.Sigmoid(),
        )

        self.GRU_R = nn.GRU((window_size) * dim_z, dim_z * 5, batch_first=True)
        self.fc_R = nn.Sequential(
            nn.Linear(dim_z * 5, dim_z),
            nn.Sigmoid(),
        )

    def forward(
        self,
        pred_data,
        window_data,
        state=None,
        Probability=None,
    ):
        batch_size = pred_data.shape[0]

        if state is None:
            state = torch.zeros(batch_size, self.dim_x, 1, device=pred_data.device)
            self.h_xt = torch.zeros(1, batch_size, self.dim_z * 5).to(pred_data.device)
            self.h_P = torch.zeros(1, batch_size, self.dim_z * 5).to(pred_data.device)
            self.h_R = torch.zeros(1, batch_size, self.dim_z * 5).to(pred_data.device)
        
        xt, self.h_xt = self.GRU_xt(window_data.view(batch_size, -1).unsqueeze(1), self.h_xt)
        xt = self.fc_xt(xt.squeeze(1)).unsqueeze(-1)
        P_diag, self.h_P = self.GRU_P(window_data.view(batch_size, -1).unsqueeze(1), self.h_P)
        P_diag = self.fc_P(P_diag).squeeze(1)
        R_diag, self.h_R = self.GRU_R(pred_data.view(batch_size, -1).unsqueeze(1), self.h_R)
        R_diag = self.fc_R(R_diag).squeeze(1)

        R = torch.diag_embed(R_diag, dim1=-2, dim2=-1).to(pred_data.device)  # 观测噪声协方差矩阵
        P = torch.diag_embed(P_diag, dim1=-2, dim2=-1).to(pred_data.device)  # 过程噪声协方差矩阵
        # print('xt.shape', xt.shape)
        # print('P.shape', P.shape)
        # print('R.shape', R.shape)
        H_batch = self.H.unsqueeze(0).repeat(batch_size, 1, 1).to(pred_data.device)

        y = pred_data[:,-1,:].unsqueeze(-1) - torch.bmm(H_batch, xt)
        S = torch.bmm(H_batch, torch.bmm(P, H_batch.transpose(1, 2))) + R
        
        # print('r.shape', r.shape)


        K = torch.bmm(
            torch.bmm(P, H_batch.transpose(1, 2)), torch.inverse(S)
        )  # Kg(k)=P(k|k-1)H'/[HP(k|k-1)H' + R],H=1
        x = xt + torch.bmm(K, y)  # X(k|k) = X(k|k-1) + Kg(k)[Z(k) - HX(k|k-1)]
        return x, xt, P_diag, R_diag


class sc5v3(nn.Module):
    """
    sc1v1:直接照搬as1v1
    sc1v2:对速度和加速度降频处理，提升稳定性
    sc3v0:在网络中加入静态kalman滤波
    sc3v1:对kalman中的r和q灵活赋值
    sc3v2:神经网络输出r和q
    sc3v4:神经网络输出r和q改为基准+比例
    sc3v5:神经网络直接运算qrp和k
    sc5v1:GRU实现sc3v5
    sc5v2:GRU实现sc3v4
    sc5v3:GRU拟合正态分布的均值和方差
    """

    def __init__(self, window_size):
        super(sc5v3, self).__init__()
        self.window_size = window_size  # 小window
        self.kalman = KalmanFilter(self.window_size, 3, 3)

    def _initialize(self):
        return -1

    def forward(self, data_pred, current_window=None, gt=None, state=None, Probability=None):
        data_pred = data_pred.float()
        current_window = current_window.float()
        state, xt, P_diag, R_diag = self.kalman(data_pred, current_window, state, Probability)
        # output = {"pro_data": state[:,:3,0], "state": state,"Probability": Probability}
        return state[:,:3,0],state,Probability