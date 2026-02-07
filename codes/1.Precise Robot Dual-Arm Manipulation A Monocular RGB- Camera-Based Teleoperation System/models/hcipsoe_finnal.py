# -----------------------------------------------------
# Copyright (c) Shanghai Jiao Tong University. All rights reserved.
# Written by Jiefeng Li (jeff.lee.sjtu@gmail.com)
# -----------------------------------------------------
import torch
import torch.nn as nn
from easydict import EasyDict

from .layers.Resnet_cbam import ResNet_cbam


def nets():
    return nn.Sequential(
        nn.Linear(2, 64),
        nn.LeakyReLU(),  # 发现这里都喜欢使用LeakyReLU()
        nn.Linear(64, 64),
        nn.LeakyReLU(),
        nn.Linear(64, 2),
        nn.Tanh(),  # 张量的每个元素缩放到[-1,1]
    )
    # return nn.Sequential(nn.Linear(2, 256), nn.LeakyReLU(), nn.Linear(256, 2), nn.Tanh())


def nett():
    return nn.Sequential(
        nn.Linear(2, 64),
        nn.LeakyReLU(),
        nn.Linear(64, 64),
        nn.LeakyReLU(),
        nn.Linear(64, 2),  # 相比而言少了一个Tanh()
    )
    # return nn.Sequential(nn.Linear(2, 256), nn.LeakyReLU(), nn.Linear(256, 2))


# 这是简单的线性输出，没有阶跃函数。对其进行了线性
class Linear(nn.Module):
    def __init__(self, in_channel, out_channel, bias=True, norm=True):
        super(Linear, self).__init__()
        self.bias = bias
        self.norm = norm
        self.linear = nn.Linear(in_channel, out_channel, bias)  # 简单的线性偏置：y=xW+b
        nn.init.xavier_uniform_(
            self.linear.weight, gain=0.01
        )  # 对linear权重矩阵机械能均始化操作

    def forward(self, x):
        y = x.matmul(self.linear.weight.t())  # 这里是对权重矩阵进行转置

        if self.norm:
            x_norm = torch.norm(x, dim=1, keepdim=True)  # 沿第二维度计算张量的L2范数
            y = y / x_norm  # 对输入的张量第二维度进行归一化，使每个张量的长度为一

        if self.bias:
            y = y + self.linear.bias
        return y


class RegressFlow3D_cbam(nn.Module):
    def __init__(self, resnet, joint_num, norm_layer=nn.BatchNorm2d, **cfg):
        super(RegressFlow3D_cbam, self).__init__()
        self.fc_dim = [-1]
        self._norm_layer = norm_layer
        self.num_blones = 17
        self.num_joints = joint_num
        # self.height_dim = self._preset_cfg['IMAGE_SIZE'][0]
        # self.width_dim = self._preset_cfg['IMAGE_SIZE'][1]

        # self.preact = ResNet(f"resnet{cfg['NUM_LAYERS']}")  # 这里直接构建了网络，是自己写的可能有改动，无预训练
        self.preact = ResNet_cbam(
            f"resnet{resnet}"
        )  # 这里直接构建了网络，是自己写的可能有改动，无预训练
        # Imagenet pretrain model
        import torchvision.models as tm  # noqa: F401,F403

        # x = eval("tm.resnet50(weights=tm.ResNet50_Weights.IMAGENET1K_V2)")  # 这里直接载入了现成的网络resnet50，经过预训练

        self.feature_channel = 2048

        self.root_idx = 0  # 这里使设定了根节点参数

        # model_state = self.preact.state_dict()
        # state = {
        #     k: v
        #     for k, v in x.state_dict().items()
        #     if k in self.preact.state_dict() and v.size() == self.preact.state_dict()[k].size()
        # }  # 这里是将现有的resnet网络与自己改的resnetd网络的结构参数进行比较，相同结构的参数，被存入state中

        # model_state.update(state)  # 这是将state中的权值更新给 model_state
        # self.preact.load_state_dict(model_state)  # 将现有的预训练值载入修改后的resnet网络

        self.avg_pool = nn.AdaptiveAvgPool2d(1)

        self.fcs, out_channel = (
            self._make_fc_layer()
        )  # 建立全连接网络，并且给了输出的通道数，

        self.fc_coord = Linear(
            out_channel, self.num_joints * 3
        )  # 这里是使用了线性变换，有归一化和偏置--得到坐标
        self.fc_sigma = nn.Linear(
            out_channel, self.num_joints * 3
        )  # 这里是简单的线性变换，输出的维度就是j*3了

        self.fc_layers = [
            self.fc_coord,
            self.fc_sigma,
        ]  # 这是将有归一化和没归一化线性变换组合，后续好像没有使用，组合起来方便初始化

        self.skeleton = (
            (1, 0),
            (2, 1),
            (3, 2),
            (4, 0),
            (5, 4),
            (6, 5),
            (7, 0),
            (17, 7),
            (8, 17),
            (9, 8),
            (10, 9),
            (11, 17),
            (12, 11),
            (13, 12),
            (14, 17),
            (15, 14),
            (16, 15),
        )  # 17

    def _make_fc_layer(self):
        fc_layers = []
        num_deconv = len(self.fc_dim)
        input_channel = self.feature_channel
        for i in range(num_deconv):
            if self.fc_dim[i] > 0:
                fc = nn.Linear(input_channel, self.fc_dim[i])
                bn = nn.BatchNorm1d(self.fc_dim[i])
                fc_layers.append(fc)
                fc_layers.append(bn)
                fc_layers.append(nn.ReLU(inplace=True))
                input_channel = self.fc_dim[i]
            else:
                fc_layers.append(nn.Identity())

        return nn.Sequential(*fc_layers), input_channel

    def _initialize(self):
        for m in self.fcs:  # 全连接网络的初始化
            if isinstance(m, nn.Linear):
                nn.init.xavier_uniform_(m.weight, gain=0.01)
        for m in self.fc_layers:
            if isinstance(m, nn.Linear):  # 用于分支网络的初始化
                nn.init.xavier_uniform_(m.weight, gain=0.01)

    def forward(self, x, labels=None):
        BATCH_SIZE = x.shape[0]

        feat = self.preact(x)  # 前端网络

        # Positional Pooling
        _, _, f_h, f_w = feat.shape
        feat = self.avg_pool(feat).reshape(
            BATCH_SIZE, -1
        )  # 自适应池化，转化为1维，便于后续使用全连接层

        out_coord = self.fc_coord(feat).reshape(
            BATCH_SIZE, self.num_joints, 3
        )  # 归一化线性层，转化为batch*j*3
        assert out_coord.shape[2] == 3  # 确定是三维的

        out_sigma = self.fc_sigma(feat).reshape(
            BATCH_SIZE, self.num_joints, -1
        )  # 线性层，转化为batch*j*-1 但是fc_sigma定义的输出是j*3，所有就还是b*j*3和out_coord是一样的

        # (B, N, 3)
        pred_jts = out_coord.reshape(
            BATCH_SIZE, self.num_joints, 3
        )  # 得到目标3维坐标---这里的关节点坐标是怎么监督来的

        if not self.training:
            pred_jts[:, :, 2] = (
                pred_jts[:, :, 2] - pred_jts[:, self.root_idx: self.root_idx + 1, 2]
            )
        # 这里是转化为了相对于根关节的深度坐标，pred_jts[:, :, 2]是每个batch，每个关节的 第三坐标：深度值

        sigma = (
            out_sigma.reshape(BATCH_SIZE, self.num_joints, -1).sigmoid() + 1e-9
        )  # 对高斯进行sigmoid处理，转化至0-1之间，得到方差
        # 这里也是得到了 b*j*3

        # 这里为什么会是方差，看看后面的监督

        scores = 1 - sigma  # 这里得到置信度
        # anchor_scores = self.ja(scores)  # 后边的声明需要处理

        scores = torch.mean(
            scores, dim=2, keepdim=True
        )  # 这里是对第三维的num/j的量进行平均，得到batch*j*1，对热图进行平均，得到每个关节的置信度分数

        # scores,_ = torch.min(scores, dim=2)
        # scores = scores.unsqueeze(2)
        # # 得到估计的骨骼长度
        # pred_bl = torch.zeros(BATCH_SIZE, self.num_blones).to(scores.device)
        # for i, (parent, child) in enumerate(self.skeleton):
        #     pred_bl[:, i] = torch.norm(pred_jts[:, parent, :] - pred_jts[:, child, :], dim=1)

        # 输出的部分是：预测的关节点坐标、方差、置信度、似然函数
        output = EasyDict(pred_jts=pred_jts, sigma=sigma, maxvals=scores, nf_loss=None)
        return output
