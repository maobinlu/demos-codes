# print(f"Running {__name__}.py from: {__file__}")


import numpy as np
import torch
import torch.nn as nn
from easydict import EasyDict
from .layers.regression_nf_3d_cbam_lite import RegressFlow3D_cbam_lite


class ShiftAttention(nn.Module):
    def __init__(self, in_planes):
        super(ShiftAttention, self).__init__()
        self.fc1 = nn.Linear(in_planes, in_planes // 2, bias=False)
        self.relu1 = nn.ReLU()
        self.fc2 = nn.Linear(in_planes // 2, in_planes, bias=False)
        self.sigmoid = nn.Sigmoid()

    def forward(self, x):
        out = self.fc2(self.relu1(self.fc1(x)))
        return self.sigmoid(out)


class Pro_jts_02_pccp(nn.Module):
    """
    将修正部分单独出来,便于实现级联
    初始化：batch_size, 骨骼长度, 骨骼连接方式
    输入：初始关节点,锚点值,真实骨骼长度
    输出：修正后的关节点
    ----
    - 完全复刻m02
    - 修改为pccp
    """

    def __init__(self, num_blones, num_jts, skeleton):
        super(Pro_jts_02_pccp, self).__init__()
        self.num_blones = num_blones
        self.num_jts = num_jts
        self.skeleton = skeleton
        self.move_bl_a = ShiftAttention(self.num_blones * 2)
        for mm in self.move_bl_a.modules():
            if isinstance(mm, nn.Linear):
                # nn.init.xavier_uniform_(mm.weight, gain=0.01)
                nn.init.kaiming_uniform_(mm.weight, a=0, mode="fan_in", nonlinearity="relu")

    def forward(self, pred_jts, gt_bl, anchor_scores):
        # 得到估计的骨骼长度
        BATCH_SIZE = pred_jts.shape[0]

        pred_bl = torch.zeros(BATCH_SIZE, self.num_blones).to(pred_jts.device)
        for i, (parent, child) in enumerate(self.skeleton):
            pred_bl[:, i] = torch.norm(pred_jts[:, parent, :] - pred_jts[:, child, :], dim=1)
        dis_bl = gt_bl - pred_bl

        move_bl = torch.zeros(BATCH_SIZE, self.num_blones, 2).to(anchor_scores.device)

        for i, (parent, child) in enumerate(self.skeleton):
            anchor_sum = anchor_scores[:, parent] + anchor_scores[:, child] + 1e-9
            move_bl[:, i, 0] = dis_bl[:, i] * anchor_scores[:, child] / anchor_sum
            move_bl[:, i, 1] = dis_bl[:, i] * anchor_scores[:, parent] / anchor_sum

        move_bl_a = self.move_bl_a(move_bl.reshape(BATCH_SIZE, -1)).reshape(
            BATCH_SIZE, self.num_blones, 2
        )
        move_bl = move_bl * move_bl_a
        move_jts = torch.zeros_like(pred_jts)
        for i, (parent, child) in enumerate(self.skeleton):
            bl3d = pred_jts[:, parent, :] - pred_jts[:, child, :]
            force_parent = move_bl[:, i, 0].unsqueeze(1) * (
                bl3d / (torch.norm(bl3d, dim=1).unsqueeze(1) + 1e-9)
            )
            move_jts[:, parent] += force_parent
            force_child = move_bl[:, i, 1].unsqueeze(1) * (
                (-bl3d) / (torch.norm(bl3d, dim=1).unsqueeze(1) + 1e-9)
            )
            move_jts[:, child] += force_child
        pro_jts = pred_jts + move_jts
        return pro_jts


class Pro_jts_endj_pccp(nn.Module):
    """
    将修正部分单独出来,便于实现级联
    初始化：batch_size, 骨骼长度, 骨骼连接方式
    输入：初始关节点,锚点值,真实骨骼长度
    输出：修正后的关节点
    ----
    - 复刻m02
    - 级联处理，首先只对末端关节进行处理
        - 直接只对z轴进行等比例修正
    """

    def __init__(self, num_blones, num_jts, skeleton, skeleton_end):
        super(Pro_jts_endj_pccp, self).__init__()
        self.num_blones = num_blones
        self.num_end_joint = len(skeleton_end)
        self.num_jts = num_jts
        self.skeleton = skeleton
        self.skeleton_end = skeleton_end
        self.move_bl_a = ShiftAttention(self.num_end_joint * 1)
        for mm in self.move_bl_a.modules():
            if isinstance(mm, nn.Linear):
                nn.init.kaiming_uniform_(mm.weight, a=0, mode="fan_in", nonlinearity="relu")

    def forward(self, pred_jts, gt_bl, anchor_scores):
        # 得到估计的骨骼长度
        BATCH_SIZE = pred_jts.shape[0]

        dis_bl = torch.zeros(BATCH_SIZE, self.num_end_joint).to(pred_jts)

        for i, (parent, child) in enumerate(self.skeleton_end):
            pred_bl = torch.norm(pred_jts[:, parent, :] - pred_jts[:, child, :], dim=1)
            dis_bl[:, i] = gt_bl[:, parent - 1] - pred_bl

        move_bl = torch.zeros(BATCH_SIZE, self.num_end_joint).to(anchor_scores.device)

        for i, (parent, child) in enumerate(self.skeleton_end):
            move_bl[:, i] = (
                dis_bl[:, i]
                * anchor_scores[:, child]  # pccp
                # * anchor_scores[:, parent]  # ppcc
                / (anchor_scores[:, parent] + anchor_scores[:, child] + 1e-9)
            )

        move_bl_a = self.move_bl_a(move_bl.reshape(BATCH_SIZE, -1)).reshape(
            BATCH_SIZE, self.num_end_joint
        )
        move_bl = move_bl * move_bl_a
        move_jts = torch.zeros_like(pred_jts)
        for i, (parent, child) in enumerate(self.skeleton_end):
            bl3d = pred_jts[:, parent, :] - pred_jts[:, child, :]
            move_jts[:, parent, 2] = (
                move_bl[:, i] * bl3d[:, 2] / (torch.norm(bl3d) + 1e-9)
            )  # 原本错误

        # 这里有一个错误，但是带来的效果确实极好的
        # bl3d：作为所有b中，这个骨骼三维长度
        # 本来是希望这里可以求得每个batch的这个骨骼的长度 是x*x+y*y+z*z
        # 但是这里却成了 x*x+y*y+z*z + x*x+y*y+z*z +x*x+y*y+z*z ...直接是加了32套
        # 所以应该是使得这里的move_jts变换变得非常小
        # 但是也增添了一些batch_norm的感觉

        pro_jts = pred_jts + move_jts
        return pro_jts


class MLP(nn.Module):
    def __init__(self, in_planes, out_planes, mid_planes=64, act=nn.LeakyReLU()):
        super(MLP, self).__init__()
        self.mlp = nn.Sequential(
            nn.Linear(in_planes, mid_planes),
            nn.LeakyReLU(),
            nn.Linear(mid_planes, mid_planes),
            nn.LeakyReLU(),
            nn.Linear(mid_planes, out_planes),
            act,
        )

    def forward(self, x):
        return self.mlp(x)


class RegressFlow3D_hci_3vm2_best_lite(nn.Module):
    """
    将rle和hci分离，先对rle锁定，只训练后续的hci部分
    - 生成anchor
    - 深层fc+res
    ----
    - 只生成z轴偏移方向，热图监督
    - 链式动态更新

    对于变量的定义：
    _a 是注意力机制默认自己     _**a 指代从哪里计算来
    jts 关节点坐标             bl 骨骼长度
    score 置信度               anchor 变动权重
    pred_ 预测量               gt_ 真实量
    dis_ 偏差量                adj_ 调节量

    """

    def __init__(self, resnet, joint_num, norm_layer=nn.BatchNorm2d):
        super(RegressFlow3D_hci_3vm2_best_lite, self).__init__()

        self.num_joints = joint_num

        self.pre_rle = RegressFlow3D_cbam_lite(resnet, joint_num)  # 这里直接构建之前的rle网络

        self.pred_mlp = MLP(3 * self.num_joints, 3 * self.num_joints, act=nn.Sigmoid())

    def _initialize(self):
        return -1

    def forward(self, x):
        BATCH_SIZE = x.shape[0]
        # self.rle_out = self.pre_rle(x, labels)
        self.rle_out = self.pre_rle(x)
        pred_jts = self.rle_out.pred_jts
        sigma = self.rle_out.sigma
        scores = self.rle_out.maxvals
        """ # 加入对锚点算法的后续处理-------------------------------------------------"""
        pred_jts_o = pred_jts.clone()
        """输出的部分是：预测的关节点坐标、方差、置信度、似然函数"""
        output = EasyDict(
            pred_jts=pred_jts
            + self.pred_mlp(pred_jts.reshape(BATCH_SIZE, -1)).reshape(
                BATCH_SIZE, self.num_joints, 3
            ),
            sigma=sigma,
            maxvals=scores.float(),
            pred_jts_o=pred_jts_o,
            pro_jts=pred_jts,
        )
        return output

    def rle_load(self, load_path):
        torch.save(self.pre_rle.state_dict(), load_path)
        print("save the rle mode" + load_path)

    def pred_mlp_load(self, load_path):
        torch.save(self.pred_mlp.state_dict(), load_path)
        print("save the pred_mlp mode" + load_path)
