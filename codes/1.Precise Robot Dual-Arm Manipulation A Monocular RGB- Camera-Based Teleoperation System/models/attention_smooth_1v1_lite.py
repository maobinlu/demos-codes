import numpy as np
import torch
import torch.nn as nn
import pdb


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


class MLP_dropout(nn.Module):
    def __init__(self, in_planes, out_planes, mid_planes=64, act=nn.LeakyReLU()):
        super(MLP_dropout, self).__init__()
        self.mlp = nn.Sequential(
            nn.Linear(in_planes, mid_planes),
            nn.LeakyReLU(),
            nn.Dropout(0.5),
            nn.Linear(mid_planes, mid_planes),
            nn.LeakyReLU(),
            nn.Dropout(0.5),
            nn.Linear(mid_planes, out_planes),
            act,
        )

    def forward(self, x):
        return self.mlp(x)


class attention_module(nn.Module):
    def __init__(self, frame_nums, joint_num=51, wight_type="arange", ratio=0.5):
        super(attention_module, self).__init__()
        self.frame_nums = frame_nums
        self.joint_num = joint_num
        self.W_V = nn.Linear(joint_num, joint_num, bias=False)
        self.layernorm1 = nn.LayerNorm(joint_num, elementwise_affine=False)
        self.layernorm2 = nn.LayerNorm(joint_num, elementwise_affine=False)
        self.mlp = MLP_dropout(
            frame_nums * joint_num, frame_nums * joint_num, mid_planes=256, act=nn.ReLU()
        )
        if wight_type == "arange":
            self.wight = torch.arange(0, frame_nums - 1, 1)
        elif wight_type == "ratio":
            self.wight = torch.tensor([0 * (ratio**i) for i in range(frame_nums)])
        else:
            print("wight_type error")
            raise ValueError
        self.wight = nn.functional.softmax(self.wight, dim=0).unsqueeze(0)

    def forward(self, x):
        BATCHSIZE = x.shape[0]
        v = self.W_V(x.reshape(-1, self.joint_num)).reshape(BATCHSIZE, -1, self.joint_num)  # B,N,J
        self.wight = self.wight.to(x.device)  # 1,N
        wight = self.wight.expand(BATCHSIZE, -1)  # 形状变为 (BATCHSIZE, N)
        output = v * wight.unsqueeze(-1) + x  # B,N,J
        output = self.layernorm1(output)
        output = (
            self.mlp(output.reshape(BATCHSIZE, -1)).reshape(BATCHSIZE, -1, self.joint_num) + output
        )
        return self.layernorm2(output)


class attention_module_cascade(nn.Module):
    def __init__(self, cascade, frame_nums, joint_num=51, wight_type="arange", ratio=1.5):
        super(attention_module_cascade, self).__init__()
        for i in range(cascade):
            self.add_module(
                "attention_module" + str(i),
                attention_module(frame_nums, joint_num, wight_type, ratio),
            )

    def forward(self, x):
        for module in self._modules.values():
            x = module(x)
        return x


class as_net(nn.Module):
    def __init__(self, window_size,joint_num=51):
        super(as_net, self).__init__()
        self.window_size = window_size
        self.pred_feat = attention_module_cascade(
            3, self.window_size, joint_num, wight_type="ratio"
        )
        self.adjust_feat = attention_module_cascade(
            3, self.window_size - 1, joint_num, wight_type="ratio"
        )
        self.pro_vel = attention_module_cascade(
            3, self.window_size - 2, joint_num, wight_type="ratio"
        )
        self.pro_acc = attention_module_cascade(
            3, self.window_size - 3, joint_num, wight_type="ratio"
        )
        # self.mlp_wight = MLP((2 * self.window_size - 2) * 51, 4, mid_planes=256, act=nn.Sigmoid())
        self.out_liner = nn.Sequential(nn.Linear(joint_num * 5, joint_num * 2), nn.Linear(joint_num * 2, joint_num))

    def _initialize(self):
        return -1

    def forward(self, pred, window):
        BATCHSIZE = pred.shape[0]
        pred_feat_out = self.pred_feat(pred).mean(dim=1)  # B,N,J
        adjust_feat_out = self.adjust_feat(window - pred[:, :-1, :]).mean(dim=1)  # B,1,J
        window_avg = window
        window_val = window_avg[:, 1:, :] - window_avg[:, :-1, :]
        pro_vel_out = self.pro_vel(window_val).mean(dim=1)  # B,N-2,J
        pro_acc_out = self.pro_acc(window_val[:, 1:, :] - window_val[:, :-1, :]).mean(
            dim=1
        )  # B,N-3,J
        adjust_feat_out = adjust_feat_out + pred[:, -1, :]
        pro_vel_out = window_avg[:, -1, :] + pro_vel_out
        pro_acc_out = window_avg[:, -1, :] + window_val[:, -1, :] + pro_acc_out
        output_all = torch.cat(
            [pred_feat_out, adjust_feat_out, pro_vel_out, pro_acc_out, pred[:, -1, :]], dim=1
        )
        output = self.out_liner(output_all)

        return output


class as1v1(nn.Module):
    """
    as0v1:对最后的输出不使用/4，而是用加权或者fc整合
    as0v2:不计算
    as1v0:更新训练方式，使用大window,小段逐步训练
    as1v1:类似as0v4，使用vel和acc，并且加入了pred做fc
    """

    def __init__(self, window_size):
        super(as1v1, self).__init__()
        self.infer_net = as_net(window_size)
        self.window_size = window_size  # 小window

    def _initialize(self):
        return -1

    def forward(self, data_pred, current_window=None, gt=None):
        data_pred = data_pred.float()
        if data_pred.shape[0] == 1 and current_window.shape[1] == self.window_size - 1:
            current_window = current_window.float()
            prod = self.infer_net(data_pred, current_window)
            # output = {"pro_jts": prod}
            return prod
        else:
            print("data_pred shape error")
