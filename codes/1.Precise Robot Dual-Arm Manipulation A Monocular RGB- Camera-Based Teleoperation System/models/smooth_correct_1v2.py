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
    def __init__(self, frame_nums, data_dim=51, wight_type="arange", ratio=0.5):
        super(attention_module, self).__init__()
        self.frame_nums = frame_nums
        self.data_dim = data_dim
        self.W_V = nn.Linear(data_dim, data_dim, bias=False)
        self.layernorm1 = nn.LayerNorm(data_dim, elementwise_affine=False)
        self.layernorm2 = nn.LayerNorm(data_dim, elementwise_affine=False)
        self.mlp = MLP_dropout(
            frame_nums * data_dim, frame_nums * data_dim, mid_planes=256, act=nn.ReLU()
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
        v = self.W_V(x.reshape(-1, self.data_dim)).reshape(BATCHSIZE, -1, self.data_dim)  # B,N,J
        self.wight = self.wight.to(x.device)  # 1,N
        wight = self.wight.expand(BATCHSIZE, -1)  # 形状变为 (BATCHSIZE, N)
        output = v * wight.unsqueeze(-1) + x  # B,N,J
        output = self.layernorm1(output)
        output = (
            self.mlp(output.reshape(BATCHSIZE, -1)).reshape(BATCHSIZE, -1, self.data_dim) + output
        )
        return self.layernorm2(output)


class attention_module_cascade(nn.Module):
    def __init__(self, cascade, frame_nums, data_dim=51, wight_type="arange", ratio=1.5):
        super(attention_module_cascade, self).__init__()
        for i in range(cascade):
            self.add_module(
                "attention_module" + str(i),
                attention_module(frame_nums, data_dim, wight_type, ratio),
            )

    def forward(self, x):
        for module in self._modules.values():
            x = module(x)
        return x


class as_net_v2(nn.Module):
    def __init__(self, window_size,vel_step=1,acc_step=1):
        super(as_net_v2, self).__init__()
        self.window_size = window_size
        self.data_dim = 3
        self.vel_step = vel_step
        self.acc_step = acc_step
        self.pred_feat = attention_module_cascade(
            3, self.window_size, data_dim=self.data_dim, wight_type="ratio"
        )
        self.adjust_feat = attention_module_cascade(
            3, self.window_size - 1, data_dim=self.data_dim, wight_type="ratio"
        )
        self.pro_vel = attention_module_cascade(
            3, self.window_size - self.vel_step - 1, data_dim=self.data_dim, wight_type="ratio"
        )
        self.pro_acc = attention_module_cascade(
            3,
            self.window_size - self.vel_step - self.acc_step - 1,
            data_dim=self.data_dim,
            wight_type="ratio",
        )
        # self.mlp_wight = MLP((2 * self.window_size - 2) * 51, 4, mid_planes=256, act=nn.Sigmoid())
        self.out_liner = nn.Sequential(
            nn.Linear(self.data_dim * 5, self.data_dim * 2),
            nn.Linear(self.data_dim * 2, self.data_dim),
        )

    def _initialize(self):
        return -1

    def forward(self, pred, window):
        # 这里的pred是原始得到的数据，window是之前的处理过的数据
        # 这里的mean是仿照的avg_pool
        BATCHSIZE = pred.shape[0]
        pred_feat_out = self.pred_feat(pred).mean(dim=1)  # B,N,J -- B,1,J
        adjust_feat_out = self.adjust_feat(window - pred[:, :-1, :]).mean(dim=1)  # B,1,J
        window_avg = window
        # window_val = window_avg[:, 1:, :] - window_avg[:, :-1, :]
        window_val_step = (
            window_avg[:, self.vel_step :, :] - window_avg[:, : -self.vel_step, :]
        )  # B,N-vel_step,J
        pro_vel_out = self.pro_vel(window_val_step).mean(dim=1)  # B,N-2,J -- B,1,J
        pro_acc_out = self.pro_acc(
            window_val_step[:, self.acc_step :, :] - window_val_step[:, : -self.acc_step, :]
        ).mean(
            dim=1
        )  # B,N-3,J -- B,1,J
        adjust_feat_out = adjust_feat_out + pred[:, -1, :]
        pro_vel_out = window_avg[:, -1, :] + pro_vel_out / self.vel_step
        pro_acc_out = (
            window_avg[:, -1, :]
            + (window_val_step[:, -1, :] + pro_acc_out / self.acc_step) / self.vel_step
        )
        output_all = torch.cat(
            [pred_feat_out, adjust_feat_out, pro_vel_out, pro_acc_out, pred[:, -1, :]], dim=1
        )
        output = self.out_liner(output_all)

        return output


class sc1v2(nn.Module):
    """
    sc1v1:直接照搬as1v1
    sc1v2:对速度和加速度降频处理，提升稳定性
    """

    def __init__(self, window_size):
        super(sc1v2, self).__init__()
        self.infer_net = as_net_v2(window_size)
        self.window_size = window_size  # 小window

    def _initialize(self):
        return -1

    def forward(self, data_pred, current_window=None, gt=None):
        data_pred = data_pred.float()
        current_window = current_window.float()
        prod = self.infer_net(data_pred, current_window)
        return prod
