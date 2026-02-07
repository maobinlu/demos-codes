import torch
import math
import numpy as np


class Filter:
    def __init__(self, s, n, r=1, d=0):
        """
        Parameters:
            s (int): The size of the window.
            n (int): The number of elements.
            r (float, optional): The decay rate. Defaults to 1.
            d (int, optional): The number of dimensions. Defaults to 0.

        """
        self.weights_sum = 0
        if d == 0:
            self.mask = torch.ones((s, n))
            self.history = torch.ones((s, n))
        else:
            self.mask = torch.ones((s, n, d))
            self.history = torch.ones((s, n, d))
        for i in range(s):
            self.weights_sum += math.exp(-1 * r * i)
            self.mask[i] = self.mask[i] * math.exp(-1 * r * i)

        self.win_size = s

    def init_history(self, item):
        for i in range(self.win_size):
            self.history[i] = item
        self.history = self.history.to(item.device)
        self.mask = self.mask.to(item.device)

    def update(self, item):
        self.history[1:] = self.history[:-1].clone()
        self.history[0] = item
        self.item = self.history * self.mask
        item_out = self.item.sum(dim=0) / self.weights_sum
        return item_out


class AdaptiveFilter:
    def __init__(self, s, n, r=1, d=0, stability_threshold=0.05, max_r=5.0, min_r=0.5,
                 smoothing_factor=0.4, sensitivity=50):
        """
        自适应滤波器，能够根据输入变化的幅度智能调整滤波强度

        Parameters:
            s (int): 窗口大小
            n (int): 元素数量
            r (float, optional): 初始衰减率。默认为1
            d (int, optional): 维度数。默认为0
            stability_threshold (float, optional): 稳定性阈值，用于判断静止状态。默认为0.05
            max_r (float, optional): 最大衰减率。默认为5.0
            min_r (float, optional): 最小衰减率。默认为0.5
            smoothing_factor (float, optional): 衰减率变化的平滑因子。默认为0.2（越小变化越平滑）
            sensitivity (float, optional): 对运动变化的敏感度。默认为1.5

        Example:
            self.adaptive_filter = AdaptiveFilter(s=32, n=4, r=1)
            if i == 0:
                self.adaptive_filter.init_history(bbox)
            bbox = self.adaptive_filter.update(bbox)
        """
        self.win_size = s
        self.n = n
        self.d = d
        self.stability_threshold = stability_threshold
        self.max_r = max_r
        self.min_r = min_r
        self.smoothing_factor = smoothing_factor
        self.sensitivity = sensitivity

        # 为每个元素独立初始化衰减率
        if d == 0:
            self.decay_rates = np.ones(n) * r
            self.target_decay_rates = np.ones(n) * r  # 目标衰减率，用于平滑过渡
            self.mask = np.ones((s, n))
            self.history = np.ones((s, n))
            self.weights_sum = np.ones(n)
        else:
            self.decay_rates = np.ones((n, d)) * r
            self.target_decay_rates = np.ones((n, d)) * r
            self.mask = np.ones((s, n, d))
            self.history = np.ones((s, n, d))
            self.weights_sum = np.ones((n, d))

        # 初始化掩码
        self._update_masks()

        # 用于计算运动幅度的变量
        self.prev_item = None
        if d == 0:
            self.motion_history = np.zeros((5, n))  # 存储更多历史以更好地评估运动趋势
        else:
            self.motion_history = np.zeros((5, n, d))

    def _update_masks(self):
        """根据当前各元素的衰减率更新掩码"""
        if self.d == 0:
            # 重置权重和
            self.weights_sum = np.zeros(self.n)

            # 对每个元素单独计算掩码
            for i in range(self.n):
                for j in range(self.win_size):
                    weight = np.exp(-self.decay_rates[i] * j)
                    self.mask[j, i] = weight
                    self.weights_sum[i] += weight
        else:
            # 重置权重和
            self.weights_sum = np.zeros((self.n, self.d))

            # 对每个元素和维度单独计算掩码
            for i in range(self.n):
                for j in range(self.d):
                    for k in range(self.win_size):
                        weight = np.exp(-self.decay_rates[i, j] * k)
                        self.mask[k, i, j] = weight
                        self.weights_sum[i, j] += weight

    def init_history(self, item):
        """
        初始化历史记录

        Parameters:
            item (numpy array): 用于初始化历史记录的项
        """
        self.history[:] = item  # 将历史记录的所有元素设置为item
        self.prev_item = item.copy()  # 初始化前一项

    def _calculate_motion_magnitude(self, item):
        """
        计算当前输入项与前一项之间的每个元素的运动幅度

        Parameters:
            item (numpy array): 当前输入项

        Returns:
            numpy array: 每个元素的运动幅度
        """
        if self.prev_item is None:
            self.prev_item = item.copy()
            if self.d == 0:
                return np.zeros(self.n)
            else:
                return np.zeros((self.n, self.d))

        # 计算当前项与前一项之间的差异
        diff = np.abs(item - self.prev_item)

        # 更新前一项
        self.prev_item = item.copy()

        return diff

    def _adjust_decay_rates(self, motion_magnitudes):
        """
        根据每个元素的运动幅度调整其目标衰减率，并平滑过渡到新衰减率

        Parameters:
            motion_magnitudes (numpy array): 每个元素的运动幅度
        """
        # 更新运动历史
        self.motion_history[1:] = self.motion_history[:-1]
        self.motion_history[0] = motion_magnitudes

        # 计算加权平均运动幅度，赋予最近的运动更高权重
        weights = np.array([0.4, 0.3, 0.15, 0.1, 0.05])[:, np.newaxis]
        if self.d == 0:
            avg_motion = np.sum(self.motion_history * weights, axis=0)
        else:
            weights = weights[:, np.newaxis, np.newaxis]
            avg_motion = np.sum(self.motion_history * weights, axis=0)

        # 根据运动幅度计算目标衰减率
        if self.d == 0:
            for i in range(self.n):
                # 非线性映射：运动大时衰减率小（响应快），运动小时衰减率大（更稳定）
                motion_factor = np.tanh(avg_motion[i] * self.sensitivity)
                # print(f"Motion factor: {motion_factor}")  # tem 打印运动因子，用于调试
                # 反向映射：将0-1范围映射到min_r-max_r，注意这里是反向的！
                # 高速时需要低衰减率（高响应），低速时需要高衰减率（高稳定性）
                self.target_decay_rates[i] = self.max_r - motion_factor * (self.max_r - self.min_r)

                # 平滑过渡到目标衰减率
                self.decay_rates[i] += self.smoothing_factor * \
                    (self.target_decay_rates[i] - self.decay_rates[i])
        else:
            for i in range(self.n):
                for j in range(self.d):
                    motion_factor = np.tanh(avg_motion[i, j] * self.sensitivity)
                    self.target_decay_rates[i, j] = self.max_r - motion_factor * (self.max_r - self.min_r)
                    self.decay_rates[i, j] += self.smoothing_factor * \
                        (self.target_decay_rates[i, j] - self.decay_rates[i, j])
        # 更新掩码
        self._update_masks()
        # print(f"Decay rates: {self.target_decay_rates}")  # tem 打印衰减率，用于调试

    def update(self, item):
        """
        使用新项更新历史记录并返回加权和

        Parameters:
            item (numpy array): 要添加的新项

        Returns:
            numpy array: 使用掩码的加权和
        """
        # 计算每个元素的运动幅度
        motion_magnitudes = self._calculate_motion_magnitude(item)
        
        # 调整每个元素的衰减率并更新掩码
        self._adjust_decay_rates(motion_magnitudes)

        # 将历史记录向右移动
        self.history[1:] = self.history[:-1]
        self.history[0] = item  # 用新项更新第一个元素

        # 应用掩码
        weighted_items = self.history * self.mask

        # 计算加权和
        if self.d == 0:
            item_out = np.zeros_like(item)
            for i in range(self.n):
                item_out[i] = np.sum(weighted_items[:, i]) / self.weights_sum[i]
        else:
            item_out = np.zeros_like(item)
            for i in range(self.n):
                for j in range(self.d):
                    item_out[i, j] = np.sum(weighted_items[:, i, j]) / self.weights_sum[i, j]

        return item_out


class AdaptiveFilter_ref:
    def __init__(self, s, n, r=1, d=0, n_ref=3, d_ref=2, max_r=5.0, min_r=0.5,
                 smoothing_factor=0.6, sensitivity=1/5):
        """
        自适应滤波器，能够根据输入的相关变量的变化的幅度智能调整滤波强度

        Parameters:
            s (int): 窗口大小
            n (int): 元素数量
            r (float, optional): 初始衰减率。默认为1
            d (int, optional): 维度数。默认为0
            n_ref (int, optional): 相关变量的元素数量
            d_ref (int, optional): 相关变量的维度数
            max_r (float, optional): 最大衰减率。默认为5.0
            min_r (float, optional): 最小衰减率。默认为0.5
            smoothing_factor (float, optional): 衰减率变化的平滑因子。默认为0.2（越小变化越平滑）
            sensitivity (float, optional): 对运动变化的敏感度。默认为1.5

        Example:
            self.adaptive_filter = AdaptiveFilter(s=32, n=4, r=1)
            if i == 0:
                self.adaptive_filter.init_history(bbox)
            bbox = self.adaptive_filter.update(bbox)
        """
        self.win_size = s
        self.n = n
        self.d = d
        self.n_ref = n_ref
        self.d_ref = d_ref
        self.max_r = max_r
        self.min_r = min_r
        self.smoothing_factor = smoothing_factor
        self.sensitivity = sensitivity

        # 为每个元素独立初始化衰减率
        # 相关变量与目标变量，维度不同，只设置n个衰变率，大概率也是一样的
        self.decay_rates = np.ones(n) * r
        self.target_decay_rates = np.ones(n) * r  # 目标衰减率，用于平滑过渡
        self.mask = np.ones((s, n))
        self.history = np.ones((s, n))
        self.weights_sum = np.ones(n)

        # 初始化掩码
        self._update_masks()

        # 用于计算运动幅度的变量
        self.prev_item = None
        if self.d_ref == 0:
            self.motion_history = np.zeros((5, n_ref))  # 存储更多历史以更好地评估运动趋势
        else:
            self.motion_history = np.zeros((5, n_ref, d_ref))

    def _update_masks(self):
        """根据当前各元素的衰减率更新掩码"""
        if self.d == 0:
            # 重置权重和
            self.weights_sum = np.zeros(self.n)

            # 对每个元素单独计算掩码
            for i in range(self.n):
                for j in range(self.win_size):
                    weight = np.exp(-self.decay_rates[i] * j)
                    self.mask[j, i] = weight
                    self.weights_sum[i] += weight
        else:
            # 重置权重和
            self.weights_sum = np.zeros((self.n, self.d))

            # 对每个元素和维度单独计算掩码
            for i in range(self.n):
                for j in range(self.d):
                    for k in range(self.win_size):
                        weight = np.exp(-self.decay_rates[i, j] * k)
                        self.mask[k, i, j] = weight
                        self.weights_sum[i, j] += weight

    def init_history(self, item):
        """
        初始化历史记录

        Parameters:
            item (numpy array): 用于初始化历史记录的项
        """
        self.history[:] = item  # 将历史记录的所有元素设置为item
        # self.prev_item = item.copy()  # 初始化前一项

    def _calculate_motion_magnitude(self, item):
        """
        计算当前输入项与前一项之间的每个元素的运动幅度

        Parameters:
            item (numpy array): 当前输入项

        Returns:
            numpy array: 每个元素的运动幅度
        """
        if self.prev_item is None:
            self.prev_item = item.copy()
            if self.d_ref == 0:
                return np.zeros(self.n_ref)
            else:
                return np.zeros((self.n_ref, self.d_ref))

        # 计算当前项与前一项之间的差异
        diff = np.abs(item - self.prev_item)

        # 更新前一项
        self.prev_item = item.copy()

        return diff

    def _adjust_decay_rates(self, motion_magnitudes):
        """
        根据每个元素的运动幅度调整其目标衰减率，并平滑过渡到新衰减率

        Parameters:
            motion_magnitudes (numpy array): 每个元素的运动幅度
        """
        # 更新运动历史
        self.motion_history[1:] = self.motion_history[:-1]
        self.motion_history[0] = motion_magnitudes

        # 计算加权平均运动幅度，赋予最近的运动更高权重
        weights = np.array([0.4, 0.3, 0.15, 0.1, 0.05])
        if self.d_ref == 0:
            weights = weights[:, np.newaxis]
            avg_motion = np.linalg.norm(self.motion_history * weights, axis=(-1)).mean()
        else:
            weights = weights[:, np.newaxis, np.newaxis]
            avg_motion = np.linalg.norm(self.motion_history * weights, axis=(0,-1)).mean()
        # print(f"Avg motion: {avg_motion}")  # tem 打印平均运动幅度，用于调试
        # 根据运动幅度计算目标衰减率
        if self.d == 0:
            for i in range(self.n):
                # 非线性映射：运动大 衰减率大 factor大（响应快），运动小 衰减率小 factor小（更稳定）
                motion_factor = np.tanh(avg_motion * self.sensitivity)
                if i==0:
                    motion_factor = np.tanh(avg_motion * self.sensitivity*1.1) #tag 对z轴特殊处理，滤波更强
                self.target_decay_rates[i] = self.min_r + motion_factor * (self.max_r- self.min_r )

                # 平滑过渡到目标衰减率
                self.decay_rates[i] += self.smoothing_factor * \
                    (self.target_decay_rates[i] - self.decay_rates[i])
        else:
            for i in range(self.n):
                for j in range(self.d):
                    motion_factor = np.tanh(avg_motion[i, j] * self.sensitivity)
                    self.target_decay_rates[i, j] = self.max_r - motion_factor * (self.max_r - self.min_r)
                    self.decay_rates[i, j] += self.smoothing_factor * \
                        (self.target_decay_rates[i, j] - self.decay_rates[i, j])
        # 更新掩码
        self._update_masks()
        return motion_factor
        # print(f"Decay rates: {self.target_decay_rates}")  # tem 打印衰减率，用于调试

    def update(self, item, item_ref):
        """
        使用新项更新历史记录并返回加权和

        Parameters:
            item (numpy array): 要添加的新项

        Returns:
            numpy array: 使用掩码的加权和
        """
        # 计算每个元素的运动幅度
        motion_magnitudes = self._calculate_motion_magnitude(item_ref)

        # 调整每个元素的衰减率并更新掩码
        factor = self._adjust_decay_rates(motion_magnitudes)

        # 将历史记录向右移动
        self.history[1:] = self.history[:-1]
        self.history[0] = item  # 用新项更新第一个元素

        # 应用掩码
        weighted_items = self.history * self.mask

        # 计算加权和
        if self.d == 0:
            item_out = np.zeros_like(item)
            for i in range(self.n):
                item_out[i] = np.sum(weighted_items[:, i]) / self.weights_sum[i]
        else:
            item_out = np.zeros_like(item)
            for i in range(self.n):
                for j in range(self.d):
                    item_out[i, j] = np.sum(weighted_items[:, i, j]) / self.weights_sum[i, j]

        return item_out, factor #tem factor
        # return item_out, motion_magnitudes
