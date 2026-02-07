from calendar import c
from math import e
from nis import cat
from turtle import position, width

from matplotlib.widgets import EllipseSelector
import pandas as pd
from scipy.__config__ import show
from sympy import N
import rospy
from geometry_msgs.msg import PoseStamped
from scipy.interpolate import interp1d
import numpy as np
import matplotlib.pyplot as plt
import pdb


class mc_node:
    def __init__(self, topic_name, saved_path=None):
        rospy.init_node("mc_node", anonymous=True)
        self.get_maker = False
        if saved_path is not None:
            self.data = np.load(saved_path, allow_pickle=True)
            # pdb.set_trace()
        else:
            rospy.Subscriber(topic_name, PoseStamped, self.mc_callback, queue_size=1)
            self.data = []

    def mc_callback(self, data):
        pose = data.pose
        position = pose.position
        if position.x < 8000000:
            self.data.append([position.x, position.y, position.z])
            self.get_maker = True
        else:
            self.get_maker = False
            # rospy.loginfo(f"marker loss")
            self.data.append([9000000, 0, 0])
        # rospy.loginfo(f"Received pose: {[position.x, position.y, position.z]}")
        self.current_data = np.array([position.x, position.y, position.z])

    def show_trajectory(self, data_np_list, mask):
        fig = plt.figure()
        num_list = len(data_np_list)
        for i in range(num_list):
            data_np = data_np_list[i]
            # pdb.set_trace()
            data_np[:, 1] = data_np[:, 1] - np.mean(data_np[:, 1])
            data_np[:, 2] = data_np[:, 2] - np.mean(data_np[:, 2])
            data_np[:, 0] = data_np[:, 0] - np.mean(data_np[:, 0])
            ax = fig.add_subplot(1, num_list, i+1, projection="3d")
            for n in range(len(mask)-1):
                if mask[n] and mask[n+1]:
                    ax.plot(data_np[n:n+2, 0], data_np[n:n+2, 1], data_np[n:n+2, 2], color="blue")
                else:
                    if data_np[n, 0] > 2000 or data_np[n+1, 0] > 2000:
                        continue
                    ax.plot(data_np[n:n+2, 0], data_np[n:n+2, 1], data_np[n:n+2, 2], color="red", linewidth=3)
            # ax.plot(data_np[:, 0], data_np[:, 1], data_np[:, 2], color="blue")
            ax.set_xlabel('X 轴')
            ax.set_ylabel('Y 轴')
            ax.set_zlabel('Z 轴')
        plt.show()

    def loss_compensation(self, data_np, windows_size=5, mask=None):
        if mask is not None:
            self.mask = np.array(mask)
        else:
            cat_indx = np.where(data_np[:, 0] != 9000000)
            cat_start = min(cat_indx[0])
            cat_end = max(cat_indx[0])
            data_np = data_np[cat_start:cat_end, :]
            self.data_cat = data_np.copy()
            self.mask = np.where(data_np[:, 0] != 9000000, True, False)
        false_indix = np.where(self.mask == False)[0]
        if false_indix.size == 0:
            return data_np
        start_gap = false_indix[0] - 0
        end_gap = data_np.shape[0] - (false_indix[-1]+1)
        # windows_size = min(start_gap, end_gap, windows_size)
        # print(f"windows_size: {windows_size}")
        # pdb.set_trace()
        segment = self.find_segments(false_indix)
        self.segments = segment
        seg_num = len(segment)
        for i in range(seg_num):
            start, end = segment[i]
        # for (start, end), i in enumerate(segment):
            # pdb.set_trace()
            if start == 0 or end == data_np.shape[0]-1:
                continue
            if i >= 1:
                start_len = min(windows_size, start - segment[i-1][1])
            else:
                start_len = min(windows_size, start_gap)
            if i < seg_num-1:
                end_len = min(windows_size, segment[i+1][0]-end)
            else:
                end_len = min(windows_size, end_gap)
                # pdb.set_trace()
            x_start = np.arange(start - start_len, start, 1, dtype='float64')
            x_end = np.arange(end, end + end_len, 1, dtype='float64')
            x = np.concatenate((x_start, x_end))
            y_start = data_np[start - start_len:start, :]
            y_end = data_np[end:end + end_len, :]
            y = np.concatenate((y_start, y_end), axis=0)
            if y.shape[0] >= 3:
                interp_func = interp1d(x, y, axis=0, kind="quadratic", fill_value="extrapolate")
            elif y.shape[0] == 2:
                interp_func = interp1d(x, y, axis=0, kind="linear", fill_value="extrapolate")
            else:
                continue
            data_np[start:end, :] = interp_func(np.arange(start, end))
        return data_np

    def find_segments(self, false_indix):
        self.segments = []
        start = false_indix[0]
        end = 0
        for i in range(len(false_indix)-1):
            # 后一个比前一个大于1，就是不连续了，
            # 但是如果只有一个点，那么认为还是连续的
            if false_indix[i+1] - false_indix[i] > 2:
                end = false_indix[i]+1
                self.segments.append([start, end])
                start = false_indix[i+1]
        end = false_indix[-1]+1
        self.segments.append([start, end])
        return self.segments


if __name__ == "__main__":
    node = mc_node("/vrpn_client_node/hand4/pose")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        np.save("data3.npy", np.array(node.data), allow_pickle=True)
        node.data = np.array(node.data)
        data_compensation = node.loss_compensation(np.array(node.data))
        data_pre = np.array(node.data_cat)[node.mask]
        # pdb.set_trace()

        node.show_trajectory([np.array(node.data_cat), data_compensation], node.mask)

    # node = mc_node("/vrpn_client_node/hand4/pose", 'data2.npy')
    # node.data = np.array(node.data)
    # data_compensation = node.loss_compensation(np.array(node.data))
    # data_pre = np.array(node.data_cat)[node.mask]
    # node.show_trajectory([np.array(node.data_cat), data_compensation], node.mask)
