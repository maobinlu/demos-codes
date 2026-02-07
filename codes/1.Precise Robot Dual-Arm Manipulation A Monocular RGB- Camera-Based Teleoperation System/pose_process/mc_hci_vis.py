'''
Author       : station geyuanliu@gmail.com
Date         : 2025-02-24 11:24:45
LastEditors: station geyuanliu@gmail.com
LastEditTime: 2025-02-24 20:44:00
FilePath: /hcipose_lite/pose_process/mc_hci_vis.py
Description  : 用于可视化处理动捕数据和hci数据
version      : 
notion       : 
'''

import pathadd
import numpy as np
import pdb
from utils.show_traj import show_trajectory3d_web, show_trajectory3d_comp_web
import pandas as pd


if __name__ == "__main__":
    try:
        detected_data = np.load(
            '/home/aloha-pc/interbotix_ws/src/hcipose_lite/save_data/correct_data/hand_root_sc1v2_train.npz',
            allow_pickle=True,
        )
        gt_data = np.load(
            '/home/aloha-pc/interbotix_ws/src/hcipose_lite/save_data/correct_data/hand_root_sc1v2_train_gt.npz',
            allow_pickle=True,
        )
        pred_data = np.load(
            '/home/aloha-pc/interbotix_ws/src/hcipose_lite/save_data/correct_data/hand_root_108_train.npz',
            allow_pickle=True,
        )
    except:
        raise ImportError("Detected data do not exist!")

    ground_truth_data_name = gt_data["name"]
    ground_truth_data_hand_3d = gt_data["hand_3d"]
    detected_data_imgname = detected_data["name"]
    detected_data_hand_3d = detected_data["hand_3d"]
    pred_data_hand_3d = pred_data["hand_3d"]
    sequence_num = len(ground_truth_data_name)

    sampe_id = np.random.randint(0, sequence_num)
    print("sample_id:", sampe_id)
    print("ground_truth_data_name:", ground_truth_data_name[sampe_id][0])
    data_len = len(ground_truth_data_hand_3d[sampe_id])
    data_vis_len = 200
    data_cat = np.random.randint(0, data_len - data_vis_len)
    data_cat = 0
    print(f'data cat:{data_cat}--{data_cat+data_vis_len}')
    show_trajectory3d_comp_web(
        [
            detected_data_hand_3d[sampe_id][data_cat : data_cat + data_vis_len],
            ground_truth_data_hand_3d[sampe_id][data_cat : data_cat + data_vis_len],
            pred_data_hand_3d[sampe_id][data_cat : data_cat + data_vis_len],
        ],
        7025,
    )
