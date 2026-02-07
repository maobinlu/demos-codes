import os
from eval_metrics import calculate_mpjpe, calculate_pampjpe, calculate_accel_error
import cv2
import numpy as np
import torch
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from tqdm import trange
from matplotlib.backends.backend_agg import FigureCanvasAgg
import pdb

SUB_FIG_SIZE = 4
SUB_FIG_UNIT = 100
VIEW_NUM = 1
VIEW_CAMERA = [[10, 10]]


SKELETON_3D_RADIUS = 1.7  # 骨架半径
SKELETON_AX_DIST = 6.5
FRAME_RATE = 30

SKELRTON_COLOR = ["red", "black"]


# 绘制关节点三维运动轨迹
def plot_joint_trajectory(ax_3d, skeleton, gt):
    # pdb.set_trace()

    skeleton = -skeleton[
        :, :, [2, 0, 1]
    ]  # 将骨架的z坐标放在第一列，x坐标放在第二列，y坐标放在第三列
    gt = -gt[:, :, [2, 0, 1]]
    center = np.mean(skeleton[:, 0, :], axis=0)  # 计算骨架的中心点
    # pdb.set_trace()
    ax_3d.set_xlim3d(
        [center[0] - SKELETON_3D_RADIUS / 2, center[0] + SKELETON_3D_RADIUS / 2]
    )  # 设置x轴的范围为中心点的x坐标减去骨架半径的一半到中心点的x坐标加上骨架半径的一半
    ax_3d.set_ylim3d([center[1] - SKELETON_3D_RADIUS / 2, center[1] + SKELETON_3D_RADIUS / 2])
    ax_3d.set_zlim3d([center[2], center[2] + SKELETON_3D_RADIUS / 2])
    ax_3d.plot3D(
        skeleton[:, 16, 0],
        skeleton[:, 16, 1],
        skeleton[:, 16, 2],
        color=SKELRTON_COLOR[0],
        alpha=0.9,
    )
    ax_3d.plot3D(
        gt[:, 16, 0],
        gt[:, 16, 1],
        gt[:, 16, 2],
        color=SKELRTON_COLOR[1],
        alpha=0.9,
    )
    return


def plot_skeleton(ax_3d, skeleton, color, edges=None, joints=None):
    skeleton = -skeleton[:, [2, 0, 1]]
    center = np.mean(skeleton, axis=0)  # 计算骨架的中心点

    ax_3d.set_xlim3d(
        [center[0] - SKELETON_3D_RADIUS / 2, center[0] + SKELETON_3D_RADIUS / 2]
    )  # 设置x轴的范围为中心点的x坐标减去骨架半径的一半到中心点的x坐标加上骨架半径的一半
    ax_3d.set_ylim3d([center[1] - SKELETON_3D_RADIUS / 2, center[1] + SKELETON_3D_RADIUS / 2])
    ax_3d.set_zlim3d([center[2] - SKELETON_3D_RADIUS / 2, center[2] + SKELETON_3D_RADIUS / 2])

    kps_lines_3d = (
        (1, 0),
        (2, 1),
        (3, 2),  # 2
        (4, 0),
        (5, 4),
        (6, 5),  # 5
        (7, 8),  # 7
        (7, 0),
        (9, 8),
        (10, 9),  # 9
        (11, 7),
        (12, 11),
        (13, 12),  # 12
        (14, 7),
        (15, 14),
        (16, 15),  # 15
    )  # 16

    rad_line = [1, 2, 3, 14, 15, 16]

    for link in range(len(kps_lines_3d)):
        i1 = kps_lines_3d[link][0]
        i2 = kps_lines_3d[link][1]
        p1 = skeleton[i1]
        p2 = skeleton[i2]
        k = 0 if i1 in rad_line or i2 in rad_line else 1
        ax_3d.plot3D(
            [p1[0], p2[0]],
            [p1[1], p2[1]],
            [p1[2], p2[2]],
            color=SKELRTON_COLOR[k],
            marker="o",
            markersize=2,
            alpha=0.9,
        )

    return


def visualize_3d(
    vis_output_video_path,
    vis_output_video_name,
    data_pred,
    data_gt,
    predicted_pos,
    start_frame,
    end_frame,
):
    print("Visualizing the result ...")

    # dataset_edges = eval(dataset_name.upper() + "_" + estimator_name.upper() + "_3D_EDGES")
    # dataset_joints = eval(dataset_name.upper() + "_" + estimator_name.upper() + "_3D_JOINTS")

    if not os.path.exists(vis_output_video_path):
        os.makedirs(vis_output_video_path)

    len_seq = data_gt.shape[0]

    m2mm = 1000

    # calculate errors
    mpjpe_in = np.array(calculate_mpjpe(data_pred, data_gt).cpu()) * m2mm
    mpjpe_out = np.array(calculate_mpjpe(predicted_pos, data_gt).cpu()) * m2mm

    acc_in = (
        np.concatenate(
            (
                np.array([0]),
                np.array(calculate_accel_error(data_pred, data_gt).cpu()),
                np.array([0]),
            )
        )
        * m2mm
    )
    acc_out = (
        np.concatenate(
            (
                np.array([0]),
                np.array(calculate_accel_error(predicted_pos, data_gt).cpu()),
                np.array([0]),
            )
        )
        * m2mm
    )

    data_pred = np.array(data_pred.cpu())
    data_gt = np.array(data_gt.cpu())
    predicted_pos = np.array(predicted_pos.cpu())

    anim_output = {
        "Estimator": data_pred,
        "Estimator + SmoothNet": predicted_pos,
        "Ground truth": data_gt,
    }

    videoWriter = cv2.VideoWriter(
        os.path.join(vis_output_video_path, vis_output_video_name),
        cv2.VideoWriter_fourcc(*"mp4v"),
        FRAME_RATE,
        (
            SUB_FIG_SIZE * VIEW_NUM * len(anim_output) * SUB_FIG_UNIT,
            SUB_FIG_SIZE * 3 * SUB_FIG_UNIT,
        ),
    )

    for frame_i in trange(max(0, start_frame), min(len_seq, end_frame)):
        fig = plt.figure(figsize=(SUB_FIG_SIZE * VIEW_NUM * len(anim_output), SUB_FIG_SIZE * 3))
        plt.subplots_adjust(
            left=None, bottom=None, right=None, top=None, wspace=None, hspace=0.5
        )  # 用来调整子图之间的间距
        last_joint = None
        for view_i in range(VIEW_NUM):
            view_camera = VIEW_CAMERA[view_i]
            for index, (title, data) in enumerate(anim_output.items()):
                ax_3d = fig.add_subplot(
                    3,  # 3行
                    len(anim_output) * VIEW_NUM + 1,  # 列数 2*3=6
                    (view_i) * len(anim_output)
                    + index
                    + 1,  # 利用view_i和index来确定位置 1*3+1=4; 1*3+2=5; 1*3+3=6
                    projection="3d",  # 3D图
                )
                ax_3d.view_init(elev=view_camera[0], azim=view_camera[1])

                ax_3d.set_xticklabels([])
                ax_3d.set_yticklabels([])
                ax_3d.set_zticklabels([])

                ax_3d.dist = SKELETON_AX_DIST
                ax_3d.set_title(title, fontsize=3 * SUB_FIG_SIZE)

                plot_skeleton(ax_3d, data[frame_i, :, :], SKELRTON_COLOR)
        ax_3dtr = fig.add_subplot(
            3,  # 3行
            len(anim_output) * VIEW_NUM + 1,  # 列数 2*3=6
            len(anim_output) * VIEW_NUM
            + 1,  # 利用view_i和index来确定位置 1*3+1=4; 1*3+2=5; 1*3+3=6
            projection="3d",  # 3D图
        )
        ax_3dtr.view_init(elev=view_camera[0], azim=view_camera[1])

        ax_3dtr.set_xticklabels([])
        ax_3dtr.set_yticklabels([])
        ax_3dtr.set_zticklabels([])

        ax_3dtr.dist = SKELETON_AX_DIST
        ax_3dtr.set_title("right hand trajectory", fontsize=3 * SUB_FIG_SIZE)
        if frame_i > 0:
            plot_joint_trajectory(ax_3dtr, data_pred[:frame_i, :, :], data_gt[:frame_i, :, :])

        ax_acc = fig.add_subplot(3, 1, 2)
        ax_acc.set_title("Accel Error Visualize", fontsize=4 * SUB_FIG_SIZE)

        ax_mpjpe = fig.add_subplot(3, 1, 3)
        ax_mpjpe.set_title("MPJPE Visualize", fontsize=4 * SUB_FIG_SIZE)

        ax_acc.plot(
            acc_in[:frame_i], color=(202 / 255, 0 / 255, 32 / 255), label="Estimator (Accel)"
        )
        ax_acc.plot(acc_out[:frame_i], "c", label="Estimator + SmoothNet (Accel)")

        ax_acc.legend()
        ax_acc.grid(True)
        ax_acc.set_xlabel("Frame", fontsize=3 * SUB_FIG_SIZE)
        ax_acc.set_ylabel("Mean Acceleration Error (mm/s2)", fontsize=3 * SUB_FIG_SIZE)
        ax_acc.set_xlim((max(0, start_frame), min(len_seq, end_frame)))
        ax_acc.set_ylim((0, np.max((np.max(acc_in), np.max(acc_out)))))
        ax_acc.tick_params(axis="x", labelsize=3 * SUB_FIG_SIZE)
        ax_acc.tick_params(axis="y", labelsize=3 * SUB_FIG_SIZE)
        ax_acc.legend(fontsize=3 * SUB_FIG_SIZE)

        ax_mpjpe.plot(
            mpjpe_in[:frame_i], color=(202 / 255, 0 / 255, 32 / 255), label="Estimator (MPJPE)"
        )
        ax_mpjpe.plot(mpjpe_out[:frame_i], "c", label="Estimator + SmoothNet (MPJPE)")

        ax_mpjpe.legend()
        ax_mpjpe.grid(True)
        ax_mpjpe.set_xlabel("Frame", fontsize=3 * SUB_FIG_SIZE)
        ax_mpjpe.set_ylabel("Mean Position Error (mm)", fontsize=3 * SUB_FIG_SIZE)
        ax_mpjpe.set_xlim((max(0, start_frame), min(len_seq, end_frame)))
        ax_mpjpe.set_ylim((0, np.max((np.max(mpjpe_in), np.max(mpjpe_out)))))
        ax_mpjpe.tick_params(axis="x", labelsize=3 * SUB_FIG_SIZE)
        ax_mpjpe.tick_params(axis="y", labelsize=3 * SUB_FIG_SIZE)
        ax_mpjpe.legend(fontsize=3 * SUB_FIG_SIZE)

        canvas = FigureCanvasAgg(plt.gcf())
        canvas.draw()
        final_img = np.array(canvas.renderer.buffer_rgba())[:, :, [2, 1, 0]]

        # plt.savefig("tmp" + str(frame_i) + ".png")

        videoWriter.write(final_img)
        # 储存最后一帧
        if frame_i == min(len_seq, end_frame) - 1:
            cv2.imwrite(os.path.join(vis_output_video_path, "last_frame.png"), final_img)
            cv2.imwrite(os.path.join(vis_output_video_path, "last_frame.tif"), final_img)
            plt.close()

            plt_temp = plt.figure()
            ax_temp = plt_temp.add_subplot(
                1,
                1,
                1,
                projection="3d",
            )
            ax_temp.view_init(elev=view_camera[0], azim=view_camera[1])
            ax_temp.set_xticklabels([])
            ax_temp.set_yticklabels([])
            ax_temp.set_zticklabels([])
            ax_temp.dist = SKELETON_AX_DIST
            ax_temp.set_title("right hand trajectory", fontsize=3 * SUB_FIG_SIZE)
            if frame_i > 0:
                plot_joint_trajectory(ax_temp, data_pred[:frame_i, :, :], data_gt[:frame_i, :, :])
            plt.savefig(os.path.join(vis_output_video_path, "trajectory.png"))

        plt.close()

    videoWriter.release()
    print(
        f"Finish! The video is stored in "
        + os.path.join(vis_output_video_path, vis_output_video_name)
    )
