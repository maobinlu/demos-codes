'''
Author       : station geyuanliu@gmail.com
Date         : 2025-02-24 11:22:33
LastEditors: 解博炜
LastEditTime: 2025-03-05
FilePath: show_traj.py
Description  : 轨迹可视化函数
version      : 
notion       : 
'''
"""
Author: station geyuanliu@gmail.com
Date: 2025-02-13 14:09:53
LastEditors: station geyuanliu@gmail.com
LastEditTime: 2025-02-13 14:12:22
FilePath: /piper_hci/utils/show_traj.py
Description: 轨迹可视化函数
version: 
notion: 
"""
import matplotlib.pyplot as plt
import numpy as np
import sys

def show_trajectory3d(templist: list):
    """展示三维轨迹 默认两行排列
    Args:
        templist (list): 轨迹列表
    """
    fig = plt.figure()
    num_plots = len(templist)

    for i, temp in enumerate(templist):
        ax = fig.add_subplot(2, int(num_plots / 2), i + 1, projection="3d")
        n = len(temp)
        points = np.zeros((n, 3))
        for j, coord in enumerate(temp):
            points[j] = coord
        ax.plot(points[:, 0], points[:, 1], points[:, 2], color="blue")
        ax.set_xlabel("X 轴")
        ax.set_ylabel("Y 轴")
        ax.set_zlabel("Z 轴")
        ax.set_title(f"轨迹 {i + 1}")  # 添加标题
        ax.invert_yaxis()
        # 保存列表
        # np.save(action_name+f'/hand_trajectory_{i}.npy', points)

    plt.show()


def show_joints(temp_joints: np.array,save_path: str =None):
    '''展示关节角度曲线
    Args:
        temp_joints (np.array): array1(n,6)
        save_path (str, optional): 路径. Defaults to None.
    '''
    fig = plt.figure()
    joint_num = temp_joints.shape[1]
    times = np.arange(temp_joints.shape[0])

    for i in range(joint_num):
        ax = fig.add_subplot(joint_num, 1, i + 1)
        ax.plot(times, temp_joints[:, i])
        ax.set_xlabel("times")
        ax.set_ylabel(f"{i+1} joint")
        # ax.set_title(f"轨迹 {i + 1} 关节")  # 添加标题
        # np.save(action_name+f'/hand_trajectory_{i}.npy', points)
    if save_path is not None:
        plt.savefig(save_path)
    plt.show()

def show_joints_comp(temp_joints_list: list,save_path: str =None):
    '''对比两关节曲线

    Args:
        temp_joints_list (list): [array1(n,6),array2(n,6)]
        save_path (str, optional): 路径. Defaults to None.
    '''
    fig = plt.figure()
    joint_num = temp_joints_list[0].shape[1]
    times = np.arange(temp_joints_list[0].shape[0])
    for i in range(joint_num):
        ax = fig.add_subplot(joint_num, 1, i + 1)
        ax.plot(times, temp_joints_list[0][:, i],color='blue',label='pred')
        ax.plot(times, temp_joints_list[1][:, i],color='red',label='pro')
        ax.set_xlabel("times")
        ax.set_ylabel(f"{i+1} joint")
        # ax.set_title(f"轨迹 {i + 1} 关节")  # 添加标题
    if save_path is not None:
        plt.savefig(save_path)
    plt.show()

import plotly.graph_objs as go
import dash
from dash import dcc, html

def show_joints_web(temp_joints: np.array, port: int = 7025):
    '''展示关节角度曲线，并通过 Dash 生成 Web 可视化
    Args:
        temp_joints (np.array): 关节角度数据，形状为 (n, 6)，表示 n 个时间点，6 个关节
        port (int, optional): 本地服务器端口，默认为 7025
    '''
    # 创建 Dash 应用
    app = dash.Dash(__name__)

    # 创建显示关节数据的布局
    def create_layout():
        joint_num = temp_joints.shape[1]
        times = np.arange(temp_joints.shape[0])

        # 为每个关节创建一个子图
        figures = []
        for i in range(joint_num):
            figure = go.Figure()
            figure.add_trace(go.Scatter(x=times, y=temp_joints[:, i], mode='lines', name=f"Joint {i + 1}"))
            figure.update_layout(
                title=f"Joint {i + 1} Angle Over Time",
                xaxis_title="Time",
                yaxis_title=f"Joint {i + 1} Angle"
            )
            figures.append(figure)

        # 返回包含所有图表的布局
        return html.Div([
            dcc.Graph(figure=figures[i]) for i in range(joint_num)
        ])

    # 设置 Dash 应用的布局
    app.layout = create_layout

    # 启动服务器
    try:
        app.run_server(debug=True, port=port, use_reloader=False)  # 禁用自动重载
    except KeyboardInterrupt:
        print("服务器已停止.")
        sys.exit()

def show_joints_comp_web(temp_joints_list: list, port: int = 7025):
    '''展示关节角度曲线，并通过 Dash 生成 Web 可视化
    Args:
        temp_joints (list): 关节角度数据，形状为 [(n, 6),(n, 6)]，表示 n 个时间点，6 个关节
        port (int, optional): 本地服务器端口，默认为 7025
    '''
    # 创建 Dash 应用
    app = dash.Dash(__name__)

    # 创建显示关节数据的布局
    def create_layout():
        color_list = ['blue','red','green','yellow','black','purple','orange','pink','brown','gray']
        joint_num = temp_joints_list[0].shape[1]
        times = np.arange(temp_joints_list[0].shape[0])

        # 为每个关节创建一个子图
        figures = []
        for i in range(joint_num):
            figure = go.Figure()
            for j, temp in enumerate(temp_joints_list):
                figure.add_trace(go.Scatter(x=times, y=temp[:, i], mode='lines', name=f"Joint {i + 1} for data {j+1}",line=dict(color=color_list[j])))
            figure.update_layout(
                title=f"Joint {i + 1} Angle Over Time",
                xaxis_title="Time",
                yaxis_title=f"Joint {i + 1} Angle"
            )
            figures.append(figure)

        # 返回包含所有图表的布局
        return html.Div([
            dcc.Graph(figure=figures[i]) for i in range(joint_num)
        ])

    # 设置 Dash 应用的布局
    app.layout = create_layout

    # 启动服务器
    try:
        app.run_server(debug=True, port=port, use_reloader=False)  # 禁用自动重载
    except KeyboardInterrupt:
        print("服务器已停止.")
        sys.exit()

def show_trajectory3d_web(templist: list, port: int = 7025):
    '''展示三维轨迹，并通过 Dash 生成 Web 可视化
    Args:
        templist (list): 轨迹列表，每个轨迹为 (n, 3) 的数组
        port (int, optional): 本地服务器端口，默认为 7025
    '''
    # 创建 Dash 应用
    app = dash.Dash(__name__)

    # 创建显示轨迹数据的布局
    def create_layout():
        figures = []
        for i, temp in enumerate(templist):
            figure = go.Figure()
            figure.add_trace(go.Scatter3d(
                x=temp[:, 0],
                y=temp[:, 1],
                z=temp[:, 2],
                mode='lines',
                name=f"Trajectory {i + 1}"
            ))
            figure.update_layout(
                title=f"Trajectory {i + 1}",
                scene=dict(
                    xaxis_title="X 轴",
                    yaxis_title="Y 轴",
                    zaxis_title="Z 轴"
                ),
                margin=dict(l=0, r=0, b=0, t=30)  # 调整边距以增大图表显示区域
            )
            figures.append(figure)

        # 返回包含所有图表的布局
        return html.Div([
            dcc.Graph(figure=figures[i], style={'height': '600px'}) for i in range(len(figures))  # 增加图表高度
        ])

    # 设置 Dash 应用的布局
    app.layout = create_layout

    # 启动服务器
    try:
        app.run_server(debug=True, port=port, use_reloader=False)  # 禁用自动重载
    except KeyboardInterrupt:
        print("服务器已停止.")
        sys.exit()

def show_trajectory3d_comp_web(temp_pose_list:list,port:int = 7025):
    '''展示三维轨迹，并通过 Dash 生成 Web 可视化
    Args:
        temp_pose_list (list): 轨迹列表，每个轨迹为 (n, 3) 的数组
        port (int, optional): 本地服务器端口，默认为 7025
    '''
    # 创建 Dash 应用
    app = dash.Dash(__name__)

    # 创建显示轨迹数据的布局
    def create_layout():
        figures = []
        color_list = ['blue','red','green','yellow','black','purple','orange','pink','brown','gray']
        figure = go.Figure()
        for i, temp in enumerate(temp_pose_list):
            figure.add_trace(go.Scatter3d(
                x=temp[:, 0],
                y=temp[:, 1],
                z=temp[:, 2],
                mode='lines',
                line=dict(color=color_list[i]),
                name=f"Trajectory {i + 1}"
            ))
        figure.update_layout(
            title=f"Trajectory {i + 1}",
            scene=dict(
                xaxis_title="X 轴",
                yaxis_title="Y 轴",
                zaxis_title="Z 轴"
            ),
            margin=dict(l=0, r=0, b=0, t=30)  # 调整边距以增大图表显示区域
        )
        figures.append(figure)

        # 返回包含所有图表的布局
        return html.Div([
            dcc.Graph(figure=figures[i], style={'height': '600px'}) for i in range(len(figures))  # 增加图表高度
        ])

    # 设置 Dash 应用的布局
    app.layout = create_layout

    # 启动服务器
    try:
        app.run_server(debug=True, port=port, use_reloader=False)  # 禁用自动重载
    except KeyboardInterrupt:
        print("服务器已停止.")
        sys.exit()

def show_trajectory2(templist, cat=None, show_num=[0, -1]):
    import math

    fig = plt.figure()
    num_plots = len(templist)
    if cat != None:
        num_plots = num_plots + 1
    len_num = math.ceil(num_plots / 2)
    col_num = math.ceil(num_plots / len_num)
    for i, temp in enumerate(templist):
        ax = fig.add_subplot(len_num, col_num, i + 1, projection="3d")
        n = len(temp)
        hand_points = np.zeros((n, 3))
        for j, coord in enumerate(temp):
            hand_points[j] = coord
        ax.plot(hand_points[:, 0], hand_points[:, 1], hand_points[:, 2], color="blue")
        ax.set_xlabel("X 轴")
        ax.set_ylabel("Y 轴")
        ax.set_zlabel("Z 轴")
        ax.set_title(f"轨迹 {i + 1}")  # 添加标题
        ax.invert_yaxis()
    if cat != None:
        ax = fig.add_subplot(len_num, col_num, num_plots, projection="3d")
        temp1 = templist[cat[0]]
        temp2 = templist[cat[1]]
        n = len(temp1)
        hand_points = np.zeros((n, 3))
        hand_points2 = np.zeros((n, 3))
        for j, coord in enumerate(temp1):
            hand_points[j] = coord
            hand_points2[j] = temp2[j]
        ax.plot(
            hand_points[show_num[0]: show_num[1], 0],
            hand_points[show_num[0]: show_num[1], 1],
            hand_points[show_num[0]: show_num[1], 2],
            color="blue",
        )
        ax.plot(
            hand_points2[show_num[0]: show_num[1], 0],
            hand_points2[show_num[0]: show_num[1], 1],
            hand_points2[show_num[0]: show_num[1], 2],
            color="red",
        )
        ax.set_xlabel("X 轴")
        ax.set_ylabel("Y 轴")
        ax.set_zlabel("Z 轴")
        ax.set_title(f"and")  # 添加标题
        ax.invert_yaxis()
        # 设置视角
        # ax.view_init(elev=view[0], azim=view[1])
    # return fig
    plt.show()

