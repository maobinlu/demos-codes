from urdfpy import URDF
from pathlib import Path
from mr_urdf_loader import loadURDF
import numpy as np
import modern_robotics as mr
import pdb
import math


j2_init = -np.deg2rad(162.24)
j3_init = np.deg2rad(72.24)
urdf_name = "./arm/urdf/arm.urdf"
MR = loadURDF(urdf_name)
M = np.around(MR["M"], 4)
Slist = np.around(MR["Slist"], 4)
Mlist = MR["Mlist"]
Glist = MR["Glist"]
Blist = MR["Blist"]
print("urdf M\n", M)
print("urdf Slist\n", Slist)
# region
M = np.array([[1.0, 0.0, 0.0, 0.2154],
              [0.0, -1.0, 0.0, 0.0],
              [0.0, -0.0, -1.0, -0.1365],
              [0.0, 0.0, 0.0, 1.0],])

Slist = np.array([[0.0, 0.0, 1.0, -0.0, -0.0, -0.0,],
                  [0.0, 1.0, 0.0, -0.0, -0.0, 0.03],
                  [0.0, 1.0, 0.0, -0.0, -0.0, 0.2154],
                  [1.0, 0.0, 0.0, 0.0, -0.1365, -0.0],
                  [0.0, 1.0, 0.0, 0.1365, -0.0, 0.2154],
                  [1.0, 0.0, 0.0, 0.0, -0.1365, -0.0],]).T
# endregion

M = np.array([[1.0, 0.0, 0.0, 0.2124],
              [0.0, -1.0, 0.0, 0.0],
              [0.0, -0.0, -1.0, -0.1385],
              [0.0, 0.0, 0.0, 1.0],])
Slist = np.array([[0, 0, 1, 0, 0, 0],
                  [0, 1, 0, 0, 0, 0.03],
                  [0, 1, 0, 0, 0, 0.212],
                  [0, 0, -1, 0, 0.212, 0],
                  [0, 1, 0, 0.138, 0, 0.212],
                  [0, 0, -1, 0, 0.212, 0],]).T

# Slist = np.array([[0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
#                   [0.0, 1.0, 0.0, -0.267, 0.0, 0.0],
#                   [0.0, 1.0, 0.0, -0.5515, 0.0, 0.0535],
#                   [0.0, 0.0, -1.0, 0.0, 0.131, 0],
#                   [0.0, 1.0, 0.0, -0.209, 0.0, 0.131],
#                   [0.0, 0.0, -1.0, 0.0, 0.207, 0.0]]).T

# M = np.array([[1.0, 0.0, 0.0, 0.207],
#               [0.0, -1.0, 0.0, 0.0],
#               [0.0, 0.0, -1.0, 0.112],
#               [0.0, 0.0, 0.0, 1.0]])
print("M\n", M)
print("Slist\n", Slist)

# M = np.array(
#     [[0.,      0.,      1.,     0.3055],
#      [0.,     -1.,      0.,     0],
#      [1.,      0.,     -0.,     0.1845],
#      [0.,      0.,      0.,     1.]])
# Slist = np.array(
#     [[0.,     0.,     0.,      1.,      0.,      1,],
#      [0.,     1.,     -1.,     0.,     -1.,      0,],
#      [-1.,    0.,      0.,     0.,      0.,      0,],
#      [0,      0.128,   0.1845, 0.,      0.1845,  0,],
#      [0.0369, 0,       0.,     0.1845,  0.,      0.1845],
#      [0.,     0.0669,  0.1097, 0,      -0.0318,  0]])


# region
# def IK_2_real(thetalist):
#     thetalist[2] = thetalist[2] + np.deg2rad(162.24)
#     thetalist[3] = thetalist[3] - np.deg2rad(72.24)
#     return thetalist


# # 请在此处添加用于计算M矩阵的其他代码


# def real_2_IK(thetalist):
#     thetalist[2] = thetalist[2] - np.deg2rad(162.24)
#     thetalist[3] = thetalist[3] + np.deg2rad(72.24)
#     return thetalist
# endregion

actuated_joints_num = MR["actuated_joints_num"]
thetalist_0 = np.array([0, 0, 0, 0, 0, 0])
thetalist_1 = np.array([0, math.pi/3 + j2_init, -math.pi/3 + j3_init, 0, 0, 0])
# thetalist_1 = real_2_IK(thetalist_1)
# print("FKinSpace\n", mr.FKinSpace(M, Slist.T, thetalist_1))
T = np.around(np.array(mr.FKinSpace(M, Slist, thetalist_1)), 6)
print("T\n", T)
print("real\n", thetalist_1)
print("IKinSpace\n", mr.IKinSpace(Slist, M, T, thetalist_0, 0.0001, 0.0001))
theta_list, ik = mr.IKinSpace(Slist, M, T, thetalist_0, 0.0001, 0.0001)

# def dh_to_matrix(a, alpha, d, theta):
#     return np.array(
#         [
#             [np.cos(theta), -np.sin(theta), 0, a],
#             [
#                 np.sin(theta) * np.cos(alpha),
#                 np.cos(theta) * np.cos(alpha),
#                 -np.sin(alpha),
#                 -d * np.sin(alpha),
#             ],
#             [
#                 np.sin(theta) * np.sin(alpha),
#                 np.cos(theta) * np.sin(alpha),
#                 np.cos(alpha),
#                 d * np.cos(alpha),
#             ],
#             [0, 0, 0, 1],
#         ]
#     )


# def dh_to_matrix_i(a, alpha, d, theta):
#     return np.array(
#         [
#             [-np.sin(theta), -np.cos(theta), 0, a],
#             [
#                 np.cos(theta) * np.cos(alpha),
#                 -np.sin(theta) * np.cos(alpha),
#                 -np.sin(alpha),
#                 -d * np.sin(alpha),
#             ],
#             [
#                 np.cos(theta) * np.sin(alpha),
#                 -np.sin(theta) * np.sin(alpha),
#                 np.cos(alpha),
#                 d * np.cos(alpha),
#             ],
#             [0, 0, 0, 1],
#         ]
#     )


# def wq_to_slist(w, q):
#     Slist = []
#     print(w.shape, q.shape)
#     for i in range(w.shape[0]):
#         p1 = w[i]
#         p2 = np.cross(-w[i], q[i])
#         p = np.concatenate((p1, p2), axis=0)
#         Slist.append(p)
#         print(p1)
#         print(p2)
#     return np.array(Slist)


# def dh_to_M_and_Slist(dh_parameters, w, q):
#     M = np.eye(4)
#     for a, alpha, d, theta in dh_parameters:
#         print(a, alpha, d, theta)
#         M = np.dot(M, dh_to_matrix(a, alpha, d, theta))
#     Slist = wq_to_slist(w, q)
#     return M, Slist


# dh_parameters = np.array(
#     [
#         [0, 0, 0, 0],
#         [0.030, -np.pi / 2, 0, 0],
#         [0.1854, 0, 0, 0],
#         [0, -np.pi / 2, 0.1365, 0],
#         [0, np.pi / 2, 0, 0],
#         [0, -np.pi / 2, 0, 0],
#     ]
# )
# w = np.array([[0, 0, 1], [0, 1, 0], [0, 1, 0], [1, 0, 0], [0, 1, 0], [1, 0, 0]])
# q = np.array(
#     [
#         [0, 0, 0],
#         [0.0300, 0, 0],
#         [0.2154, 0, 0],
#         [0.2154, 0, -0.1365],
#         [0.2154, 0, -0.1365],
#         [0.2154, 0, -0.1365],
#     ]
# )

# # dh_parameters = np.array([[0, 0, 0, 0],
# #                          [0.030, -np.pi/2, 0, 0],
# #                          [0.1854, 0, 0, 0],
# #                          [0, -np.pi/2, 0.1365, 0],
# #                          [0, np.pi/2, 0, 0],
# #                          [0, -np.pi/2, 0, 0]])
# # w = np.array([[0, 0, 1],
# #               [0, 1, 0],
# #               [0, 1, 0],
# #               [1, 0, 0],
# #               [0, 1, 0],
# #               [1, 0, 0]])
# # q = np.array([[0, 0, 0],
# #               [0.0300, 0, 0],
# #               [0.14657, 0, 0.05657],
# #               [-0.005, 0, 0.05657],
# #               [-0.005, 0, 0.05657],
# #               [-0.005, 0, 0.05657]])

# # M, Slist = dh_to_M_and_Slist(dh_parameters, w, q)
# # print("M\n", np.around(M, 4))
# # print("Slist\n", np.around(Slist, 4))


# 定义符号变量
# theta1, theta2, theta3, theta4, theta5, theta6 = sp.symbols('theta1:7')
# a1, a2, d4 = sp.symbols('a1 a2 d4')
# alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = sp.symbols('alpha1:7')


theta1, theta2, theta3, theta4, theta5, theta6 = 0, 0, 0, 0, 0, 0
a1, a2, d4 = 0.030, 0.18240564, 0.1385

# 修改后的DH参数表，格式：(alpha, a, d, theta)
DH_params = [
    (0, 0, 0, theta1),
    (-np.pi / 2, a1, 0, theta2),
    (0, a2, 0, theta3),
    (-np.pi / 2, 0, d4, theta4),
    (np.pi / 2, 0, 0, theta5),
    (-np.pi / 2, 0, 0, theta6),
]

# 定义转换矩阵
T = np.eye(4)
slist = np.zeros((6, 6))

for i, (alpha, a, d, theta) in enumerate(DH_params):
    # 创建当前连杆的转换矩阵
    Ti = np.array([
        [np.cos(theta), -np.sin(theta), 0, a],
        [
            np.sin(theta) * np.cos(alpha),
            np.cos(theta) * np.cos(alpha),
            -np.sin(alpha),
            -np.sin(alpha) * d,
        ],
        [
            np.sin(theta) * np.sin(alpha),
            np.cos(theta) * np.sin(alpha),
            np.cos(alpha),
            np.cos(alpha) * d,
        ],
        [0, 0, 0, 1],
    ]
    )

    # 更新总转换矩阵
    T = np.dot(T, Ti)
    omega = T[:3, 2]  # 旋转轴
    p = T[:3, 3]  # 位置向量
    v = -np.cross(omega, p)  # 位置向量和旋转轴的叉积
    slist[i, :3] = omega
    slist[i, 3:] = v

    # # 存储关节轴向量的方向（z轴方向）和位置（原点位置）
    # slist[i, :3] = T[:3, 2].T  # Transpose to match dimensions
    # slist[i, 3:] = T[:3, 3].T  # Transpose to match dimensions

# 打印slist矩阵和最终的转换矩阵T
# print(np.around(slist, 3))
# print(np.around(T, 3))


class uxarm6:
    Slist = np.array([[0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0, -0.267, 0.0, 0.0],
                      [0.0, 1.0, 0.0, -0.5515, 0.0, 0.0535],
                      [0.0, 0.0, -1.0, 0.0, 0.131, 0],
                      [0.0, 1.0, 0.0, -0.209, 0.0, 0.131],
                      [0.0, 0.0, -1.0, 0.0, 0.207, 0.0]]).T

    M = np.array([[1.0, 0.0, 0.0, 0.207],
                  [0.0, -1.0, 0.0, 0.0],
                  [0.0, 0.0, -1.0, 0.112],
                  [0.0, 0.0, 0.0, 1.0]])

    Guesses = [[0, 0, -1.5708, 0, 0, 0],
               [2.0944, 0, -1.5708, 0, 0, 0],
               [-2.0944, 0, -1.5708, 0, 0, 0]]
