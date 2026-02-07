import numpy as np
import modern_robotics as mr

# DH参数
a = [0, -30, 185.4, 0, 0, 0]
alpha = [0, -np.pi/2, 0, -np.pi/2, np.pi/2, -np.pi/2]
d = [0, 0, 0, 136.5, 0, 0]
theta_offset = [0, 0, 0, 0, 0, 0]  # 初始关节角度偏移

# 初始末端执行器配置M（假设所有关节角度为0）
M = np.array([[1, 0, 0, 0],
              [0, 1, 0, 0],
              [0, 0, 1, d[3]],
              [0, 0, 0, 1]])

# 构建螺旋轴screw_axes
screw_axes = np.zeros((6, 6))
current_transform = np.eye(4)

for i in range(6):
    # 从关节 i-1 到关节 i 的转换
    transform_i = mr.TransInv(mr.DH(a[i], alpha[i], d[i], theta_offset[i]))
    current_transform = current_transform @ transform_i
    
    # Z轴旋转，因此螺旋轴在基坐标系中的方向为当前转换矩阵的第三列
    omega = current_transform[:3, 2]  # 旋转轴
    p = current_transform[:3, 3]  # 位置向量
    v = -np.cross(omega, p)  # 位置向量和旋转轴的叉积
    screw_axes[i, :3] = omega
    screw_axes[i, 3:] = v

# 打印结果
print("Screw Axes (S):")
print(screw_axes)
print("End-Effector Configuration (M):")
print(M)
