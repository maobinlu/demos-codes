import numpy
import numpy as np
import math as m
import math
from sympy import im

pi = m.pi


class ik_robot_pip:
    def __init__(self):
        self.a1 = 0.030
        self.a2 = 0.18240564
        self.d4 = 0.1385
        self.d6 = 0.1205

        self.init_theata = np.array([0, -np.deg2rad(162.24), np.deg2rad(72.24), 0, 0, 0])

        self.a = [0, self.a1, self.a2, 0, 0, 0]
        self.d = [0, 0, 0, self.d4, 0, 0]
        self.ap = [0, -pi / 2, 0, -pi / 2, pi / 2, -pi / 2]
        self.T6W = numpy.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, self.d6], [0, 0, 0, 1]])
        self.TW6 = np.linalg.inv(self.T6W)
        self.rev = 2 * m.pi
        self.limits = np.array([[-m.pi, m.pi],
                                [0, m.pi],
                                [-m.pi/2, m.pi/2],
                                [-m.pi, m.pi],
                                [-m.pi/2, m.pi/2],
                                [-m.pi, m.pi]])

    def Ti(self, a, d, ap, t):
        ti = np.array(
            [
                [m.cos(t), -m.sin(t), 0, a],
                [m.sin(t) * m.cos(ap), m.cos(t) * m.cos(ap), -m.sin(ap), -m.sin(ap) * d],
                [m.sin(t) * m.sin(ap), m.cos(t) * m.sin(ap), m.cos(ap), m.cos(ap) * d],
                [0, 0, 0, 1],
            ]
        )
        return ti

    def inverse_kine(self, T0W):
        T06 = np.dot(T0W, self.TW6)  # 变换到坐标系6
        theta = []  # 存最后的多组解
        theta3 = []
        x = T06[0][3]
        y = T06[1][3]
        z = T06[2][3]
        r = x * x + y * y + z * z
        # 解算theta3并判断取值是否满足条件，得到多组theta3
        is1 = True
        is2 = True
        temp1 = (2 * self.a1 * m.sqrt(r - z * z) - r - self.a1 * self.a1 + self.a2 * self.a2 + self.d4 * self.d4) / (
            2 * self.a2 * self.d4
        )
        temp2 = (r + 2 * self.a1 * m.sqrt(r - z * z) + self.a1 * self.a1 - self.a2 * self.a2 - self.d4 * self.d4) / (
            2 * self.a2 * self.d4
        )

        if m.fabs(temp1) > 1:
            is1 = False
            theta3.append(0)
            theta3.append(0)
        else:
            theta3.append(m.asin(temp1))
            theta3.append(pi - m.asin(temp1))

        if m.fabs(temp2) > 1:
            is2 = False
            theta3.append(0)
            theta3.append(0)
        else:
            theta3.append(-m.asin(temp2))
            theta3.append(pi + m.asin(temp2))

        if not (is1 or is2):
            return False, None

        # 过程中需要用到的旋转矩阵
        R06 = T06[0:3, 0:3]
        R43 = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])

        # 根据多组theta3解算
        for t3 in theta3:
            s3 = m.sin(t3)
            c3 = m.cos(t3)
            k1 = -self.d4 * s3 + self.a2
            k2 = -self.d4 * c3
            k3 = self.a2 * self.a2 + self.d4 * self.d4 - 2 * self.a2 * self.d4 * s3 + self.a1 * self.a1
            dd = (r - k3) / (2 * self.a1)
            # 解算thetself.a2
            t2 = pi / 2 - m.atan2(k1 * dd + k2 * z, -k1 * z + k2 * dd)
            s2 = m.sin(t2)
            c2 = m.cos(t2)
            # 解算thetself.a1
            g1 = (self.a2 - self.d4 * s3) * c2 - self.d4 * c3 * s2 + self.a1
            t1 = m.atan2(g1 * y, g1 * x)
            s1 = m.sin(t1)
            c1 = m.cos(t1)

            # 计算得到R46，根据ZYZ旋转解456
            t = [t1, t2, t3]
            T03 = np.eye(4)
            for i in range(3):
                T03 = np.dot(T03, self.Ti(self.a[i], self.d[i], self.ap[i], t[i]))

            R30 = np.linalg.inv(T03[0:3, 0:3])
            R36 = np.dot(R30, R06)
            R = np.dot(R43, R36)

            beta = m.atan2(m.sqrt(R[2][0] ** 2 + R[2][1] ** 2), R[2][2])
            sb = m.sin(beta)
            alpha = m.atan2(R[1][2] / sb, R[0][2] / sb)
            gamma = m.atan2(R[2][1] / sb, -R[2][0] / sb)
            # 得到theta456
            t4 = alpha + pi
            t5 = beta
            t6 = gamma + pi
            t.append(t4)
            t.append(t5)
            t.append(t6)
            # 对数值进行处理
            t = np.array(t) - self.init_theata
            for i in range(6):

                # 绝对值超过2pi或接近2pi的，变到2pi以内
                # while t[i] >= 2 * pi:
                #     t[i] = t[i] - 2 * pi
                # while t[i] <= -2 * pi:
                #     t[i] = t[i] + 2 * pi
                # if m.fabs(t[i] - 2 * pi) < 0.000001:
                #     t[i] = 0
                # elif m.fabs(t[i] + 2 * pi) < 0.000001:
                #     t[i] = 0
                # # 对于三个2pi范围内的角度，将绝对值大于pi的变到pi。
                # if i == 0 or i == 3 or i == 5:
                #     if t[i] > pi:
                #         t[i] = t[i] - 2 * pi
                #     elif t[i] < -pi:
                #         t[i] = 2 * pi + t[i]

                if t[i] <= -self.rev:
                    t[i] %= -self.rev
                elif t[i] >= self.rev:
                    t[i] %= self.rev

                if round(t[i], 3) < round(self.limits[i][0], 3):
                    t[i] += self.rev
                elif round(t[i], 3) > round(self.limits[i][1], 3):
                    t[i] -= self.rev
                t[i] = round(t[i], 5)
                # 对第四关节和第六关节的角度进行处理
            if t[4] == 0:
                print('t[4] == 0')
                t[5] = t[5]+t[3]
                t[3] = 0
                # print(t)
            if self.check_joint_limits(t):
                theta.append(t)
                # print('--------------')
                # print(t)
        if theta == []:
            return False, None
        return True, np.array(theta)  # 返回theta

    def check_joint_limits(self, theta_list):
        for x in range(len(theta_list)):
            if round(theta_list[x], 3) < round(self.limits[x][0], 3) or round(theta_list[x], 3) > round(self.limits[x][1], 3):
                return False
        return True

    def ik_cal(self, T, theta_cur=np.array([0, 0, 0, 0, 0, 0])):
        ik_ok, thetas = self.inverse_kine(T)
        if ik_ok:
            ik_n = len(thetas)
            theta_cur = np.repeat(theta_cur, ik_n, axis=0).reshape(ik_n, 6)
            index = np.argmin(np.sum(np.abs(thetas - theta_cur), axis=1))  # 最接近的关节角度
            return ik_ok, thetas[index]
        else:
            return ik_ok, None

    def fk_cal(self, theta_list):
        theta_list = theta_list + self.init_theata
        T = np.eye(4)
        for i in range(6):
            T = np.dot(T, self.Ti(self.a[i], self.d[i], self.ap[i], theta_list[i]))
        T = np.dot(T, self.T6W)
        return T

    def work_space(self):
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        from tqdm import tqdm
        # T_o = self.fk_cal([0,  math.pi/4, -math.pi/4, 0, 0, 0])
        num_points = 50000  # 点数决定了工作空间的分辨率和计算的密度
        positions = []
        directions = []
        for _ in tqdm(range(num_points)):
            theta_list = [0, 0, 0, 0, 0, 0]
            for i in range(6):
                theta_list[i] = np.random.uniform(low=self.limits[i][0], high=self.limits[i][1])
            T = self.fk_cal(theta_list)
            positions.append(T[:3, 3])
            directions.append(T[:3, 2])  # Z轴方向
        positions = np.array(positions)
        directions = np.array(directions)
        # 只绘制方向是1,0,0的工作空间
        positions = positions[np.round(directions[:, 0], 3) == 1]
        directions = directions[np.round(directions[:, 0], 3) == 1]
        print('number', len(positions))
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # 使用quiver绘制箭头
        ax.quiver(positions[:, 0], positions[:, 1], positions[:, 2],
                  directions[:, 0], directions[:, 1], directions[:, 2],
                  length=0.1, normalize=True)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Workspace of the Robotic Arm with End-Effector Directions')
        plt.savefig('workspace.jpeg')
        fig, ax = plt.subplots(1, 2)
        ax[0].scatter(positions[:, 1], positions[:, 2], color='r')
        ax[0].set_title('yz-projection')
        ax[1].scatter(positions[:, 0], positions[:, 2], color='r')
        ax[1].set_title('xz-projection')
        plt.savefig('workspace_xyz.jpeg')
        plt.show

    def work_space2(self):
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        from tqdm import tqdm
        T_o = self.fk_cal([0,  math.pi/4, -math.pi/4, 0, 0, 0])
        T_o = np.round(T_o, 5)
        num_points = 50000  # 点数决定了工作空间的分辨率和计算的密度
        positions = []
        directions = []
        x_r = 0.3
        y_r = 0.3
        z_r = 0.3
        for _ in tqdm(range(num_points)):
            for i in range(3):
                x = np.random.uniform(low=-x_r, high=x_r)
                y = np.random.uniform(low=-y_r, high=y_r)
                z = np.random.uniform(low=-z_r, high=z_r)
            T = T_o.copy()
            T[0, 3] = x+T_o[0, 3]
            T[1, 3] = y+T_o[1, 3]
            T[2, 3] = z+T_o[2, 3]
            ik_ok, thetas = self.ik_cal(T)
            if ik_ok:
                positions.append(T[:3, 3])
                directions.append(T[:3, 2])  # Z轴方向
        positions = np.array(positions)
        directions = np.array(directions)
        # 只绘制方向是1,0,0的工作空间
        # positions = positions[np.round(directions[:, 0], 3) == 1]
        # directions = directions[np.round(directions[:, 0], 3) == 1]
        print('number', len(positions))
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # 使用quiver绘制箭头
        ax.quiver(positions[:, 0], positions[:, 1], positions[:, 2],
                  directions[:, 0], directions[:, 1], directions[:, 2],
                  length=0.1, normalize=True)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Workspace of the Robotic Arm with End-Effector Directions')
        plt.savefig('workspace.jpeg')
        fig, ax = plt.subplots(1, 2)
        ax[0].scatter(positions[:, 1], positions[:, 2], color='r')
        ax[0].set_title('yz-projection')
        ax[1].scatter(positions[:, 0], positions[:, 2], color='r')
        ax[1].set_title('xz-projection')
        plt.savefig('workspace_xyz.jpeg')


if __name__ == "__main__":
    ik = ik_robot_pip()
    t = [0, m.pi/4, -m.pi/4, 0, 0, 0]
    print(t)
    T = ik.fk_cal(t)
    print(np.around(T, 3))
    T[0, 3] = T[0, 3]-0.2
    # ok, the = ik.inverse_kine(T)
    ok, the = ik.ik_cal(T)
    # print(ok)
    if ok:
        print(np.around(the, 3))
    else:
        print("error")
    ik.work_space()
