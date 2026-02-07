from urdfpy import URDF
from pathlib import Path
from mr_urdf_loader import loadURDF
import numpy as np
import modern_robotics as mr
import pdb
import math


class ik_robot_mr:
    def __init__(self):
        self.M = np.array([[1.0, 0.0, 0.0, 0.2124],
                           [0.0, -1.0, 0.0, 0.0],
                           [0.0, -0.0, -1.0, -0.1385],
                           [0.0, 0.0, 0.0, 1.0],])
        self.Slist = np.array([[0, 0, 1, 0, 0, 0],
                               [0, 1, 0, 0, 0, 0.03],
                               [0, 1, 0, 0, 0, 0.212],
                               [0, 0, -1, 0, 0.212, 0],
                               [0, 1, 0, 0.138, 0, 0.212],
                               [0, 0, -1, 0, 0.212, 0],]).T
        self.initial_guesses = [[0, 0, -1.5708, 0, 0, 0],
                                [2.0944, 0, -1.5708, 0, 0, 0],
                                [-2.0944, 0, -1.5708, 0, 0, 0]]
        self.init = np.array([0, -np.deg2rad(162.24), np.deg2rad(72.24), 0, 0, 0])
        self.limits = np.array([[-math.pi, math.pi],
                                [0, math.pi],
                                [-math.pi/2, math.pi/2],
                                [-math.pi, math.pi],
                                [-math.pi/2, math.pi/2],
                                [-math.pi, math.pi]])
        self.rev = 2 * math.pi

    def ik_cal(self, T, guess_theta=None):
        initial_guesses = self.initial_guesses
        if guess_theta is not None:
            initial_guesses = [guess_theta + self.init] + initial_guesses
        for guess in initial_guesses:
            theta_list, success = mr.IKinSpace(self.Slist, self.M, T, guess, 0.01, 0.01)
            theta_list = theta_list - self.init
            solution_found = True
            # Check to make sure a solution was found and that no joint limits were violated
            if success:
                for x in range(len(theta_list)):
                    if theta_list[x] <= -self.rev:
                        theta_list[x] %= -self.rev
                    elif theta_list[x] >= self.rev:
                        theta_list[x] %= self.rev

                    if round(theta_list[x], 3) < round(self.limits[x][0], 3):
                        theta_list[x] += self.rev
                    elif round(theta_list[x], 3) > round(self.limits[x][1], 3):
                        theta_list[x] -= self.rev

                solution_found = self.check_joint_limits(theta_list)
            else:
                solution_found = False

            if solution_found:

                return True, np.around(theta_list, 3)

        return False, np.around(theta_list, 3)

    def fk_cal(self, theta_list):
        theta_list = theta_list + self.init
        T = mr.FKinSpace(self.M, self.Slist, theta_list)
        return T
    
    def work_space(self):
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        num_points = 50000  # 点数决定了工作空间的分辨率和计算的密度
        positions = []
        directions = []
        for _ in range(num_points):
            theta_list = np.zeros((6,1))
            for i in range(6):
                theta_list[i] = np.random.uniform(low=self.limits[i][0], high=self.limits[i][1])
            T = mr.FKinSpace(self.M, self.Slist, theta_list+self.init)
            positions.append(T[:3, 3])
            directions.append(T[:3, 2])  # Z轴方向
        positions = np.array(positions)
        directions = np.array(directions)
        #只绘制方向是1,0,0的工作空间
        positions = positions[directions[:, 0] > 0.9]
        directions = directions[directions[:, 0] > 0.9]

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
        fig,ax = plt.subplots(1,2)
        ax[0].scatter(positions[:,1], positions[:,2], color='r')
        ax[0].set_title('yz-projection')
        ax[1].scatter(positions[:,0], positions[:,2], color='r')
        ax[1].set_title('xz-projection')
        plt.savefig('workspace_xyz.jpeg')


    
    def check_joint_limits(self, theta_list):
        for x in range(len(theta_list)):
            if round(theta_list[x], 3) < round(self.limits[x][0], 3) or round(theta_list[x], 3) > round(self.limits[x][1], 3):
                return False
        return True


if __name__ == "__main__":
    ik = ik_robot_mr()
    T = ik.fk_cal(np.array([0, math.pi/3, -math.pi/3, 0, 0, 0]))
    ik_ok, theta = ik.ik_cal(T)
    # pdb.set_trace()
    print(T)
    print(ik_ok)
    print(theta)
    print("end")
    ik.work_space()
