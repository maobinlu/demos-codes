import numpy as np
import casadi as ca
import yaml

import time

# Quaternion Multiplication
def quat_mult(q1,q2):
    ans = ca.vertcat(q2[0,:] * q1[0,:] - q2[1,:] * q1[1,:] - q2[2,:] * q1[2,:] - q2[3,:] * q1[3,:],
           q2[0,:] * q1[1,:] + q2[1,:] * q1[0,:] - q2[2,:] * q1[3,:] + q2[3,:] * q1[2,:],
           q2[0,:] * q1[2,:] + q2[2,:] * q1[0,:] + q2[1,:] * q1[3,:] - q2[3,:] * q1[1,:],
           q2[0,:] * q1[3,:] - q2[1,:] * q1[2,:] + q2[2,:] * q1[1,:] + q2[3,:] * q1[0,:])
    return ans

# Quaternion-Vector Rotation
def rotate_quat(q1,v1):
    ans = quat_mult(quat_mult(q1, ca.vertcat(0, v1)), ca.vertcat(q1[0,:],-q1[1,:], -q1[2,:], -q1[3,:]))
    return ca.vertcat(ans[1,:], ans[2,:], ans[3,:]) # to covert to 3x1 vec

def RK4(f_c:ca.Function, X0, U, dt, M:int):
    DT = dt/M
    X1 = X0
    for _ in range(M):
        k1 = DT*f_c(X1,        U)
        k2 = DT*f_c(X1+0.5*k1, U)
        k3 = DT*f_c(X1+0.5*k2, U)
        k4 = DT*f_c(X1+k3,     U)
        X1 = X1+(k1+2*k2+2*k3+k4)/6
    # F = ca.Function('F', [X0, U], [X1] ,['X0', 'U'], ['X1'])
    return X1

def EulerIntegral(f_c:ca.Function, X0, U, dt, M:int):
    DT = dt/M
    X1 = X0
    for _ in range(M):
        X1 = X1 + DT*f_c(X1, U)
    
    return X1

def constrain(a, lb, ub):
    if a<lb:
        a=lb
    if a>ub:
        a=ub
    return a
  

class QuadrotorModel(object):
    def __init__(self, cfg_f):
        
        self._m = 1.0         # total mass
        self._arm_l = 0.23    # arm length
        self._c_tau = 0.0133  # torque constant
        
        self._G = 9.81
        self._J = np.diag([0.01, 0.01, 0.02])     # inertia
        self._J_inv = np.linalg.inv(self._J)
        self._D = np.diag([0.6, 0.6, 0.6])
        
        self._v_xy_max = ca.inf
        self._v_z_max = ca.inf
        self._omega_xy_max = 5
        self._omega_z_max = 1
        self._vim_max = 1
        self._T_max = 4.179
        self._T_min = 0

        self.load(cfg_f)
        
        self._X_lb = [-ca.inf, -ca.inf, -ca.inf,
                      -self._v_xy_max, -self._v_xy_max, -self._v_z_max,
                      -1,-1,-1,-1,
                      -self._omega_xy_max, -self._omega_xy_max, -self._omega_z_max,
                      -self._vim_max,-self._vim_max,-self._vim_max,-self._vim_max,-self._vim_max,-self._vim_max]
        self._X_ub = [ca.inf, ca.inf, ca.inf,
                      self._v_xy_max, self._v_xy_max, self._v_z_max,
                      1,1,1,1,
                      self._omega_xy_max, self._omega_xy_max, self._omega_z_max,
                      self._vim_max,self._vim_max,self._vim_max,self._vim_max,self._vim_max,self._vim_max]

        self._U_lb = [self._T_min, self._T_min, self._T_min, self._T_min]
        self._U_ub = [self._T_max, self._T_max, self._T_max, self._T_max]

    def load(self, cfg_f):
      with open(cfg_f, 'r') as f:
        cfg = yaml.load(f, Loader=yaml.FullLoader)
        
        if "mass" in cfg:
          self._m = cfg["mass"]
        else:
          print("No mass specified in " + cfg_f)
        
        if "arm_length" in cfg:
          self._arm_l = cfg["arm_length"]
        else:
          print("No arm length specified in " + cfg_f)

        if "inertia" in cfg:
          self._J = np.diag(cfg["inertia"])
          self._J_inv = np.linalg.inv(self._J)
        else:
          print("No inertia specified in " + cfg_f)
        
        if "torque_coeff" in cfg:
          self._c_tau = cfg["torque_coeff"]
        else:
          print("No torque coefficient specified in " + cfg_f)
        
        if "drag_coeff" in cfg:
          self._D = np.diag(cfg["drag_coeff"])
        else:
           print("No drag coefficient specified in " + cfg_f)
        
        if "v_xy_max" in cfg:
          self._v_xy_max = cfg["v_xy_max"]
        else:
          self._v_xy_max = ca.inf
        if "v_z_max" in cfg:
          self._v_z_max = cfg["v_z_max"]
        else:
          self._v_z_max = ca.inf
        
        if "omega_xy_max" in cfg:
          self._omega_xy_max = cfg["omega_xy_max"]
        else:
          print("No max angular velocity xy specfied in " + cfg_f)
        if "omega_z_max" in cfg:
          self._omega_z_max = cfg["omega_z_max"]
        else:
          print("No max angular velocity z specfied in " + cfg_f)
        
        if "thrust_min" in cfg:
          self._T_min = cfg["thrust_min"]
        else:
          print("No min thrust specified in " + cfg_f)
        if "thrust_max" in cfg:
          self._T_max = cfg["thrust_max"]
        else:
          print("No max thrust specified in " +cfg_f)

#
#   T1    T3
#     \  /
#      \/
#      /\
#     /  \
#   T4    T2
#
    def dynamics(self):
        px, py, pz = ca.SX.sym('px'), ca.SX.sym('py'), ca.SX.sym('pz')
        vx, vy, vz = ca.SX.sym('vx'), ca.SX.sym('vy'), ca.SX.sym('vz')
        qw, qx, qy, qz = ca.SX.sym('qw'), ca.SX.sym('qx'), ca.SX.sym('qy'), ca.SX.sym('qz')
        wx, wy, wz = ca.SX.sym('wx'), ca.SX.sym('wy'), ca.SX.sym('wz')
        v_1,v_2,v_3,v_4,v_5,v_6 = ca.SX.sym('v_1'), ca.SX.sym('v_2'), ca.SX.sym('v_3'), ca.SX.sym('v_4'), ca.SX.sym('v_5'), ca.SX.sym('v_6')
        
        T1, T2, T3, T4 = ca.SX.sym('T1'), ca.SX.sym('T2'), ca.SX.sym('T3'), ca.SX.sym('T4')

        taux = self._arm_l/np.sqrt(2)*(T1+T4-T2-T3)
        tauy = self._arm_l/np.sqrt(2)*(T1+T3-T2-T4)
        tauz = self._c_tau*(T3+T4-T1-T2)
        thrust = (T1+T2+T3+T4)

        wsin = np.array([0.1, 0.2, 0.2])
        phi = [None] * 3
        psi = np.array([1, 0])
        m = np.array([[0, 1], [-3, -2]])
        n = np.array([[0], [1]])
        tinv = [None] * 3
        for i in range(3):
            phi[i] = np.array([[0, 1], [-wsin[i] ** 2, 0]])

        for i in range(3):
            tinv[i] = np.array([[3 - wsin[i] ** 2, 2], [-2 * wsin[i] ** 2, 3 - wsin[i] ** 2]])

        PHI = np.block([[phi[0], np.zeros((2, 2)), np.zeros((2, 2))],
                        [np.zeros((2, 2)), phi[1], np.zeros((2, 2))],
                        [np.zeros((2, 2)), np.zeros((2, 2)), phi[2]]])   #6*6矩阵
        PSI = np.block([[psi, np.zeros((1, 2)), np.zeros((1, 2))],
                        [np.zeros((1, 2)), psi, np.zeros((1, 2))],
                        [np.zeros((1, 2)), np.zeros((1, 2)), psi]]) #3*6矩阵
        M = np.block([[m, np.zeros((2, 2)), np.zeros((2, 2))],
                      [np.zeros((2, 2)), m, np.zeros((2, 2))],
                      [np.zeros((2, 2)), np.zeros((2, 2)), m]]) #6*6矩阵
        N = np.block([[n, np.zeros((2, 1)), np.zeros((2, 1))],
                      [np.zeros((2, 1)), n, np.zeros((2, 1))],
                      [np.zeros((2, 1)), np.zeros((2, 1)), n]]) #6*3矩阵
        TINV = np.block([[tinv[0], np.zeros((2, 2)), np.zeros((2, 2))],
                        [np.zeros((2, 2)), tinv[1], np.zeros((2, 2))],
                        [np.zeros((2, 2)), np.zeros((2, 2)), tinv[2]]])  #6*6矩阵
        tau = ca.veccat(taux, tauy, tauz)
        w = ca.veccat(wx, wy, wz)
        v_im = ca.veccat(v_1, v_2, v_3, v_4, v_5, v_6)
        w_dot = self._J_inv@( tau - ca.cross(w,self._J@w) + (PSI@TINV)@v_im)
        v_dot = (M + (N@PSI)@TINV)@v_im + N@tau - ((M@N)@ self._J)@w

        fdrag =rotate_quat(ca.veccat(qw,qx,qy,qz) ,self._D@rotate_quat(ca.veccat(qw,-qx,-qy,-qz), ca.veccat(vx,vy,vz)))

        X_dot = ca.vertcat(
            vx,
            vy,
            vz,
            2 * (qw * qy + qx * qz) * (-thrust/self._m) - fdrag[0],
            2 * (qy * qz - qw * qx) * (-thrust/self._m) - fdrag[1],
            (qw * qw - qx * qx - qy * qy + qz * qz) * (-thrust/self._m) + self._G - fdrag[2],
            0.5 * (-wx * qx - wy * qy - wz * qz),
            0.5 * (wx * qw + wz * qy - wy * qz),
            0.5 * (wy * qw - wz * qx + wx * qz),
            0.5 * (wz * qw + wy * qx - wx * qy),
            w_dot[0],
            w_dot[1],
            w_dot[2],
            v_dot[0],
            v_dot[1],
            v_dot[2],
            v_dot[3],
            v_dot[4],
            v_dot[5]
        )  # 19*1矩阵

        X = ca.vertcat(px, py, pz,
                       vx, vy, vz,
                       qw, qx, qy, qz,
                       wx, wy, wz,
                       v_1, v_2, v_3, v_4, v_5, v_6) # 19*1矩阵
        U = ca.vertcat(T1, T2, T3, T4) 

        fx = ca.Function('f', [X, U], [X_dot], ['X', 'U'], ['X_dot'])
        return fx
    
    def ddynamics(self, dt):
        f = self.dynamics()
        X0 = ca.SX.sym("X", f.size1_in(0))
        U = ca.SX.sym("U", f.size1_in(1))
        
        X1 = RK4(f, X0, U, dt, 1)
        # X1 = EulerIntegral(f, X0, U, dt, 1)
        q_l = ca.sqrt(X1[6:10].T@X1[6:10])
        X1[6:10] = X1[6:10]/q_l
        
        return ca.Function("ddyn", [X0, U], [X1], ["X0", "U"], ["X1"])
    
    def ddynamics_dt(self):
        f = self.dynamics()
        X0 = ca.SX.sym("X", f.size1_in(0))
        U = ca.SX.sym("U", f.size1_in(1))
        dt = ca.SX.sym('dt')
        # X1 = RK4(f, X0, U, dt, 1)
        X1 = EulerIntegral(f, X0, U, dt, 1)
        
        q_l = ca.sqrt(X1[6:10].T@X1[6:10])
        X1[6:10] = X1[6:10]/q_l
        
        return ca.Function("ddyn_t", [X0, U, dt], [X1], ["X0", "U", "dt"], ["X1"])

class QuadrotorModel_Traj(object):
    def __init__(self, cfg_f):
        
        self._m = 1.0         # total mass
        self._arm_l = 0.23    # arm length
        self._c_tau = 0.0133  # torque constant
        
        self._G = 9.81
        self._J = np.diag([0.01, 0.01, 0.02])     # inertia
        self._J_inv = np.linalg.inv(self._J)
        self._D = np.diag([0.6, 0.6, 0.6])
        
        self._v_xy_max = ca.inf
        self._v_z_max = ca.inf
        self._omega_xy_max = 5
        self._omega_z_max = 1
        self._T_max = 4.179
        self._T_min = 0

        self.load(cfg_f)
        
        self._X_lb = [-ca.inf, -ca.inf, -ca.inf,
                      -self._v_xy_max, -self._v_xy_max, -self._v_z_max,
                      -1,-1,-1,-1,
                      -self._omega_xy_max, -self._omega_xy_max, -self._omega_z_max]
        self._X_ub = [ca.inf, ca.inf, ca.inf,
                      self._v_xy_max, self._v_xy_max, self._v_z_max,
                      1,1,1,1,
                      self._omega_xy_max, self._omega_xy_max, self._omega_z_max]

        self._U_lb = [self._T_min, self._T_min, self._T_min, self._T_min]
        self._U_ub = [self._T_max, self._T_max, self._T_max, self._T_max]

    def load(self, cfg_f):
      with open(cfg_f, 'r') as f:
        cfg = yaml.load(f, Loader=yaml.FullLoader)
        
        if "mass" in cfg:
          self._m = cfg["mass"]
        else:
          print("No mass specified in " + cfg_f)
        
        if "arm_length" in cfg:
          self._arm_l = cfg["arm_length"]
        else:
          print("No arm length specified in " + cfg_f)

        if "inertia" in cfg:
          self._J = np.diag(cfg["inertia"])
          self._J_inv = np.linalg.inv(self._J)
        else:
          print("No inertia specified in " + cfg_f)
        
        if "torque_coeff" in cfg:
          self._c_tau = cfg["torque_coeff"]
        else:
          print("No torque coefficient specified in " + cfg_f)
        
        if "drag_coeff" in cfg:
          self._D = np.diag(cfg["drag_coeff"])
        else:
           print("No drag coefficient specified in " + cfg_f)
        
        if "v_xy_max" in cfg:
          self._v_xy_max = cfg["v_xy_max"]
        else:
          self._v_xy_max = ca.inf
        if "v_z_max" in cfg:
          self._v_z_max = cfg["v_z_max"]
        else:
          self._v_z_max = ca.inf
        
        if "omega_xy_max" in cfg:
          self._omega_xy_max = cfg["omega_xy_max"]
        else:
          print("No max angular velocity xy specfied in " + cfg_f)
        if "omega_z_max" in cfg:
          self._omega_z_max = cfg["omega_z_max"]
        else:
          print("No max angular velocity z specfied in " + cfg_f)
        
        if "thrust_min" in cfg:
          self._T_min = cfg["thrust_min"]
        else:
          print("No min thrust specified in " + cfg_f)
        if "thrust_max" in cfg:
          self._T_max = cfg["thrust_max"]
        else:
          print("No max thrust specified in " +cfg_f)

#
#   T1    T3
#     \  /
#      \/
#      /\
#     /  \
#   T4    T2
#
    def dynamics(self):
        px, py, pz = ca.SX.sym('px'), ca.SX.sym('py'), ca.SX.sym('pz')
        vx, vy, vz = ca.SX.sym('vx'), ca.SX.sym('vy'), ca.SX.sym('vz')
        qw, qx, qy, qz = ca.SX.sym('qw'), ca.SX.sym('qx'), ca.SX.sym('qy'), ca.SX.sym('qz')
        wx, wy, wz = ca.SX.sym('wx'), ca.SX.sym('wy'), ca.SX.sym('wz')
        
        T1, T2, T3, T4 = ca.SX.sym('T1'), ca.SX.sym('T2'), ca.SX.sym('T3'), ca.SX.sym('T4')

        taux = self._arm_l/np.sqrt(2)*(T1+T4-T2-T3)
        tauy = self._arm_l/np.sqrt(2)*(T1+T3-T2-T4)
        tauz = self._c_tau*(T3+T4-T1-T2)
        thrust = (T1+T2+T3+T4)
            
        tau = ca.veccat(taux, tauy, tauz)
        w = ca.veccat(wx, wy, wz)
        w_dot = self._J_inv@( tau - ca.cross(w,self._J@w) )

        fdrag =rotate_quat(ca.veccat(qw,qx,qy,qz) ,self._D@rotate_quat(ca.veccat(qw,-qx,-qy,-qz), ca.veccat(vx,vy,vz)))

        X_dot = ca.vertcat(
            vx,
            vy,
            vz,
            2 * (qw * qy + qx * qz) * (-thrust/self._m) - fdrag[0],
            2 * (qy * qz - qw * qx) * (-thrust/self._m) - fdrag[1],
            (qw * qw - qx * qx - qy * qy + qz * qz) * (-thrust/self._m) + self._G - fdrag[2],
            0.5 * (-wx * qx - wy * qy - wz * qz),
            0.5 * (wx * qw + wz * qy - wy * qz),
            0.5 * (wy * qw - wz * qx + wx * qz),
            0.5 * (wz * qw + wy * qx - wx * qy),
            w_dot[0],
            w_dot[1],
            w_dot[2]
        )

        X = ca.vertcat(px, py, pz,
                       vx, vy, vz,
                       qw, qx, qy, qz,
                       wx, wy, wz)
        U = ca.vertcat(T1, T2, T3, T4)

        fx = ca.Function('f', [X, U], [X_dot], ['X', 'U'], ['X_dot'])
        return fx
    
    def ddynamics(self, dt):
        f = self.dynamics()
        X0 = ca.SX.sym("X", f.size1_in(0))
        U = ca.SX.sym("U", f.size1_in(1))
        # print("X0 ",f.size1_in(0))
        X1 = RK4(f, X0, U, dt, 1)
        # X1 = EulerIntegral(f, X0, U, dt, 1)
        q_l = ca.sqrt(X1[6:10].T@X1[6:10])
        X1[6:10] = X1[6:10]/q_l
        
        return ca.Function("ddyn", [X0, U], [X1], ["X0", "U"], ["X1"])
    
    def ddynamics_dt(self):
        f = self.dynamics()
        X0 = ca.SX.sym("X", f.size1_in(0))
        U = ca.SX.sym("U", f.size1_in(1))
        dt = ca.SX.sym('dt')
        # print("X0 ",f.size1_in(0))
 
        # X1 = RK4(f, X0, U, dt, 1)
        X1 = EulerIntegral(f, X0, U, dt, 1)
        
        q_l = ca.sqrt(X1[6:10].T@X1[6:10])
        X1[6:10] = X1[6:10]/q_l
        
        return ca.Function("ddyn_t", [X0, U, dt], [X1], ["X0", "U", "dt"], ["X1"])


class QuadrotorSimpleModel(object):
    def __init__(self, cfg_f):
        
        self._G = 9.81
        self._D = np.diag([0.3, 0.3, 0.3])
        self._v_xy_max = ca.inf
        self._v_z_max = ca.inf
        self._omega_xy_max = 4
        self._omega_z_max = 1
        self._a_z_max = 17
        self._a_z_min = 0

        with open(cfg_f, 'r') as f:
            self._cfg = yaml.load(f, Loader=yaml.FullLoader)
        
        if 'omega_xy_max' in self._cfg:
            self._omega_xy_max = self._cfg['omega_xy_max']
        if 'omega_z_max' in self._cfg:
            self._omega_z_max = self._cfg['omega_z_max']
        if 'G' in self._cfg:
            self._G = self._cfg['G']
        if 'a_z_min' in self._cfg:
            self._a_z_min = self._cfg['a_z_min']
        if 'a_z_max' in self._cfg:
            self._a_z_max = self._cfg['a_z_max']
      
        print("_v_xy_max, _v_z_max: ", self._v_xy_max, self._v_z_max)
        print("_omega_xy_max, _omega_z_max: ", self._omega_xy_max, self._omega_z_max)
        print("_a_z_max, _a_z_min: ", self._a_z_max, self._a_z_min)
        
        self._X_lb = [-ca.inf, -ca.inf, -ca.inf,
                      -self._v_xy_max, -self._v_xy_max, -self._v_z_max,
                      -1,-1,-1,-1]
        self._X_ub = [ca.inf, ca.inf, ca.inf,
                      self._v_xy_max, self._v_xy_max, self._v_z_max,
                      1,1,1,1]
        self._U_lb = [self._a_z_min, -self._omega_xy_max, -self._omega_xy_max, -self._omega_z_max]
        self._U_ub = [self._a_z_max,  self._omega_xy_max,  self._omega_xy_max,  self._omega_z_max]
        
    def dynamics(self):
        px, py, pz = ca.SX.sym('px'), ca.SX.sym('py'), ca.SX.sym('pz')
        vx, vy, vz = ca.SX.sym('vx'), ca.SX.sym('vy'), ca.SX.sym('vz')
        qw, qx, qy, qz = ca.SX.sym('qw'), ca.SX.sym('qx'), ca.SX.sym('qy'), ca.SX.sym('qz')
        
        az_B = ca.SX.sym('az_B')
        wx, wy, wz = ca.SX.sym('wx'), ca.SX.sym('wy'), ca.SX.sym('wz')

        X = ca.vertcat(px, py, pz,
                       vx, vy, vz,
                       qw, qx, qy, qz)
        U = ca.vertcat(az_B, wx, wy, wz)

        fdrag =rotate_quat(ca.veccat(qw,qx,qy,qz) ,self._D@rotate_quat(ca.veccat(qw,-qx,-qy,-qz), ca.veccat(vx,vy,vz)))

        X_dot = ca.vertcat(
            vx,
            vy,
            vz,
            2 * (qw * qy + qx * qz) * az_B - fdrag[0],
            2 * (qy * qz - qw * qx) * az_B - fdrag[1],
            (qw * qw - qx * qx - qy * qy + qz * qz) * az_B + self._G - fdrag[2],
            0.5 * (-wx * qx - wy * qy - wz * qz),
            0.5 * (wx * qw + wz * qy - wy * qz),
            0.5 * (wy * qw - wz * qx + wx * qz),
            0.5 * (wz * qw + wy * qx - wx * qy)
        )

        fx = ca.Function('f', [X, U], [X_dot], ['X', 'U'], ['X_dot'])
        return fx
    
    def ddynamics(self, dt):
        f = self.dynamics()
        X0 = ca.SX.sym("X", f.size1_in(0))
        U = ca.SX.sym("U", f.size1_in(1))
        
        X1 = RK4(f, X0, U, dt, 1)
        q_l = ca.sqrt(X1[6:10].T@X1[6:10])
        X1[6:10] = X1[6:10]/q_l
        
        return ca.Function("ddyn", [X0, U], [X1], ["X0", "U"], ["X1"])
    
    def ddynamics_dt(self):
        f = self.dynamics()
        X0 = ca.SX.sym("X", f.size1_in(0))
        U = ca.SX.sym("U", f.size1_in(1))
        dt = ca.SX.sym('dt')
        
        X1 = RK4(f, X0, U, dt, 1)
        q_l = ca.sqrt(X1[6:10].T@X1[6:10])
        X1[6:10] = X1[6:10]/q_l
        
        return ca.Function("ddyn_t", [X0, U, dt], [X1], ["X0", "U", "dt"], ["X1"])


if __name__ == "__main__":
    quad = QuadrotorModel('quad.yaml')
    q_sim = QuadrotorSim(quad)
    
    U = np.array([2,1,0,0])
    for i in range(100):
        t1 = time.time()
        q_sim.step10ms(U)
        t2 = time.time()
        print(q_sim._X[6:10])
        print(t2-t1)
    
