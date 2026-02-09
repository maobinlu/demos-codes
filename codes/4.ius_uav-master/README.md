# 调控：针对不可靠环境下调控抗扰性差等难题，提出了复杂不确定系统安全稳健调控方法，实现了对未知动态环境下复杂系统的智能学习与安全稳健调控，有效提升了复杂系统调控的稳定性和安全性
# IUS_uav
Gitee配置参考：https://hqqtqwvkaq8.feishu.cn/wiki/INYgw5lkciAN2gkUJFJcUgkJnXg

针对复杂不确定环境下系统抗扰性弱、调控可靠性不足的挑战，本研究提出了一套融合控制障碍函数（Control Barrier Function, CBF）与内模模型预测控制（Internal Model-based Model Predictive Control, IMMPC）的安全稳健协同调控框架。该框架面向多无人机系统在动态未知环境中的协同作业问题，通过主动安全约束与抗扰预测控制相结合，实现了复杂系统在不确定干扰下的智能学习、安全防护与动态优化调控，显著提升了系统运行的稳定性与安全性。

安装依赖：

mkdir -p build

cd build

cmake ..  -DACADOS_WITH_QPOASES=ON  -DACADOS_WITH_OSQP=OFF  -DACADOS_WITH_OPENMP=ON   -DCMAKE_BUILD_TYPE=Release

make install -j4

程序运行：

roslaunch px4 livox.launch

cd jy_2025_race

./sim_test.sh

# 如果实际路径与sim_test.sh中的路径不一致，改写成实际路径
