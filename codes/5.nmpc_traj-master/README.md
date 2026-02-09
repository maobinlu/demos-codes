# 决策：针对未知环境探索复杂度高等难题，提出了多重知识表达的分层探索方法，实现了非完全信息下复杂动态系统自主决策

针对未知环境探索中状态空间复杂、信息不完全、动态变化快等难题，本研究提出了一种融合多重知识表达的分层自主探索与决策框架。该框架通过在环境表征层、决策规划层与评估学习层中构建并利用互补的知识模型，使多无人机系统能够在非完全信息、动态不确定的复杂环境下，实现高效、智能且目标导向的自主探索与稳健决策。

安装依赖：

sudo apt update

sudo apt-get install libgoogle-glog-dev

sudo apt install ros-noetic-pcl-conversions

sudo apt install ros-noetic-pcl-msgs

sudo apt install ros-noetic-pcl-ros

sudo apt-get install libpcl-dev

sudo apt-get install libyaml-cpp-dev

sudo apt install -y g++ git libglfw3-dev libglew-dev libjpeg-dev libjsoncpp-dev libpng-dev python3-dev libdw-dev

sudo apt-get install python3-catkin-tools

sudo apt install libncurses-dev

程序运行：

roslaunch fast_fly_waypoint fast_fly.launch

<launch>
    <arg name="drone_id" default="0"/>
      
    <node pkg="fast_fly_waypoint" name="pendulum" type="pendulum.py" output="screen">
    
    </node>
    
    <node pkg="fast_fly_waypoint" name="track" type="track_robo.py" output="screen">
    
        <remap from="~odom" to="/airsim_node/drone_1/pose_gt" />   
        
        <remap from="~track_traj" to="/plan/track_traj" />
        
    </node>
    
    <node pkg="fast_fly_waypoint" name="plan" type="plan.py" output="screen">
    
        <remap from="~odom" to="/airsim_node/drone_1/pose_gt" />   
        
    </node>

</launch>
