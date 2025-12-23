#!/usr/bin/env bash
# 保存为 run_all.sh，然后 chmod +x run_all.sh

# 配置
WORKSPACE=/home/yy/sight/nav_mapping
ROS_SETUP=/opt/ros/humble/setup.bash

# 一个通用的命令前缀：先退出 conda，再加载 ROS 和工作空间环境
CMD_PREFIX="conda deactivate 2>/dev/null || true; \
source $ROS_SETUP; \
cd $WORKSPACE; \
source install/setup.bash; "

# 先清理可能残留的进程
pkill -9 -f ros 2>/dev/null || true
pkill -9 -f gazebo 2>/dev/null || true

# 1. 仿真 + 机器人
gnome-terminal -- bash -lc "$CMD_PREFIX ros2 launch wpr_simulation2 explore.launch.py; exec bash"

# 2. SLAM
sleep 3
gnome-terminal -- bash -lc "$CMD_PREFIX ros2 launch slam_pkg slam.launch.py use_sim_time:=true; exec bash"

# 3. Nav2 导航（使用 nav_pkg）
sleep 10
gnome-terminal -- bash -lc "$CMD_PREFIX ros2 launch nav_pkg nav.launch.py use_sim_time:=true; exec bash"

# 4. 自主探索节点
sleep 8
gnome-terminal -- bash -lc "$CMD_PREFIX ros2 launch explore_pkg explore.launch.py; exec bash"