依赖安装脚本

    cd src/wpr_simulation2/scripts/
    ./install_for_humble.sh

colcon build

source install/setup.bash

地图在src/wpr_simulation2/worlds/explore.world

factory1.world里的路径改一下，可以搜索‘/home/yy/factory_ws/src/models’，修改这些路径

编辑查看地图 

    gazebo explore.world

在insert中添加模型

1.仿真建图（会加载比较久）

    ros2 launch wpr_simulation2 explore.launch.py 
    ros2 launch slam_pkg slam.launch.py 
    ros2 run wpr_simulation2 keyboard_vel_cmd # 控制机器人移动

地图保存
控制小车终端按X退出，ros2 run nav2_map_server map_saver_cli -f map
ros2 run nav2_map_server map_saver_cli -f ~/factory_ws/src/wpr_simulation2/maps/map
将生成在主目录的两个map文件移动到/src/wpr_simulation2/maps/

2.导航

    ros2 launch wpr_simulation2 explore.launch.py 
    ros2 launch nav_pkg nav.launch.py

先用2D Pose Estimate确定机器人初始位置
用nav2 goal给定目标点

可以add相机获取到的画面（也有深度图）

3.自主探索建图

    ros2 launch wpr_simulation2 explore.launch.py  # 启动仿真
    ros2 launch slam_pkg slam.launch.py           # 启动建图
    ros2 launch nav_pkg nav.launch.py             # 启动导航
    ros2 launch explore_pkg explore.launch.py     # 启动自主探索

或者使用集成启动文件（启动仿真、建图和探索节点，导航需单独启动）：

    ros2 launch explore_pkg explore_full.launch.py  # 启动仿真、建图和探索
    ros2 launch nav_pkg nav.launch.py                # 需要单独启动导航

探索参数说明：
- exploration_radius: 探索半径（米），负数表示无限制
  例如：ros2 launch explore_pkg explore.launch.py exploration_radius:=10.0
- frontier_distance_threshold: 前沿点聚类距离阈值（米），默认0.5
- min_frontier_size: 最小前沿点数量，默认20
- goal_timeout: 目标超时时间（秒），默认60.0
- update_frequency: 更新频率（Hz），默认1.0

探索功能说明：
- 自动检测地图中的前沿点（未知区域与已知区域的边界）
- 选择最佳前沿点作为探索目标
- 自动发送导航命令到nav2
- 实时计算并发布建图完成度（/explore/progress话题）
- 可视化前沿点（/explore/frontiers话题，可在RViz中查看）

后面可以根据自己的机器人去修改