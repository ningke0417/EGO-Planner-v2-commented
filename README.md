# EGO-Planner-v2
Swarm Playground, the codebase of the paper "[Swarm of micro flying robots in the wild](https://www.science.org/doi/10.1126/scirobotics.abm5954)".

# Installation-Free Usage

Please follow the [tutorial PDF file](swarm-playground/[README]_Brief_Documentation_for_Swarm_Playground.pdf) with corresponding videos ([1](swarm-playground/main_ws/WatchMe_main.mp4), [2](swarm-playground/formation_ws/WatchMe_formation.mp4), [3](swarm-playground/tracking_ws/WatchMe_tracking.mp4), [4](swarm-playground/interlaced_flight_ws/WatchMe_interlaced_flights.mp4)) to run the code.

This work was born out of [MINCO](https://github.com/ZJU-FAST-Lab/GCOPTER).
If you find it interesting, please give both repos stars generously. Thanks.

<img src="images/cover.jpg" alt="drawing" width="400"/>
roslaunch ego_planner formation_4uav_criterion_no_split_wall.launch rviz:=true 
cd /home/robot/Codes/EGO-Planner-criterion-validation/swarm-playground/formation_ws
source devel/setup.zsh
roslaunch ego_planner formation_4uav_criterion_auto_split_wall.launch rviz:=true
cd /home/robot/Codes/EGO-Planner-criterion-validation/swarm-playground/formation_ws
source devel/setup.zsh
roslaunch ego_planner formation_6uav_no_split_wall.launch rviz:=true
cd /home/robot/Codes/EGO-Planner-criterion-validation
source /opt/ros/noetic/setup.zsh
source swarm-playground/formation_ws/devel/setup.zsh

roslaunch ego_planner formation_6uav_criterion_auto_split_wall.launch

cd /home/robot/Codes/EGO-Planner-criterion-validation/swarm-playground/formation_ws
source devel/setup.zsh
roslaunch ego_planner formation_6uav_sparse_random.launch

# 8-UAV 2x4 formation (1 m spacing)
分布式开发版（默认仍为集中式，可显式切换）：
```bash
roslaunch ego_planner rectangle_8uav_fixed_split_merge_wall.launch coordination_mode:=distributed
roslaunch ego_planner rectangle_8uav_fixed_split_merge_wall.launch coordination_mode:=centralized
```
编译、通信协议、测试和当前限制见 [分布式编队说明](docs/distributed-formation.md)。

分布式换槽现在支持八机跨列重新分配：最终八个矩形槽位不变，物理编号不变，
允许绕到右侧的飞机接手右侧槽位，而不必为恢复原编号顺序横穿返回。
`swap_scope` 默认 `all`；随机环境默认开启换槽，固定墙默认关闭。显式测试命令：

```bash
# 随机环境，八机完整换槽。
roslaunch ego_planner rectangle_8uav_sparse_random.launch \
  coordination_mode:=distributed swap_enabled:=true swap_scope:=all
# 固定墙也要测试换槽时，需显式开启。
roslaunch ego_planner rectangle_8uav_fixed_split_merge_wall.launch \
  coordination_mode:=distributed swap_enabled:=true swap_scope:=all
# 旧四组前后交换：swap_enabled:=true swap_scope:=column
# 不换槽对照：swap_enabled:=false
```

以上场景分别运行。完整换槽使用各机局部地图认证的报价、轮换协调者和八机确认，
没有固定中央分配节点；不是经典无协调者价格拍卖，也不保证物理轨迹原子切换。
只在首次分裂后、所有成员距各自当前终点不超过 8 米时评估；进入 0.8 米以内的飞机
锁定自己的槽位，其他未到站飞机仍可交换剩余槽位。未知/受阻直线不报价，所有当前
目标仍须有有效认证；收益不足或确认不全就保留原目标。新消息和规划器接口更新后需完整编译
`catkin_make -j2 -l2`、重新 source 并重启仿真，不能只编译 agent。

固定墙环境：
roslaunch ego_planner rectangle_8uav_fixed_split_merge_wall.launch
随机障碍环境：
roslaunch ego_planner rectangle_8uav_sparse_random.launch

从github上把最新更新/在其他电脑上传上去的更新同步到本地电脑：
git pull origin criterion-validation


cd ~/Codes/EGO-Planner-criterion-validation/swarm-playground/formation_ws
source /opt/ros/noetic/setup.zsh
source devel/setup.zsh
分布式固定墙环境：
roslaunch ego_planner rectangle_8uav_fixed_split_merge_wall.launch \
  coordination_mode:=distributed
分布式随机障碍物场景：
roslaunch ego_planner rectangle_8uav_sparse_random.launch \              
  coordination_mode:=distributed
从本机连接高性能电脑:
ssh rise-lab@10.252.41.159
在高性能电脑运行命令：
cd /home/rise-lab/split-merge/EGO-Planner-v2-commented/swarm-playground/formation_ws
source /opt/ros/noetic/setup.zsh
source devel/setup.zsh
分布式固定墙环境：
roslaunch ego_planner rectangle_8uav_fixed_split_merge_wall.launch \
  coordination_mode:=distributed
分布式随机障碍物场景：
roslaunch ego_planner rectangle_8uav_sparse_random.launch \
  coordination_mode:=distributed
断开某一个无人机的通信命令：
另开一个终端运行：
cd /home/rise-lab/split-merge/EGO-Planner-v2-commented/swarm-playground/formation_ws
source /opt/ros/noetic/setup.zsh
source devel/setup.zsh
编队协调失效，规划和运动继续：
rosnode kill /drone_1_formation_agent
规划继续，但轨迹无法执行，产生重规划风暴：
rosnode kill /drone_1_traj_server
不再规划，traj_server 检测心跳丢失并保持最后位置：
rosnode kill /drone_1_ego_planner_node