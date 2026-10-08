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

固定墙环境：
roslaunch ego_planner rectangle_8uav_fixed_split_merge_wall.launch
随机障碍环境：
roslaunch ego_planner rectangle_8uav_sparse_random.launch

从github上把最新更新/在其他电脑上传上去的更新同步到本地电脑：
git pull origin criterion-validation
