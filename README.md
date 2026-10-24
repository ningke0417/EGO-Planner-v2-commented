# EGO-Planner-v2

当前工作分支在官方 `9d85475` 基础上提供与判据验证项目一致的八机固定墙和随机障碍物场景。
编译、运行及对比边界见 [八机原版对照说明](docs/original_8uav_baseline.md)。

Swarm Playground, the codebase of the paper "[Swarm of micro flying robots in the wild](https://www.science.org/doi/10.1126/scirobotics.abm5954)".

# Installation-Free Usage

Please follow the [tutorial PDF file](swarm-playground/[README]_Brief_Documentation_for_Swarm_Playground.pdf) with corresponding videos ([1](swarm-playground/main_ws/WatchMe_main.mp4), [2](swarm-playground/formation_ws/WatchMe_formation.mp4), [3](swarm-playground/tracking_ws/WatchMe_tracking.mp4), [4](swarm-playground/interlaced_flight_ws/WatchMe_interlaced_flights.mp4)) to run the code.

This work was born out of [MINCO](https://github.com/ZJU-FAST-Lab/GCOPTER).
If you find it interesting, please give both repos stars generously. Thanks.

<img src="images/cover.jpg" alt="drawing" width="400"/>
cd /home/robot/Codes/EGO-Planner-v2-commented/swarm-playground/formation_ws
source /opt/ros/noetic/setup.zsh
source devel/original_baseline/setup.zsh
固定墙场景：
roslaunch ego_planner rectangle_8uav_fixed_split_merge_wall.launch
随机障碍物场景：
roslaunch ego_planner rectangle_8uav_sparse_random.launch