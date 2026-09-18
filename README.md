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