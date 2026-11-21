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
在高性能电脑运行的命令：
cd /home/rise-lab/split-merge/EGO-Planner-v2-official-baseline/swarm-playground/formation_ws
source /opt/ros/noetic/setup.zsh
source devel/original_baseline/setup.zsh
固定墙场景：
roslaunch ego_planner rectangle_8uav_fixed_split_merge_wall.launch

roslaunch ego_planner rectangle_8uav_fixed_split_merge_wall.launch \
  wall_size_y:=8.0
随机障碍物场景：
roslaunch ego_planner rectangle_8uav_sparse_random.launch

## 与 commented 工程一致的小块障碍物切换

在加载了本工程 `devel/original_baseline` 的 Noetic 终端中，8 个小块、第 1 套位置：

```bash
roslaunch ego_planner rectangle_8uav_sparse_random.launch \
  random_seed:=17 small_obstacle_count:=8 small_position_seed:=1
```

保持 `random_seed:=17`，将 `small_position_seed` 依次设置为 `1` 到 `10`，即十组
可复现的小块位置；`small_obstacle_count` 调整小块数量，`0` 表示全部去掉。
大型、堆叠和窄缝障碍物固定。恢复原默认布局用
`small_obstacle_count:=6 small_position_seed:=-1`。

本基线本身使用官方分布式规划器。`commented` 工程的对应命令额外带
`coordination_mode:=distributed`，并使用相同地图参数。
详见 [十组场景与两工程命令](docs/matched_random_scenes.md)，固定参数和逐组地图校验和见
[场景清单](docs/random_scene_suite.json)。

第二个实验场景（8 个小块、位置种子 1）的 10 次轻量录包命令和文件命名，见
[场景 02 录包说明](docs/scene02_bag_benchmark.md)。录包脚本支持
`--file-prefix scene02_ego_baseline_r17_small8_pos1`，自动生成带 `run01`–`run10`
编号的 bag、位置 CSV 和指标文件。

第三个实验场景（8 个小块、位置种子 2）的命令和命名见
[场景 03 录包说明](docs/scene03_bag_benchmark.md)；已完成各批次的结果见
[录包实验索引](recordings/EXPERIMENT_INDEX.md)。

第四个实验场景（8 个小块、位置种子 3）的 10 次轻量录包命令和文件命名见
[场景 04 录包说明](docs/scene04_bag_benchmark.md)。

## 容器中 roslaunch 卡住、RViz 无法打开

本机已观察到 `roslaunch` 在导入 Python `ssl` 时阻塞于系统库的内存映射阶段，
此时尚未启动 RViz；X11 显示连接正常。可在 `split-merge` 容器的新终端中运行：

```bash
cd /home/rise-lab/split-merge/EGO-Planner-v2-official-baseline
bash tools/with_noetic_runtime.sh \
  roslaunch ego_planner rectangle_8uav_fixed_split_merge_wall.launch wall_size_y:=8.0
```

脚本自动加载 Noetic 和本工程的 `original_baseline` 环境，并为这次运行创建内容相同的
独立 OpenSSL 库副本，绕过共享文件的加载阻塞；系统库、规划参数和 RViz 配置均不改动。
关闭本次运行后会清理其临时副本。该方式已实际验证 RViz 窗口、8 米墙体参数与八路里程计。
其他 Noetic 命令也可放在 `bash tools/with_noetic_runtime.sh` 后面运行。
