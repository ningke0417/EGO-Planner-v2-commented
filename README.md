# EGO-Planner-v2

当前工作分支在官方 `9d85475` 基础上提供与判据验证项目一致的固定墙和随机障碍物场景，
包含稀疏随机障碍和密集随机方柱，支持 6、8、12、16 机矩形编队。
编译、运行及对比边界见 [八机原版对照说明](docs/original_8uav_baseline.md)。

## 可复用矩形编队：6 / 12 / 16 机

统一使用 `rectangle_formation.launch`，通过行列参数选择规模，默认打开 RViz。
场景、感知和无人机模型与判据验证项目共用相同实现条件。
行沿飞行方向 x 从前向后排列，列沿 y 排列，相邻间距默认均为 1 m。

```zsh
cd ~/Codes/EGO-Planner-v2-commented/swarm-playground/formation_ws
source /opt/ros/noetic/setup.zsh
source devel/original_baseline/setup.zsh
```

固定墙（下面三条命令分别运行）：

```zsh
# 两行三列：6 机
roslaunch ego_planner rectangle_formation.launch scene:=fixed_wall formation_rows:=2 formation_columns:=3
# 三行四列：12 机
roslaunch ego_planner rectangle_formation.launch scene:=fixed_wall formation_rows:=3 formation_columns:=4
# 四行四列：16 机
roslaunch ego_planner rectangle_formation.launch scene:=fixed_wall formation_rows:=4 formation_columns:=4
```

随机障碍物（下面三条命令分别运行）：

```zsh
roslaunch ego_planner rectangle_formation.launch scene:=sparse_random formation_rows:=2 formation_columns:=3
roslaunch ego_planner rectangle_formation.launch scene:=sparse_random formation_rows:=3 formation_columns:=4
roslaunch ego_planner rectangle_formation.launch scene:=sparse_random formation_rows:=4 formation_columns:=4
```

新入口的固定墙默认宽 6 m，与参考项目的 `distributed_fixed_wall.launch` 一致；
原八机入口仍保留 4 m 默认值。随机地图默认种子为 17、6 个小障碍物；
各规模使用同一地图，修改编队规模不会重新选种子或放大障碍物。
编译、间距调整、静态预览及验证命令见 [矩形编队说明](docs/rectangular_formations.md)。

## 与判据验证项目一致的密集随机方柱

对应参考项目的 `distributed_cluttered_random.launch`。地图为 40×18×3 m，
默认种子 17、60 根方柱，横截面两个边长均在 0.45–1.1 m 范围内；
编队中心起终点为 x=-17/17 m，两端各留出 5 m 障碍物净空，柱间最小几何间隔 0.6 m。

在已加载上述环境的终端中，下面命令分别运行：

```zsh
# 六机
roslaunch ego_planner rectangle_formation.launch scene:=cluttered_random formation_rows:=2 formation_columns:=3
# 八机
roslaunch ego_planner rectangle_formation.launch scene:=cluttered_random formation_rows:=2 formation_columns:=4
# 十二机
roslaunch ego_planner rectangle_formation.launch scene:=cluttered_random formation_rows:=3 formation_columns:=4
# 十六机
roslaunch ego_planner rectangle_formation.launch scene:=cluttered_random formation_rows:=4 formation_columns:=4
```

默认打开 RViz。`random_seed`、`small_obstacle_count`、`small_size_min`、`small_size_max`
和 `obstacle_separation` 可调整布局、数量、尺寸及间隔；相同参数下各规模共用相同地图。
该场景关闭大型长障碍、堆叠块和强制窄通道，规划仍使用本项目的官方 EGO 编队规划器。

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
