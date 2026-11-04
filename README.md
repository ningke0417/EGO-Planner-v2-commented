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

只改变随机小块障碍物的位置时，保持 `random_seed` 和其他地图参数不变，
单独修改 `small_position_seed`（非负整数）：

```bash
roslaunch ego_planner rectangle_8uav_sparse_random.launch \
  coordination_mode:=distributed random_seed:=17 small_position_seed:=42
```

大型障碍、堆叠障碍和窄通道的位置、尺寸保持不变，小块障碍的尺寸也保持不变。
相同的两个种子可复现相同地图；`small_position_seed:=-1`（默认值）恢复原始布局。
该参数仅在启动时读取，切换布局需结束当前仿真后重新启动。
首次使用此功能需在 `formation_ws` 编译 `catkin_make -j2 -l2` 并重新 source。

以上场景分别运行。完整换槽使用各机局部地图认证的报价、轮换协调者和八机确认，
没有固定中央分配节点；不是经典无协调者价格拍卖，也不保证物理轨迹原子切换。
只在首次分裂后、所有成员距各自当前终点不超过 8 米时评估；进入 0.8 米以内的飞机
锁定自己的槽位，其他未到站飞机仍可交换剩余槽位。未知/受阻直线不报价，所有当前
目标仍须有有效认证；收益不足或确认不全就保留原目标。新消息和规划器接口更新后需完整编译
`catkin_make -j2 -l2`、重新 source 并重启仿真，不能只编译 agent。

固定墙环境：
roslaunch ego_planner rectangle_8uav_fixed_split_merge_wall.launch
第一堵墙当前默认长 8 m；批量轻量化录包、成功率和全员到达计时见 [录包说明](docs/bag-experiments.md)。
随机障碍环境：
roslaunch ego_planner rectangle_8uav_sparse_random.launch

仅调整随机小块的数量和位置，大型、堆叠及窄缝障碍保持原布局：
```bash
roslaunch ego_planner rectangle_8uav_sparse_random.launch \
  coordination_mode:=distributed random_seed:=17 \
  small_obstacle_count:=8 small_position_seed:=1
```
`small_obstacle_count` 是小块数量；更换 `small_position_seed` 为 `0、1、2……`
可换一套小块位置，非负位置种子之间切换时小块尺寸也保持一致。
保持 `random_seed:=17`，不要用它切换小块，否则大型障碍物也会重新生成。
`small_position_seed:=-1 small_obstacle_count:=6` 恢复原默认小块布局。

第二场景批量录包（8 个小块、位置种子 1，10 轮）：
```bash
python3 scripts/record_fixed_wall_batch.py \
  --scene sparse_random --runs 10 --random-seed 17 \
  --small-obstacle-count 8 --small-position-seed 1 \
  --experiment-label scene02_distributed_map17_small08_pos01 --port 11422
```
需要在已 source Noetic 和 formation_ws 的仓库根目录运行；命名、记录内容与统计口径见 [录包说明](docs/bag-experiments.md)。

第三场景批量录包（8 个小块、位置种子 2，10 轮）：
```bash
python3 scripts/record_fixed_wall_batch.py \
  --scene sparse_random --runs 10 --random-seed 17 \
  --small-obstacle-count 8 --small-position-seed 2 \
  --experiment-label scene03_distributed_map17_small08_pos02 --port 11423
```
第三场景 10 轮已完成并逐包校验：[实验报告](bags/scene03_random_small08_posseed02_mapseed17_distributed_20261025/scene03_experiment_report.md)、[各轮指标](bags/scene03_random_small08_posseed02_mapseed17_distributed_20261025/scene03_trial_metrics.csv)、[bag 与位置文件索引](bags/scene03_random_small08_posseed02_mapseed17_distributed_20261025/scene03_recording_file_index.csv)。

任务1后台对比实验：ours（分布式并开启换槽）、Zhou、Quan 的固定墙第一堵墙均为6m，
八机2×4，每项目50轮，共150轮。串行交替运行，逐轮更新结果，结束后自动停止。
[统一实验协议](docs/comparison-task01.md) ·
[当前进度](bags/comparison_task01_wall06m_50each_20261027_154538/status.json) ·
[实时汇总报告](bags/comparison_task01_wall06m_50each_20261027_154538/REPORT.md) ·
[全部单轮指标与文件路径](bags/comparison_task01_wall06m_50each_20261027_154538/all_trial_metrics.csv) ·
[后台日志](bags/comparison_task01_wall06m_50each_20261027_154538_supervisor.log)

任务2后台对比实验：第一堵墙7m，三项目各50轮，共150轮，沿用任务1算法配置和统一统计口径。
[实验说明](docs/comparison-task02.md) ·
[当前进度](bags/comparison_task02_wall07m_50each_20261027_191753/status.json) ·
[实时汇总报告](bags/comparison_task02_wall07m_50each_20261027_191753/REPORT.md) ·
[全部单轮指标与文件路径](bags/comparison_task02_wall07m_50each_20261027_191753/all_trial_metrics.csv) ·
[后台日志](bags/comparison_task02_wall07m_50each_20261027_191753_supervisor.log)

任务3后台对比实验：第一堵墙8m，三项目各50轮，共150轮，沿用前两批算法配置和统一统计口径。
[实验说明](docs/comparison-task03.md) ·
[当前进度](bags/comparison_task03_wall08m_50each_20261027_215902/status.json) ·
[实时汇总报告](bags/comparison_task03_wall08m_50each_20261027_215902/REPORT.md) ·
[全部单轮指标与文件路径](bags/comparison_task03_wall08m_50each_20261027_215902/all_trial_metrics.csv) ·
[后台日志](bags/comparison_task03_wall08m_50each_20261027_215902_supervisor.log)

第四场景批量录包（8 个小块、位置种子 3，八机分布式并开启换槽，10 轮）：
```bash
python3 scripts/record_fixed_wall_batch.py \
  --scene sparse_random --runs 10 --random-seed 17 \
  --small-obstacle-count 8 --small-position-seed 3 --swap-enabled \
  --experiment-label scene04_distributed_map17_small08_pos03 --port 0
```
第四场景 10 轮已完成并逐包校验：[实验报告](bags/scene04_random_small08_posseed03_mapseed17_distributed_20261025/scene04_experiment_report.md)、[各轮指标](bags/scene04_random_small08_posseed03_mapseed17_distributed_20261025/scene04_trial_metrics.csv)、[bag 与位置文件索引](bags/scene04_random_small08_posseed03_mapseed17_distributed_20261025/scene04_recording_file_index.csv)。

默认分布式随机障碍场景轻量化录包（10 轮，自动统计成功率、全员到达时间及论文式45编队误差）：
```bash
python3 scripts/record_fixed_wall_batch.py --scene sparse_random --runs 10 --port 11421
```
先在 Noetic 环境中 source formation_ws/devel/setup.bash；本机 Docker 命令与统计口径见 [录包说明](docs/bag-experiments.md)。

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
第一堵墙的长度为8m:
  roslaunch ego_planner rectangle_8uav_fixed_split_merge_wall.launch \
  coordination_mode:=distributed wall_size_y:=8.0
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

任务4后台对比实验：第一堵墙9m，ours、Zhou、Quan各50轮，共150轮。
全部150次完成且逐包SHA256核验通过后，顺序控制器自动启动任务5。
[独立实验说明](docs/comparison-task04.md) ·
[调度状态](bags/comparison_task04_wall09m_50each_20261028_003704_schedule.json) ·
[当前轮次](bags/comparison_task04_wall09m_50each_20261028_003704/status.json) ·
[独立汇总报告](bags/comparison_task04_wall09m_50each_20261028_003704/REPORT.md) ·
[全部单轮指标与文件路径](bags/comparison_task04_wall09m_50each_20261028_003704/all_trial_metrics.csv)

任务5后台对比实验：第一堵墙10m，ours、Zhou、Quan各50轮，共150轮。
等待任务4全部完成并通过完整性检查后自动开始；本任务完成汇总后整个队列停止。
[独立实验说明](docs/comparison-task05.md) ·
[调度状态](bags/comparison_task05_wall10m_50each_20261028_003704_schedule.json) ·
[当前轮次](bags/comparison_task05_wall10m_50each_20261028_003704/status.json) ·
[独立汇总报告](bags/comparison_task05_wall10m_50each_20261028_003704/REPORT.md) ·
[全部单轮指标与文件路径](bags/comparison_task05_wall10m_50each_20261028_003704/all_trial_metrics.csv)

任务6后台对比实验：随机障碍map17、8个小块、位置种子1，ours（八机分布式并开启换槽）、Zhou、Quan各20次，共60个bag。
沿用统一指标口径，三项目真实地图已预检一致；仅执行任务6，完成后自动停止。
[实验说明](docs/comparison-task06.md) ·
[当前进度](bags/comparison_task06_random_map17_small08_pos01_20each_20261028_205227/status.json) ·
[独立汇总报告](bags/comparison_task06_random_map17_small08_pos01_20each_20261028_205227/REPORT.md) ·
[各轮指标与文件路径](bags/comparison_task06_random_map17_small08_pos01_20each_20261028_205227/all_trial_metrics.csv)

任务7后台对比实验：随机障碍map17、8个小块、位置种子2，ours（分布式并开启换槽）、Zhou、Quan各20轮，共60轮。
本任务为当前队列首项，先核验地图后开始录制。本任务完成并校验后自动启动下一项。
[独立实验说明](docs/comparison-task07.md) ·
[调度状态](bags/comparison_task07_random_map17_small08_pos02_20each_20261028_223450_schedule.json) ·
[当前轮次](bags/comparison_task07_random_map17_small08_pos02_20each_20261028_223450/status.json) ·
[独立汇总报告](bags/comparison_task07_random_map17_small08_pos02_20each_20261028_223450/REPORT.md) ·
[单轮指标与文件路径](bags/comparison_task07_random_map17_small08_pos02_20each_20261028_223450/all_trial_metrics.csv)

任务8后台对比实验：随机障碍map17、8个小块、位置种子3，ours（分布式并开启换槽）、Zhou、Quan各20轮，共60轮。
任务7全部完成并通过记录完整性检查后自动启动本任务。本任务完成并校验后自动启动下一项。
[独立实验说明](docs/comparison-task08.md) ·
[调度状态](bags/comparison_task08_random_map17_small08_pos03_20each_20261028_223450_schedule.json) ·
[当前轮次](bags/comparison_task08_random_map17_small08_pos03_20each_20261028_223450/status.json) ·
[独立汇总报告](bags/comparison_task08_random_map17_small08_pos03_20each_20261028_223450/REPORT.md) ·
[单轮指标与文件路径](bags/comparison_task08_random_map17_small08_pos03_20each_20261028_223450/all_trial_metrics.csv)

任务9后台对比实验：随机障碍map17、8个小块、位置种子4，ours（分布式并开启换槽）、Zhou、Quan各20轮，共60轮。
任务8全部完成并通过记录完整性检查后自动启动本任务。本任务完成并校验后自动启动下一项。
[独立实验说明](docs/comparison-task09.md) ·
[调度状态](bags/comparison_task09_random_map17_small08_pos04_20each_20261028_223450_schedule.json) ·
[当前轮次](bags/comparison_task09_random_map17_small08_pos04_20each_20261028_223450/status.json) ·
[独立汇总报告](bags/comparison_task09_random_map17_small08_pos04_20each_20261028_223450/REPORT.md) ·
[单轮指标与文件路径](bags/comparison_task09_random_map17_small08_pos04_20each_20261028_223450/all_trial_metrics.csv)

任务10后台对比实验：随机障碍map17、8个小块、位置种子5，ours（分布式并开启换槽）、Zhou、Quan各20轮，共60轮。
任务9全部完成并通过记录完整性检查后自动启动本任务。本任务完成并校验后自动启动下一项。
[独立实验说明](docs/comparison-task10.md) ·
[调度状态](bags/comparison_task10_random_map17_small08_pos05_20each_20261028_223450_schedule.json) ·
[当前轮次](bags/comparison_task10_random_map17_small08_pos05_20each_20261028_223450/status.json) ·
[独立汇总报告](bags/comparison_task10_random_map17_small08_pos05_20each_20261028_223450/REPORT.md) ·
[单轮指标与文件路径](bags/comparison_task10_random_map17_small08_pos05_20each_20261028_223450/all_trial_metrics.csv)

任务11后台对比实验：随机障碍map17、8个小块、位置种子6，ours（分布式并开启换槽）、Zhou、Quan各20轮，共60轮。
任务10全部完成并通过记录完整性检查后自动启动本任务。本任务完成并校验后自动启动下一项。
[独立实验说明](docs/comparison-task11.md) ·
[调度状态](bags/comparison_task11_random_map17_small08_pos06_20each_20261028_223450_schedule.json) ·
[当前轮次](bags/comparison_task11_random_map17_small08_pos06_20each_20261028_223450/status.json) ·
[独立汇总报告](bags/comparison_task11_random_map17_small08_pos06_20each_20261028_223450/REPORT.md) ·
[单轮指标与文件路径](bags/comparison_task11_random_map17_small08_pos06_20each_20261028_223450/all_trial_metrics.csv)

任务12后台对比实验：随机障碍map17、8个小块、位置种子7，ours（分布式并开启换槽）、Zhou、Quan各20轮，共60轮。
任务11全部完成并通过记录完整性检查后自动启动本任务。本任务完成并校验后自动启动下一项。
[独立实验说明](docs/comparison-task12.md) ·
[调度状态](bags/comparison_task12_random_map17_small08_pos07_20each_20261028_223450_schedule.json) ·
[当前轮次](bags/comparison_task12_random_map17_small08_pos07_20each_20261028_223450/status.json) ·
[独立汇总报告](bags/comparison_task12_random_map17_small08_pos07_20each_20261028_223450/REPORT.md) ·
[单轮指标与文件路径](bags/comparison_task12_random_map17_small08_pos07_20each_20261028_223450/all_trial_metrics.csv)

任务13后台对比实验：随机障碍map17、8个小块、位置种子8，ours（分布式并开启换槽）、Zhou、Quan各20轮，共60轮。
任务12全部完成并通过记录完整性检查后自动启动本任务。本任务完成并校验后自动启动下一项。
[独立实验说明](docs/comparison-task13.md) ·
[调度状态](bags/comparison_task13_random_map17_small08_pos08_20each_20261028_223450_schedule.json) ·
[当前轮次](bags/comparison_task13_random_map17_small08_pos08_20each_20261028_223450/status.json) ·
[独立汇总报告](bags/comparison_task13_random_map17_small08_pos08_20each_20261028_223450/REPORT.md) ·
[单轮指标与文件路径](bags/comparison_task13_random_map17_small08_pos08_20each_20261028_223450/all_trial_metrics.csv)

任务14后台对比实验：随机障碍map17、8个小块、位置种子9，ours（分布式并开启换槽）、Zhou、Quan各20轮，共60轮。
任务13全部完成并通过记录完整性检查后自动启动本任务。本任务完成并校验后自动启动下一项。
[独立实验说明](docs/comparison-task14.md) ·
[调度状态](bags/comparison_task14_random_map17_small08_pos09_20each_20261028_223450_schedule.json) ·
[当前轮次](bags/comparison_task14_random_map17_small08_pos09_20each_20261028_223450/status.json) ·
[独立汇总报告](bags/comparison_task14_random_map17_small08_pos09_20each_20261028_223450/REPORT.md) ·
[单轮指标与文件路径](bags/comparison_task14_random_map17_small08_pos09_20each_20261028_223450/all_trial_metrics.csv)

任务15后台对比实验：随机障碍map17、8个小块、位置种子10，ours（分布式并开启换槽）、Zhou、Quan各20轮，共60轮。
任务14全部完成并通过记录完整性检查后自动启动本任务。本任务完成汇总并校验后，整个队列自动停止。
[独立实验说明](docs/comparison-task15.md) ·
[调度状态](bags/comparison_task15_random_map17_small08_pos10_20each_20261028_223450_schedule.json) ·
[当前轮次](bags/comparison_task15_random_map17_small08_pos10_20each_20261028_223450/status.json) ·
[独立汇总报告](bags/comparison_task15_random_map17_small08_pos10_20each_20261028_223450/REPORT.md) ·
[单轮指标与文件路径](bags/comparison_task15_random_map17_small08_pos10_20each_20261028_223450/all_trial_metrics.csv)

任务14–15已设置续录（2026-10-29T14:03:12.960647+08:00）：保留430次有效记录；任务14补50次后顺序运行任务15的60次。
任务14未通过里程计审计的尝试完整归档，门槛不变；两项说明与报告仍独立保存。
全部完成后自动生成[任务6–15完整平均报告](bags/comparison_tasks06_to15_average_final/REPORT.md)，然后停止。
