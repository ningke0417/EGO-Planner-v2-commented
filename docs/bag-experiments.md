# 固定墙与随机障碍物批量实验

在 ROS Noetic 环境中运行（本机使用已有 Docker 容器 `split-merge`）：

```bash
docker exec --user 1000:1000 split-merge bash -lc '
  source /opt/ros/noetic/setup.bash
  cd /home/rise-lab/split-merge/EGO-Planner-v2-commented
  source swarm-playground/formation_ws/devel/setup.bash
  python3 scripts/record_fixed_wall_batch.py --runs 20 --wall-length 8 --swap-enabled
'
```

在直接安装 ROS1 的电脑上，source 上面两个 setup 文件后执行 Python 命令即可。
脚本固定使用八机分布式、`swap_scope=all`；`--swap-enabled` 开启跨列换槽。
不指定换槽开关时沿用对应 launch 的默认值；`--no-swap` 可显式关闭。
固定墙 launch 的第一堵墙长度默认已改为 8 m，脚本还会显式传参并检查实际值。
墙长指 y 方向，墙厚 0.5 m、高度范围 -0.1 至 3 m。第二堵墙保持原配置，
x=6.5 m、总长 10 m、中间开口 1.8 m。关闭 RViz，规划器、传感器和地图仍正常运行。

默认分布式随机障碍物场景，重复录制 10 轮：

```bash
docker exec --user 1000:1000 split-merge bash -lc '
  source /opt/ros/noetic/setup.bash
  cd /home/rise-lab/split-merge/EGO-Planner-v2-commented
  source swarm-playground/formation_ws/devel/setup.bash
  python3 scripts/record_fixed_wall_batch.py --scene sparse_random --runs 10 --port 11421
'
```

该场景使用 `rectangle_8uav_sparse_random.launch` 默认参数：随机种子 17、
换槽开启、跨列范围 all，起点中心 x=-14 m、终点中心 x=14 m。
每轮复用同一个默认种子，研究同一默认地图下的重复表现；不会自动切换 10 个地图。
`--random-seed` 可显式指定另一种子。碰撞检查使用本轮实际随机障碍盒，
不会套用固定墙位置。障碍盒从静态几何消息提取，另存小型 JSON；几何消息不录入 bag。

第二场景（只改小块：8 个、位置种子 1），录制 10 轮：

```bash
python3 scripts/record_fixed_wall_batch.py \
  --scene sparse_random --runs 10 --random-seed 17 \
  --small-obstacle-count 8 --small-position-seed 1 \
  --experiment-label scene02_distributed_map17_small08_pos01 --port 11422 \
  --output bags/scene02_random_small08_posseed01_mapseed17_distributed_20261025
```

同样需要已 source 的 Noetic 和 formation_ws 环境。大型、堆叠和窄缝障碍不变。
`--experiment-label` 用作每个 bag 的名称前缀：
`trial_01/scene02_distributed_map17_small08_pos01_run01.bag`；
同轮的位置表为 `drone_positions.csv`，运行指标为 `result.json`。
批次 `summary.csv` 包含 `bag_path` 和 `positions_path`，可直接定位每轮文件。
未指定标签时仍使用原先的 `flight.bag` / `positions.csv`，旧数据可继续分析。

第三场景保持 8 个小块，仅把位置种子改成 2：

```bash
python3 scripts/record_fixed_wall_batch.py \
  --scene sparse_random --runs 10 --random-seed 17 \
  --small-obstacle-count 8 --small-position-seed 2 \
  --experiment-label scene03_distributed_map17_small08_pos02 --port 11423
```

本次数据目录为 `bags/scene03_random_small08_posseed02_mapseed17_distributed_20261025/`。
各 bag 按 `scene03_distributed_map17_small08_pos02_run01.bag` 至 `run10.bag` 命名。
10 轮结果及第 4 轮收尾异常的核验恢复说明见 [第三场景报告](../bags/scene03_random_small08_posseed02_mapseed17_distributed_20261025/scene03_experiment_report.md)。

第四场景保持 8 个小块，将位置种子改成 3，录制 10 轮：

```bash
python3 scripts/record_fixed_wall_batch.py \
  --scene sparse_random --runs 10 --random-seed 17 \
  --small-obstacle-count 8 --small-position-seed 3 --swap-enabled \
  --experiment-label scene04_distributed_map17_small08_pos03 --port 0
```

本次目录为 `bags/scene04_random_small08_posseed03_mapseed17_distributed_20261025/`，
各包命名为 `scene04_distributed_map17_small08_pos03_run01.bag` 至 `run10.bag`。
到达、采样几何检查、运行时间与式（45）的统计口径保持一致。
10 轮结果见 [第四场景报告](../bags/scene04_random_small08_posseed03_mapseed17_distributed_20261025/scene04_experiment_report.md)。
每轮指标表另列 `planner_safety_event_messages`；安全诊断原文保存在批次
`planner_safety_diagnostics.json`，与实际障碍盒/机间距离的采样几何检查分开统计。

每轮创建独立 ROS master（默认端口 11420），先启动录包与指标订阅，再启动场景。
不会使用或结束其他 master 下的节点，也不复用上一轮换槽事务。
输出目录必须尚不存在；中断后保留已完成数据，不会覆盖旧结果。
续录同一批次时，传入原有场景参数、`--output 原目录 --resume`。
脚本核对实验配置并跳过已完成的轮次；有未恢复的基础设施错误时停止，避免覆盖原始数据。
`--port 0` 可为每轮选用独立空闲端口，实际端口保存在该轮 `ros_master.json`。
续录配置和脚本另存为 `resume_config.json`、`resume_recorder_source.py` 及
`resume_analysis_source.py`，原快照继续保留。指标进程提交最终事件、刷新位置表并写出
`result.json` 后才收尾，离线分析另有完成标记；全部轮次完成后写出 `batch_complete.json`。

## 记录内容

bag 使用 LZ4 压缩。白名单包含八机原始频率的 odometry、position command、
多项式轨迹、规划耗时指标、心跳、分布式状态/边证据/分配协商、规划器实际应用的
拓扑及槽位目标、安全状态、ROS 日志以及 `/experiment/events`、`/experiment/metrics`、
`/experiment/formation_error`。
白名单保存在 config.json，可按后续需求扩充。未发布的话题不会凭空生成数据。

不录点云、图像、占用栅格、地图快照、可视化路径和 MarkerArray。
随机场景的实际障碍盒另存为 obstacle_geometry.json，完整参数另存为 params.yaml，
但这份轻量 bag 无法独立重建当时的全部感知/局部地图。

每轮目录包含：

- `flight.bag`：轻量化原始数据。
- `positions.csv`：20 Hz 采样，物理 ID、位置、速度、槽位、实际目标和 odom 原始时间戳。
  需要姿态、角速度或原始采样频率时，从 bag 中读取完整 Odometry。
- `result.json`：成功、失败原因、运行时间、路径长度、最大速度、最小机间距离、
  最小障碍距离、换槽次数和终点位置，以及离线编队误差 formation_error_eq45。
- `topics.json`：实际录到的话题、消息类型、消息数量。
- `params.yaml`、`command.json`、各进程日志、`ros_logs/`：复现实验与定位异常。
- `progress.json`：本轮实时状态，每秒更新。

批次目录包含 `config.json`、`source.diff`、录制脚本副本、`summary.csv` 和 `summary.json`。
未提交的已有源码修改也保存于 diff；源码修改需要用户自行编译后才反映在可执行文件中。
本脚本不会改变优化器或仿真器。

## 统计口径

- 出发：任意一架机相对其首个 odom 位置移动至少 0.05 m，记该 odom 时间为起点。
  因此计时略晚于第一点非零运动，空间偏差至多约 5 cm 加一个 odom 采样间隔。
- 到达：八架机同时处于 `WAIT_TARGET`，距各自实际应用目标不超过 0.30 m，
  速度不超过 0.15 m/s，规划器报告当前位置地图/墙保护安全，持续 1 秒。
  记录稳定区间起点作为所有无人机到达时刻，1 秒确认时间不加入运行时间。
  odom 必须在 0.5 秒内更新，规划器回执及状态必须在 1.5 秒内更新。
  已到达机的 FSM 会清除 goal_valid，因此保留其最后有效目标，并要求回执持续更新。
- 换槽后跟随每架物理无人机收到的实际目标；改变目标会重置到达稳定判定。
- 成功：满足全员稳定到达，且本轮采样未发现物理障碍内的机体中心或机间距离小于
  0.20 m。机间 0.20 m 是可配置的碰撞代理阈值（`--pair-collision-distance`），
  不是由仿真器提供的接触检测。采样碰撞/净空在 20 Hz 检查，不能证明连续时间无碰撞。
  墙距小于 0.20 m 另记为安全裕度不足，不直接等同于接触墙体。
- 运行时间：仅成功轮次填写 `completion_time_s`；失败留 null，观测时长另存。
- 超时：出发后 240 秒为 `flight_timeout`；90 秒未出发为 `no_departure`。
  丢失任一机 odom 超过 5 秒记 `odometry_lost`。阈值均可通过命令行调整。
- 成功率：有效实验中的成功次数/有效实验次数，同时报告全部尝试口径。
  启动或录包错误与中断单列，出现基础设施错误立即停止批次，避免继续生成无效 bag。

固定墙的 20 轮和本次默认随机场景的 10 轮，均用于观察固定地图、固定参数下
系统调度和规划随机性带来的重复表现，不是对多个地图种子的泛化成功率评估。
本次随机场景数据及结果见 [10 轮结果报告](../bags/sparse_random_distributed_default_20261025/REPORT.md)。

## 按论文计算编队误差

`scripts/analyze_paper_formation_error.py` 直接读取原始 bag，复现 Quan 等
《Robust and Efficient Trajectory Planning for Formation Flight in Dense Environments》
第 12 页式（45）的 Sim(3) 最优对齐残差平方和路径平均，以及式（46）的图相似误差。
本地计算以三维质心轨迹定义编队路径，初始尺度因子 s0=1，包含分裂绕障阶段，
先逐轮平均，再对全部轮次等权平均。目标对应关系以每架物理无人机自身规划器的
formation_applied 回执为准，按回执时间应用新槽位，支持换槽异步传播；
不根据某一架机的完整分配数组推断其他机已应用新槽位，也不重新搜索最优编号排列。
分裂阶段始终评价完整八机目标矩形，式（46）使用完全图，不删去已断开的控制边。

每轮自动处理原始 bag：将异步 Odometry 插值到共同 100 Hz 时间轴，以
50 Hz 重算检查离散精度。result.json 和批次 summary.csv 中的
formation_error_eq45 为该离线结果；bag 中 `/experiment/formation_error`
是约 20 Hz 的在线估计，保留瞬时平方残差与累计弧长平均。
原始数据仍保留，可随时重新计算其他指标。

成功轮次按出发至全员到达计算；失败且已出发的轮次按出发至终止计算，并在
interval 字段标为 departure_to_failure_stop。成功和失败的编队误差均保存，
批次成功轮次均值在 mean_formation_error_eq45_successful_m2 中单列；
未出发轮次没有飞行误差。失败时 completion_time_s 保持 null。

在已 source Noetic 和 formation_ws 的环境中执行：

```bash
python3 scripts/analyze_paper_formation_error.py bags/fixed_wall_8m_distributed_swap_20261024_run
# 随机场景可包含失败飞行：
python3 scripts/analyze_paper_formation_error.py bags/sparse_random_distributed_default_20261025 --include-failures
```

结果保存在该批次的 `formation_error_paper/` 下；各轮 CSV、瞬时曲线和计算口径见
[本批次编队误差报告](../bags/fixed_wall_8m_distributed_swap_20261024_run/formation_error_paper/REPORT.md)。

验证指标逻辑（不依赖 ROS）：

```bash
python3 -m unittest discover -s tests -v
```

随机小块位置场景录制完成后，可在 Noetic 环境中运行
`scripts/validate_experiment_bags.py 批次目录`，独立检查原始bag并提取安全诊断。
校验通过后，在仓库根目录生成带场景编号的报告、逐轮指标和文件索引，例如：

```bash
python3 scripts/package_random_scene_report.py \
  bags/scene04_random_small08_posseed03_mapseed17_distributed_20261025 \
  --reference-batch bags/scene03_random_small08_posseed02_mapseed17_distributed_20261025 \
  --scene-number 4
```

该命令还会核对上一场景的固定障碍、小块尺寸、当前实际参数和录制前归档的二进制哈希。
