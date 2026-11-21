# 默认随机障碍物：轻量录包与论文式（45）

入口为 `tools/record_random_benchmark.py`，复用 `record_wall_benchmark.py` 的录包和
任务监测逻辑，默认运行官方 `rectangle_8uav_sparse_random.launch`，录制 10 次。

## 场景

- 官方分布式八机规划器；每次重新启动 ROS master、地图、规划器和模拟器。
- 保留默认 `random_seed=17`、`small_position_seed=-1`；10 次重复同一默认布局。
- 障碍物为 4 个大障碍、6 个小障碍、3 个堆叠块和 2 段窄通道，共 15 个实体。
- 编队中心从 (-14, 0, 1.5) 飞往 (14, 0, 1.5)，默认速度参数 0.5 m/s。
- 无界面执行，录包准备完成后统一发送出发触发。任务和算法参数沿用场景默认值。
- 使用独立容器网络，隔离官方通信桥固定使用的 UDP 8081。

## 本机命令

在仓库根目录执行。输出目录必须不存在，避免覆盖已有实验：

```bash
docker run --rm --network none --name ego-baseline-random10-recorder \
  --user 1000:1000 --env ROS_HOME=/tmp/ego-random-ros-home \
  --mount type=bind,src=/home/rise-lab/split-merge/EGO-Planner-v2-official-baseline,dst=/home/rise-lab/split-merge/EGO-Planner-v2-official-baseline \
  --entrypoint bash fishros2/ros:noetic-desktop-full -lc '
  source /opt/ros/noetic/setup.bash
  source /home/rise-lab/split-merge/EGO-Planner-v2-official-baseline/swarm-playground/formation_ws/devel/original_baseline/setup.bash
  exec python3 -u /home/rise-lab/split-merge/EGO-Planner-v2-official-baseline/tools/record_random_benchmark.py \
    --runs 10 --timeout 180 \
    --output /home/rise-lab/split-merge/EGO-Planner-v2-official-baseline/recordings/my_random_default_10runs
'
```

本次数据目录：`recordings/random_default_10runs_20261025/`。
加载 ROS Noetic 和本仓库独立开发环境后，也可直接运行上述 Python 入口。

## Bag 内容与文件

使用话题白名单、LZ4 压缩；里程计和指令不降采样。

- 八机完整里程计（位置、四元数、速度、消息时间戳）和位置指令。
- 八机完整多项式规划轨迹、规划统计、心跳、广播轨迹。
- ROS 汇总日志、TF、出发/停止、场景参数、任务结果事件。
- `/benchmark/formation_error`：结束后直接从该 bag 的里程计计算并追加的论文指标。

不录制全局/局部点云、图像、深度、栅格地图、地图 MarkerArray 或大型轨迹可视化。
障碍物点云照常参与仿真感知；为了核实碰撞，只在 metadata 中保存一次实体盒子的
中心和尺寸数值，不写入地图话题。

每次 `trial_NNN/` 包含 `flight.bag`、完整频率 `odometry.csv`、`result.json`、
`formation_error.json`、`metadata.json`、`parameters.yaml` 和压缩运行日志。
`live_status.json` 每约 5 秒更新一次。批次 `summary.json` / `summary.csv`
每次结束更新，包含成功率、成功任务平均耗时、式（45）的均值及全部单次结果。
模拟器把加速度写入里程计的 `twist.angular`；CSV 的 `angular_x/y/z` 保留此原始字段，
不能将其解释为真实角速度。计算编队误差使用的是 `world` 坐标下的 XYZ。

可通过 `--file-prefix` 设置包含场景/算法/参数的可读前缀，脚本自动追加运行编号。
例如 `--file-prefix scene02_ego_baseline_r17_small8_pos1` 会生成
`scene02_ego_baseline_r17_small8_pos1_run01.bag`、`…_run01_positions.csv`、
`…_run01_metrics.json` 等。此时每次 `files.json` 保存文件名映射；分析工具同时兼容
有前缀的新批次和原先固定文件名的旧批次。具体见 [场景 02 说明](scene02_bag_benchmark.md)。

## 成功率与计时

采用与此前固定墙实验相同的判定阈值：

- 实际出发：触发后，首架飞机距自身初始位置首次达到 0.05 m 的里程计时间。
- 全部到达：八机各距自身目标不超过 0.3 m、速度不超过 0.15 m/s，同时持续 1 s。
- 任务耗时：实际出发至该持续到达区间的起点，不含等待及 1 s 确认时间。
- 单次触发后最多 180 s。未全部到达时耗时为 null，超时仍保留并计入有效任务分母。
- `arrival_success` 只判断全部到达；`success` 还要求没有观察到几何碰撞及非有限里程计。
- 几何碰撞：半径 0.1 m 的球体、20 Hz 最近位置快照，检查机间重叠和与 15 个实体
  盒子的重叠。这是所设半径及采样下的离散检测，不能证明连续轨迹安全。
- 新增通用字段 `sampled_obstacle_collision` 和 `min_obstacle_surface_clearance_m`，
  随机场景不将所有障碍物误称为固定墙；`sampled_pair_collision` 表示机间碰撞。
- 录包/心跳/数据等基础设施异常会保留原因并停止批次，不将它们悄悄作为算法失败补跑。

## 论文式（45）

沿用用户此前提供的论文第 12 页定义与原固定墙计算口径：

\[
E(t)=\min_{s>0,R\in SO(3),\mathbf t}\sum_{i=0}^{7}
\|p_i^d-(sRp_i(t)+\mathbf t)\|^2,\qquad
\bar e_{dist}=\frac{1}{s_0 L}\int_{\mathcal L} E\,dl.
\]

八机位置按各自 header 时间戳线性插值到共同 100 Hz 时间轴，不外推。
固定编号与原始槽位对应，SVD 求每时刻最优正尺度、平移和 SO(3) 旋转，不允许镜像。
不除以 8、不开平方。取相对原始 1 m 间隔槽位模板的 s0=1。

论文未明确编队轨迹使用哪条轨迹，因此主结果明确采用三维质心路程的梯形积分。
另保存 0 号机路程和八机平均路程口径，以及式（46），便于后续统一比较。

完整到达任务使用“实际出发—全部到达”区间。超时或中断但有有效出发/截止数据的
任务使用“实际出发—判定截止”的已观察区间，并标记 `complete_mission=false`。
这种部分航程指标不代表完成全程，应同时看该标记与成功任务子集。未出发则指标为 null。

`formation_error_eq45` 为主口径单次值；bag 中 `/benchmark/formation_error` 和
`formation_error.json` 记录完整权重、区间、路程、样本数和其他口径。
所有任务均值与成功任务均值分别保存。

录包关闭后才进行公式计算、追加指标和完整性审核。任务结束时的 result 事件标记
`audit_pending=true`；最终 `bag_valid` 以 `result.json` / `summary.json` 为准。

## 验证

```bash
python3 -m unittest discover -s tools -p 'test_record_wall_benchmark.py' -v
python3 -m unittest discover -s tools -p 'test_analyze_formation_error.py' -v
```

结束后检查八路里程计、话题白名单、禁用传感器类型、任务事件，以及追加的唯一一条
式（45）消息。所有 raw bag 和位置 CSV 可用于独立重算后续指标。
