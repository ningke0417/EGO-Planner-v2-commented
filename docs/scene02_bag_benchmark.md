# 场景 02：8 个小块、位置种子 1，官方基线录包

本次按用户的实验顺序命名：场景 01 是此前已录制的默认 6 小块 / 位置种子 -1；
场景 02 使用下面指定的参数，运行 10 次。

```bash
roslaunch ego_planner rectangle_8uav_sparse_random.launch \
  random_seed:=17 small_obstacle_count:=8 small_position_seed:=1
```

它对应匹配场景清单中的“小块布局 01”（位置种子 1），包含 9 个固定实体和 8 个小块，
共 17 个障碍物。起飞前会核对实际实体几何 SHA-256 与
`docs/random_scene_suite.json` 一致。实验序号 02 和清单中的小块布局编号含义不同，
场景参数及实际几何校验和是最终依据。

## 本机录制命令

```bash
docker run --rm --network none --name ego-baseline-scene02-recorder \
  --user 1000:1000 --env ROS_HOME=/tmp/ego-scene02-ros-home \
  --mount type=bind,src=/home/rise-lab/split-merge/EGO-Planner-v2-official-baseline,dst=/home/rise-lab/split-merge/EGO-Planner-v2-official-baseline \
  --entrypoint bash fishros2/ros:noetic-desktop-full -lc '
  source /opt/ros/noetic/setup.bash
  source /home/rise-lab/split-merge/EGO-Planner-v2-official-baseline/swarm-playground/formation_ws/devel/original_baseline/setup.bash
  exec python3 -u /home/rise-lab/split-merge/EGO-Planner-v2-official-baseline/tools/record_random_benchmark.py \
    --runs 10 --random-seed 17 --small-obstacle-count 8 --small-position-seed 1 \
    --file-prefix scene02_ego_baseline_r17_small8_pos1 --timeout 180 \
    --output /home/rise-lab/split-merge/EGO-Planner-v2-official-baseline/recordings/scene02_random_r17_small08_pos01_ego_baseline_10runs_20261025
'
```

重录时需要使用新的输出目录；脚本会拒绝覆盖已有批次。

## 文件名称

本次目录：

`recordings/scene02_random_r17_small08_pos01_ego_baseline_10runs_20261025/`

每个 `trial_001/` … `trial_010/` 中，文件名都有场景、算法、参数和运行编号。
例如第 1 次：

| 文件 | 内容 |
|---|---|
| `scene02_ego_baseline_r17_small8_pos1_run01.bag` | LZ4 压缩话题记录 |
| `scene02_ego_baseline_r17_small8_pos1_run01_positions.csv` | 八机原始频率位置、姿态、速度和时间戳 |
| `scene02_ego_baseline_r17_small8_pos1_run01_metrics.json` | 到达、碰撞、耗时、路径、编队误差等结果 |
| `scene02_ego_baseline_r17_small8_pos1_run01_formation_error_eq45.json` | 论文式（45）的数值、计算区间和权重 |
| `scene02_ego_baseline_r17_small8_pos1_run01_scene_config.json` | 实际场景参数、实体几何、目标和阈值 |
| `scene02_ego_baseline_r17_small8_pos1_run01_ros_parameters.yaml` | 实际 ROS 参数 |
| `scene02_ego_baseline_r17_small8_pos1_run01_planner.log.gz` | 压缩规划器控制台输出 |

`files.json` 保存机器读取所需的文件名映射；其中 console 记录运行时的 `.log` 名称，
结束后对应文件压缩为 `.log.gz`。旧批次的文件名与目录保持原样，分析工具兼容两种命名。
批次 `summary.json` 和 `summary.csv` 每次结束更新，最后再输出清晰命名的总表和报告。

## 保留数据与判定

沿用 [随机场景录包说明](random_bag_benchmark.md)：八机完整里程计、位置指令、
规划轨迹、规划统计、心跳、广播轨迹、ROS 日志、TF、任务事件和场景参数均保留。
不记录地图点云、局部点云、图像、深度、栅格或地图 MarkerArray。

实际出发为首次偏离自身起点 0.05 m；全部到达要求八机距各自目标 ≤0.3 m、
速度 ≤0.15 m/s，同时持续 1 s，耗时算至该持续区间的起点。
单次最长 180 s。到达率和通过几何碰撞检查的成功率分别统计；
碰撞检测采用半径 0.1 m 的球体、20 Hz 位置快照，检查实际障碍物与机间重叠。

每次结束后直接从其 bag 的原始里程计计算论文式（45）：按 header 时间戳线性插值
到共同 100 Hz 时间轴，逐帧求最优 Sim(3) 对齐，固定物理编号，取八机残差平方和，
再按质心路程做梯形积分并除以 s0×L，s0=1。不除以 8、不开平方。
计算区间为实际出发至全部到达；未到达时标明只计算至判定截止的部分航程。
派生结果同时保存在 JSON 和 bag 的 `/benchmark/formation_error` 中。

平均耗时只使用通过碰撞检查的成功任务；式（45）同时提供全部任务和成功任务的均值。
失败记录保留，不通过补跑替换。

本批次在 `/benchmark/events` 中另保留 `archive_finalized` 封包事件，使 bag 显示的
结束时间覆盖完整原始尾段；逐条哈希验证了原有消息内容及时间戳不变。
离线指标的实际积分起止时间保存在指标消息内，追加消息的封包时间与积分端点分别记录。
