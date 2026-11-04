# 任务6：随机障碍map17 / small08 / pos01，三项目各20轮

来源：`/home/rise-lab/split-merge/对比实验.txt`任务六。仅执行任务6，共60个bag，统计完成后自动停止。

- 地图参数：`random_seed:=17 small_obstacle_count:=8 small_position_seed:=1`。
- ours八机分布式，开启跨列换槽；Zhou和Quan使用各自规划器及独立编译环境。
- 八机2×4、相邻槽位间距1m，起点中心(-14,0,1.5)、终点中心(14,0,1.5)，最大速度0.5m/s，RViz关闭。
- 保持该随机场景原有大型障碍、堆叠结构、1.6m中央缝及小块布局；不应用此前分析中的场景改动建议。
- 启动前实际运行三项目地图生成器，17个障碍物轴对齐盒逐一核验一致。每次飞行再检查实际发布几何及全部地图参数。
- 算法二进制与任务5一致；随机地图使用各自原有生成器，记录SHA256。论文及式45计算模块与任务5一致。

统计口径沿用前五批：

- 首架机相对配置起点移动0.05m记为出发；八机各自距实际目标≤0.30m、速度≤0.15m/s，持续1s记为全员到达。
  完成耗时截至稳定区间起点，不包含1s确认时间。ours跟随实际应用槽位对应的目标。
- 到达率和无碰撞到达率分别统计；成功耗时与无碰撞成功耗时分别汇总。
- 统一用半径0.10m球体、100Hz同步位置检查所有实际障碍盒和机间距离，保存最小净空、最小机间距。
  此为采样几何检查，不是连续时间碰撞证明；规划器安全诊断另存。
- 论文式45：完整八机模板做Sim(3)最优对齐，八机残差平方和按三维算术质心轨迹弧长积分，s0=1，单位m²。
  不除以8，不开平方；100Hz计算并用50Hz复核。ours使用自身实际应用槽位，Zhou/Quan固定物理编号。
- 已出发失败轮次保留截止失败时的误差并标注区间，不混入成功轮次均值。飞行超时240s，未出发90s，失败不重飞替换。
- 每轮交替串行执行ours、Zhou、Quan，各次独立容器与网络；基础设施或录包审计错误保留数据并停止。

LZ4轻量bag保留原频率位置、姿态、速度、控制指令、轨迹、规划数据、心跳、编队消息、诊断和实验事件/最终指标。
不记录点云、图像、地图栅格或地图MarkerArray；实际障碍盒另存小型`scene_geometry.json`。
每轮另存原频率`drone_positions.csv`（物理ID、双时间戳、xyz、速度和姿态）、式45汇总/时间序列、参数和压缩日志。

本次独立目录：`bags/comparison_task06_random_map17_small08_pos01_20each_20261028_205227/`。

- [当前进度](../bags/comparison_task06_random_map17_small08_pos01_20each_20261028_205227/status.json)
- [独立汇总报告](../bags/comparison_task06_random_map17_small08_pos01_20each_20261028_205227/REPORT.md)
- [各轮指标与bag/位置路径](../bags/comparison_task06_random_map17_small08_pos01_20each_20261028_205227/all_trial_metrics.csv)
- `comparison_summary.csv/json`：到达、无碰撞到达、耗时及式45的分组统计。
- `geometry_preflight.json`：三项目真实地图预检；`startup_verification.json`：首轮独立核验，首轮完成后生成。
- `supervisor.json`：后台PID；`code/`：冻结录包与评估脚本；`PROTOCOL.md`：本说明副本。

bag名称示例：`task06_ours_random_map17_small08_pos01_8uav_run001.bag`。

单独重现入口（当前已有后台实验，请勿重复启动）：

```bash
python3 -u scripts/run_comparison_benchmark.py \
  --task 6 --scene sparse_random --runs 20 \
  --random-seed 17 --small-obstacle-count 8 --small-position-seed 1 \
  --geometry-reference bags/comparison_task06_random_map17_small08_pos01_20each_20261028_205227_geometry_preflight/geometry_reference.json \
  --output bags/comparison_task06_random_map17_small08_pos01_20each_新时间戳
```

几何预检脚本`prepare_comparison_random_scene.py`需在独立ROS1环境运行，仅启动地图生成器。
当前批次完成后不会自动开始任务7。
