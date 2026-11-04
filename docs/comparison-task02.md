# 任务2：固定墙7m，三项目各50轮

来源：`/home/rise-lab/split-merge/对比实验.txt`。仅执行任务2，共150个bag，完成汇总后自动停止。

- ours：`EGO-Planner-v2-commented`，八机分布式并开启跨列换槽。
- Zhou：`EGO-Planner-v2-official-baseline`，采用其独立编译环境。
- Quan：`Swarm-Formation`，采用其独立编译环境。
- 三项目均为两行四列、邻接间距1m，起点中心(-6,0,1.5)、终点中心(10,0,1.5)，最大速度0.5m/s。
- 第一堵墙x=-2m、厚0.5m、沿y方向长7m，即y∈[-3.5,3.5]；第二堵墙、起终点和其余场景参数不变。

本批规划器/墙体生成器二进制、评估阈值、论文SHA256及Noetic镜像均与任务1核对一致。
前面关于编队误差的分析没有用于改动优化器、权重、拓扑或换槽策略，保证墙长对比口径一致。

统计定义沿用[任务1统一协议](comparison-task01.md)：

- 移动0.05m记为出发；八机各自距目标≤0.30m、速度≤0.15m/s并保持1s记为到达。
  计时终点为稳定区间起点，不含1s确认时间。
- 到达率与无碰撞到达率分别统计。碰撞采用0.10m球体、100Hz同步位置、实际墙盒及机间距离检查，
  不代表连续时间接触证明；规划器安全告警另外保存。
- 成功耗时、无碰撞成功耗时分列。式45按完整八机模板做Sim(3)最优对齐，用质心弧长积分，
  s0=1、100Hz计算并用50Hz复核，单位m²。
- 飞行超时240s、90s未出发均保留为有效失败，不以重飞替换；基础设施异常会保存记录并停止。
- 每轮交替串行执行ours、Zhou、Quan，各次新建独立网络容器，结束后删除。
- LZ4轻量录包保留原频率位置、姿态、速度、控制指令、轨迹、心跳、编队及诊断消息，
  排除点云、图像、占用栅格和地图MarkerArray。

入口命令：

```bash
python3 -u scripts/run_comparison_benchmark.py \
  --task 2 --wall-length 7 --runs 50 \
  --output bags/comparison_task02_wall07m_50each_新时间戳
```

本次后台目录：`bags/comparison_task02_wall07m_50each_20261027_191753/`。
监督PID及命令保存在`supervisor.json`，实际执行的Python代码冻结在`code/`。
`status.json`在运行时记录当前项目和轮次，结束后为`complete`或`stopped_error`。

结果入口：

- `REPORT.md`、`comparison_summary.csv/json`：逐轮自动更新的三项目汇总。
- `all_trial_metrics.csv`：每轮指标、bag及位置表相对路径。
- `ours/run_001/task02_ours_wall07m_8uav_run001.bag`等：名称标明任务、算法、墙长和轮次。
- 各轮`drone_positions.csv`、`metrics.json`、`formation_error_eq45.json`、`formation_error_timeseries.csv`。
- 各轮`scene_geometry.json`、`ros_parameters.yaml`、`scene_metadata.json`和压缩日志：复现与核验。

重新启动已存在的批次需保留相同参数并加`--resume`；已完成且通过审计的轮次不会覆盖或重飞。
