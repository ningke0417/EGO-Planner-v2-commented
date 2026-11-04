# 任务5：固定墙10m，三项目各50轮

来源：`/home/rise-lab/split-merge/对比实验.txt`。本任务共150个bag。等待任务4全部完成并通过完整性检查后自动开始；本任务完成汇总后整个队列停止。
本任务的说明、数据、报告和统计表独立保存。

- ours：`EGO-Planner-v2-commented`，八机分布式并开启跨列换槽。
- Zhou：`EGO-Planner-v2-official-baseline`，采用其独立编译环境。
- Quan：`Swarm-Formation`，采用其独立编译环境。
- 三项目均为两行四列、邻接间距1m，起点中心(-6,0,1.5)、终点中心(10,0,1.5)，最大速度0.5m/s。
- 第一堵墙x=-2m、厚0.5m、沿y方向长10m，即y∈[-5,5]；第二堵墙和其余场景参数沿用前面的任务。

启动前核验规划器/墙体生成器二进制、录包脚本、评估模块和论文与任务3一致。
顺序控制器同时固定启动文件、参数文件及Docker镜像版本；开始本任务前重新核验，发现变化即停止。
算法配置、成功判定和指标定义沿用[任务1协议](comparison-task01.md)。

- 首架机移动0.05m记为出发；八机各自距目标≤0.30m、速度≤0.15m/s并保持1s记为到达。
  耗时终点为稳定区间起点，不含1s确认时间。
- 到达率与无碰撞到达率分别统计。碰撞检查使用0.10m球体、100Hz同步位置、实际墙盒及机间距离。
  这是采样几何检查；规划器安全诊断另外保存。
- 成功耗时和无碰撞成功耗时分列。论文式45使用完整八机模板，Sim(3)最优对齐后求八机残差平方和，
  沿三维算术质心轨迹弧长积分，s0=1、100Hz计算并用50Hz复核，单位m²；不除以8，也不开平方。
  ours使用自身实际应用的槽位，Zhou、Quan使用固定物理编号；评估器不另外优化编号对应。
- 飞行超时240s、90s未出发保留为有效失败并计入分母；基础设施或录包审计错误保留记录并停止。
- 逐轮交替串行执行ours、Zhou、Quan，每次使用独立容器和独立网络。
- LZ4轻量录包保留原频率位置、姿态、速度、控制指令、轨迹、心跳、编队消息、诊断及最终指标，
  排除点云、图像、地图栅格与地图MarkerArray。原频率位置另存`drone_positions.csv`。

本次目录：`bags/comparison_task05_wall10m_50each_20261028_003704/`。
排队/运行状态：[任务5调度状态](../bags/comparison_task05_wall10m_50each_20261028_003704_schedule.json)。
实际开始后生成：[当前轮次](../bags/comparison_task05_wall10m_50each_20261028_003704/status.json)、[独立报告](../bags/comparison_task05_wall10m_50each_20261028_003704/REPORT.md)、
[单轮指标与文件路径](../bags/comparison_task05_wall10m_50each_20261028_003704/all_trial_metrics.csv)、`comparison_summary.csv/json`。
每次飞行包含`metrics.json`、`drone_positions.csv`、式45结果和时间序列、实际几何、参数、话题清单及压缩日志。
bag名称示例：`task05_ours_wall10m_8uav_run001.bag`；最后一轮为`run050`。

独立重现入口（已有后台队列时不要重复启动）：

```bash
python3 -u scripts/run_comparison_benchmark.py \
  --task 5 --wall-length 10 --runs 50 \
  --output bags/comparison_task05_wall10m_50each_新时间戳
```

监督PID保存在运行目录的`supervisor.json`，录包/评估代码冻结到`code/`；
本说明自动复制为该任务的`PROTOCOL.md`。既有批次恢复需检查异常并保留参数，加`--resume`，不能覆盖有效轮次。
后台容器与进程由顺序控制器管理；若任一任务异常，不会跳过异常启动后续任务。
