# 任务13：随机障碍map17 / small08 / pos08，三项目各20轮

来源：`/home/rise-lab/split-merge/对比实验.txt`任务13。本任务共60个bag，独立保存数据、报告和说明。
任务12全部完成并通过记录完整性检查后自动启动本任务。本任务完成并校验后自动启动下一项。

- 场景参数：`random_seed:=17 small_obstacle_count:=8 small_position_seed:=8`。
- ours使用八机分布式并开启跨列换槽；Zhou和Quan使用各自的规划器与独立编译环境。
- 八机2×4、相邻槽位间距1m，起点中心(-14,0,1.5)、终点中心(14,0,1.5)，最大速度0.5m/s，RViz关闭。
- 沿用任务6的算法、感知、评估配置；仅切换小块位置种子。大型障碍、堆叠结构、窄缝和小块尺寸保持同一基准。
- 开始前实际运行三个项目的地图生成器，核对17个障碍盒完全一致；每次飞行重新检查实际地图参数和发布的几何。
- 冻结录包/评估代码并记录规划器、地图生成器、论文与启动/参数文件SHA256。任务开始前重新检查，发现输入变化即停止。

统计定义：

- 首架机相对配置起点移动0.05m记为出发；八机各自距实际目标≤0.30m、速度≤0.15m/s并保持1s记为全员到达。
  完成耗时截至稳定区间起点，不含1s确认时间。ours使用自身实际应用槽位对应的目标。
- 分别报告到达率和无碰撞到达率，及成功耗时、无碰撞成功耗时。
- 统一用0.10m半径球体、100Hz同步位置检查所有实际障碍盒与机间距离，保存最小净空和最小机间距。
  这是采样几何检查，不代表连续时间碰撞证明；规划器诊断另外保留。
- 论文式45按完整八机模板进行Sim(3)最优对齐，八机残差平方和按三维算术质心弧长积分，s0=1，单位m²。
  不除以8、不取平方根；100Hz计算并用50Hz复核。ours使用自身实际应用槽位，Zhou/Quan使用固定物理编号。
- 已出发失败轮次保留至失败截止的误差并标明区间，不混入成功轮次均值。飞行超时240s，未出发90s；有效失败不重飞替换。
- 每轮交替串行执行ours、Zhou、Quan，各次使用独立容器和网络。
- 基础设施、几何或录包审计异常保留记录并停止队列；算法碰撞/超时属于实验结果，保留并计入分母。

LZ4轻量bag保留原频率里程计、姿态/速度、控制指令、规划轨迹/数据、心跳、编队消息、诊断与事件/最终指标。
排除点云、图像、地图栅格和地图MarkerArray。实际障碍盒另存小型JSON，不需保存点云来复核几何碰撞。
原频率位置另存`drone_positions.csv`，包含物理ID、双时间戳、xyz、速度与姿态；每轮保存式45汇总/时间序列、参数和压缩日志。

本次独立目录：`bags/comparison_task13_random_map17_small08_pos08_20each_20261028_223450/`。

- [排队/运行状态](../bags/comparison_task13_random_map17_small08_pos08_20each_20261028_223450_schedule.json)：现在即可查看。
- [当前轮次](../bags/comparison_task13_random_map17_small08_pos08_20each_20261028_223450/status.json)：实际启动后生成。
- [独立汇总报告](../bags/comparison_task13_random_map17_small08_pos08_20each_20261028_223450/REPORT.md)：实际启动后逐轮更新。
- [单轮指标及文件路径](../bags/comparison_task13_random_map17_small08_pos08_20each_20261028_223450/all_trial_metrics.csv)：含bag与位置CSV相对路径。
- `comparison_summary.csv/json`：本任务的三算法统计；不混入其他任务。
- `geometry_preflight.json`及`geometry_preflight_details/`：实际地图预检记录。
- `startup_verification.json`：三算法首轮结束后自动核验bag校验和、八路位置、参数和话题。
- `completion_verification.json`：全部60次结束后逐包校验，通过后队列才会推进。
- `supervisor.json`、`code/`、`PROTOCOL.md`：后台PID、冻结脚本与本说明副本。

bag名称示例：`task13_ours_random_map17_small08_pos08_8uav_run001.bag`。

独立重现入口（已有后台队列时不要重复启动）：

```bash
python3 -u scripts/run_comparison_benchmark.py \
  --task 13 --scene sparse_random --runs 20 \
  --random-seed 17 --small-obstacle-count 8 --small-position-seed 8 \
  --geometry-reference bags/comparison_task13_random_map17_small08_pos08_20each_20261028_223450_geometry_preflight/geometry_reference.json \
  --output bags/comparison_task13_random_map17_small08_pos08_20each_新时间戳
```

地图预检在该任务开始前由顺序控制器自动执行。首次三算法录包审计也自动执行，评估口径与任务6保持一致。
