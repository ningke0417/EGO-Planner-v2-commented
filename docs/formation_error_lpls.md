# 按论文计算编队误差

论文为用户提供的 `Robust_and_Efficient_Trajectory_Planning_for_LPLS-Optimal.pdf`，
实际标题为 *Robust and Efficient Trajectory Planning for Formation Flight in Dense Environments*。
采用 PDF 第 12 页高亮的公式（45），并同时提供公式（46）的相似度误差。

正式批次 `recordings/fixed_wall_8m_20runs_isolated/` 的全部 20 个 bag，结果如下。

| 指标 | 20 次等权平均 | 跨任务样本标准差 |
|---|---:|---:|
| 公式（45），e_dist | 2.263622085 | 0.381059205 |
| 公式（46），e_sim | 0.198816816 | 0.049466785 |

主结果采用质心路程权重、相对原始模板尺度 s0=1。第 2 次撞墙、第 4 次机间碰撞均纳入。
仅取 18 次通过碰撞检查的任务时，两项均值分别为 2.225669525、0.195998309。

## 计算口径

1. 从各 bag 的八路里程计直接读取 header 时间戳及三维位置，共 804,587 条消息。
   原始 bag SHA-256 与录制时的校验清单逐一匹配。
2. 按时间戳线性插值到共同的 100 Hz 时间轴，起止点包含精确任务边界，不外推。
3. 任务区间沿用原录制定义：首次实际离开起点 0.05 m，至全部到达区间的起点。
4. 目标编队直接使用各次 metadata 的原始 2×4、1 m 间隔槽位；固定物理编号 0–7。
5. 公式（45）在每个采样时刻最小化八机位置残差的**平方和**，允许旋转、平移和正尺度，
   采用 SVD 闭式解，旋转限制为 SO(3)，不允许镜像。不除以八，不开平方。
6. 用每个相邻采样段的路程增量对误差做梯形积分，除以 s0 与总路程。
   先算每次任务，再对 20 次取等权算术平均。
7. 式（46）使用完整图，边权为机间距离的平方，按式（1）—（5）构造对称归一化
   拉普拉斯矩阵，然后对矩阵差的 Frobenius 范数平方做相同的路程平均。

论文没有说明编队轨迹 L 具体使用哪条轨迹。本计算明确以三维质心轨迹为主口径。
为避免该约定被误认为论文唯一指定，另存两种权重的结果：

| 路程权重 | e_dist 的 20 次均值 | e_sim 的 20 次均值 |
|---|---:|---:|
| 质心（主结果） | 2.263622085 | 0.198816816 |
| 0 号机 | 2.312426371 | 0.202118439 |
| 八机平均路程增量 | 2.673402369 | 0.239249464 |

s0=1 是相对已记录原始槽位模板的尺度：初始实际编队恰等于该模板加平移。
以上是论文归一化定义下的指标值，e_dist 不应当作单机 RMSE（米）解读。
比较其他算法时，应固定相同模板、s0、编号对应关系、路程口径和任务边界。

## 文件

结果目录为 `recordings/fixed_wall_8m_20runs_isolated/formation_error_lpls/`：

- `REPORT.md`：完整公式、全部逐次结果、其他口径及图。
- `per_trial.csv`、`summary.json`：可供后续统计的结果。
- `formation_error.png`、`formation_error.pdf`：逐次误差柱状图。
- `timeseries/trial_NNN.csv`：同步时间、质心位置/路程、最优缩放及瞬时误差。
- `bag_odometry/trial_NNN.npz`：各 bag 八机原始时间戳和 XYZ。
- `extraction.json`、`validation.json`：提取来源、校验和及独立数值复核。

## 重算

已完成提取的本次数据，在仓库根目录使用本机 NumPy、Matplotlib 环境执行：

```bash
python3 tools/analyze_formation_error.py \
  --batch recordings/fixed_wall_8m_20runs_isolated \
  --output recordings/fixed_wall_8m_20runs_isolated/formation_error_lpls \
  --hz 100 --initial-scale 1
```

这会重写派生分析文件，原 bag 和原录制结果不作修改。处理新批次时，先在 ROS Noetic
环境中执行同一脚本的 `--extract-only` 模式，再在本机运行上面的分析命令。
提取模式要求输出目录中尚未有 `bag_odometry/`，以免覆盖提取缓存。

## 验证

```bash
python3 -m unittest discover -s tools -p 'test_analyze_formation_error.py' -v
```

7 项测试覆盖 Sim(3) 不变性、局部变形、固定编号、镜像限制、批量对齐、
路程积分、同步插值/禁止外推和退化编队。另在 197 个实际编队快照上用独立 SciPy
对齐和逐项构造的拉普拉斯矩阵复核，平方误差最大差异为 1.78e-15。
50 Hz / 100 Hz 的逐次结果最大相对差异小于 0.04%。
