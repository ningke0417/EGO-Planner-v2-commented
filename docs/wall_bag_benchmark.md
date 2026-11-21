# 固定墙轻量录包

入口为 `tools/record_wall_benchmark.py`。每次实验重新启动独立 ROS master、八机
规划器和模拟器，默认串行录制 20 次。第一堵墙通过 `wall_size_y:=8.0` 设置为
8 m，第二堵墙、起终点、官方规划器及速度参数沿用场景原配置。
脚本不修改规划算法，不需要 RViz，也不需要手动发送出发指令。

## 本机运行

本机已有 ROS Noetic 镜像。建议新建临时容器并使用独立网络，避免其他工程的
官方 UDP 通信桥干扰。仓库以同路径挂载，数据直接写回本机：

```bash
docker run --rm --network none --name ego-baseline-wall8m-recorder \
  --user 1000:1000 --env ROS_HOME=/tmp/ego-baseline-ros-home \
  --mount type=bind,src=/home/rise-lab/split-merge/EGO-Planner-v2-official-baseline,dst=/home/rise-lab/split-merge/EGO-Planner-v2-official-baseline \
  --entrypoint bash fishros2/ros:noetic-desktop-full -lc '
  source /opt/ros/noetic/setup.bash
  source /home/rise-lab/split-merge/EGO-Planner-v2-official-baseline/swarm-playground/formation_ws/devel/original_baseline/setup.bash
  exec python3 -u /home/rise-lab/split-merge/EGO-Planner-v2-official-baseline/tools/record_wall_benchmark.py \
    --runs 20 --wall-length 8 --timeout 180 \
    --output /home/rise-lab/split-merge/EGO-Planner-v2-official-baseline/recordings/my_wall8m_batch
'
```

输出目录必须不存在，以免覆盖已有数据。若端口 11347 被占用，可通过 `--port`
选择另一个空闲端口。脚本不会连接或关闭已有 ROS master，也会预先检查官方桥
固定使用的 UDP 8081 是否被占用。不同批次需要使用各自独立的容器网络；单独修改
ROS master 端口不能隔离 UDP 桥。`--network none` 保留本容器内的回环网络，
八机 ROS 和 UDP 通信照常工作，不与主机或其他容器交换轨迹。

在直接安装 ROS Noetic 的电脑上，加载 ROS 和本仓库独立开发目录后，直接运行
上述 Python 脚本即可，不需要 Docker。

## 记录内容

采用明确的话题白名单和 LZ4 压缩，不降采样原始里程计或指令：

- 八机 `/drone_N_visual_slam/odom`：位置、姿态、速度、消息时间戳。
- 八机 `/drone_N_planning/pos_cmd`：位置、速度、加速度、jerk、yaw 等实际指令。
- 八机 `/drone_N_planning/trajectory`：完整多项式规划轨迹。
- 八机 `/drone_N_planning/data_display`、规划心跳、广播轨迹、`/rosout_agg`。
- 出发触发、强制停止、TF，以及 `/benchmark/metadata` 和 `/benchmark/events`。

未发布的话题不会在 bag 索引中出现。所有全局和局部地图点云、栅格地图、相机图像、
深度图及大型可视化轨迹均不录制。物理墙体的尺寸保存在 metadata 的数值字段中，
完整 ROS 参数也保存在 metadata 中，便于脱离原 ROS master 后读取。
完整地图仍正常提供给仿真传感器，只是不写入 bag。

## 判定与计时

- 每架无人机目标从实际解析的航点和固定编号编队槽位计算：前四机
  x=10.5 m，后四机 x=9.5 m，y=-1.5/-0.5/0.5/1.5 m，z=1.5 m。
- 距目标不超过 **0.3 m** 且速度不超过 **0.15 m/s**，八机同时满足并持续
  **1 s**，记 `arrival_success=true`。短暂经过目标不算到达。
- `departure_stamp_ros_s` 为触发后，首架无人机距自身初始位置首次达到
  **0.05 m** 的里程计时间戳。`completion_time_s` 从该时刻算到上述持续到达
  区间的起点，不包含启动等待、触发到运动的延迟及 1 s 到达确认时间。
  另存 `trigger_to_arrival_s`。若未全部到达，耗时为 null，不将超时当作成功耗时。
- 触发后最多观察 **180 s**。超时包保留，计入成功率分母，不重试替换。
- `success` 在到达条件上，还要求没有观察到几何碰撞或非有限里程计。
  默认以半径 **0.1 m** 的球体近似无人机，20 Hz 检查无人机之间的中心距离，
  以及球体到三段实际墙体 AABB 的距离。保存最小机间距和最小墙面净空。
  这是离散采样和指定半径下的检查，不能证明连续轨迹无碰撞；半径可用
  `--vehicle-radius` 修改。它不是规划器 `safety_distance=0.2` 参数的别名。
- 总表同时给出 `arrival_rate` 和 `success_rate`，避免将到达与几何安全混为一谈。
  平均成功耗时仅使用 `success=true` 的任务。
- 缺失/过期里程计不参与到达判定。规划心跳或里程计长时间丢失、录包失败等
  基础设施异常会停止批次并标明原因，不悄悄补跑，也不计入有效飞行分母。

判定采用 20 Hz 最近位置快照，因此到达时间有约一个检查周期的离散误差；bag 和
CSV 中保留原始 100 Hz 左右里程计，可重新插值、对齐和计算其他阈值下的指标。
默认使用 ROS 墙钟；该模拟器不使用 `/clock` 驱动。

## 输出文件

```text
recordings/<batch>/
  batch.json                  命令参数、git 状态（若可用）、脚本/launch 校验和
  summary.json                 成功率、到达率、平均成功耗时及全部单次结果
  summary.csv                  便于表格处理的单次汇总
  trial_001/ ... trial_020/
    flight.bag                LZ4 压缩的原始话题；含配置和起止事件
    odometry.csv              全部八机里程计，原始消息频率
    metadata.json             墙尺寸、目标、阈值、完整 ROS 参数等
    parameters.yaml           实际 ROS 参数快照
    result.json               单次状态、耗时、碰撞标记、路径长度等
    live_status.json          每约 5 s 更新的进度快照，仅用于运行时查看
    scene.log.gz              压缩后的完整规划器控制台输出
    recorder.log / ros_logs/  录包及 ROS 日志
```

`odometry.csv` 是长表，按 `drone_id` 区分无人机，包含消息时间、bag 接收时间、
距触发/实际出发的相对时间、x/y/z、四元数、vx/vy/vz 和原始 twist.angular 三轴。
出发前的相对时间允许为负。八机消息不是硬件同步采样，计算编队误差时应按消息
时间戳插值至共同时间轴，不能简单按 CSV 行号配对。

该场景的 `poscmd_2_odom` 模拟器将**加速度**写入 `twist.angular`，CSV 的
`angular_x/y/z` 保留该原始字段，不应将它解释为真实角速度。
`path_lengths_m`、`max_speeds_mps`、`max_odom_stamp_gaps_s` 从 bag 的全部原始
里程计计算，窗口为出发触发到任务判定截止。确认后仍录制约 1 s 尾部数据，
尾部数据不加入这些汇总指标。

每次结束都会重新打开 bag，检查八路里程计、配置/事件、话题白名单和消息类型，
确认没有地图/点云/图像类型，再导出 CSV。完整性检查结果为 `bag_valid`。
bag 内的 result 事件在关闭文件前写入，其中的 `bag_valid` 尚未审核；请以关闭后
生成的 `result.json` / `summary.json` 的 `bag_valid` 为准。
`recordings/` 已加入 `.gitignore`，避免把实验数据提交到代码仓库。

## 指标逻辑验证

```bash
python3 -m unittest discover -s tools -p 'test_record_wall_benchmark.py' -v
```

覆盖实际出发计时、持续到达、过期/缺失数据、快速经过目标、碰撞后到达及非有限
数值等情况。正式采集前另有一次完整试录，保存在 `recordings/wall8m_pilot/`。
试录不计入正式 20 次批次。正式数据位于
`recordings/fixed_wall_8m_20runs_isolated/`；早先遇到其他工程 UDP 端口占用的
一次中止记录位于 `recordings/fixed_wall_8m_20runs_20261024/`，其中
`ENVIRONMENT_CONFLICT.json` 记录了排除原因，它不属于正式实验。
