# 8 机分布式编队开发版

分支：`distributed`。集中式实验代码仍可通过同一 launch 的模式参数运行；
`centralized-baseline` 分支保留改造前基线。

## 编译和运行

从仓库根目录：

```bash
source /opt/ros/noetic/setup.bash
cd swarm-playground/formation_ws
catkin_make -j2 -l2
source devel/setup.bash
roslaunch ego_planner rectangle_8uav_fixed_split_merge_wall.launch coordination_mode:=distributed
```

zsh 用户把上面的 `.bash` 换成 `.zsh`。集中式对照：

```bash
roslaunch ego_planner rectangle_8uav_fixed_split_merge_wall.launch coordination_mode:=centralized
```

两个模式请分别启动、结束后再切换。随机地图入口
`rectangle_8uav_sparse_random.launch` 也接受相同参数。未指定模式时保留集中式默认值。
不要只依赖 Git 分支名判断运行模式。

## 本次实现

- 每架机独立观测地图，只订阅本机 odom/点云；不接收中央融合地图。
- 每架机发布自身状态，只计算与自己相连的矩形边。
- 复用 StarConvex 几何库，以本机已观测空闲体素计算两个端点的空间交叠，
  并检查端点间具有厚度的空闲走廊。未知空间不能作为重连证据。
- 与集中式弱恢复一致，断边在 3 米内由两个端点分别持续确认已知空闲厚走廊后，
  持续 0.3 秒即可先以 0.22 权重建立弱连接；进入 1.7 米正式合并距离后转入正常连接。
  普通近距离重连仍保留 0.8 秒去抖，因此缩短的是绕墙后稍远距离的弱恢复窗口。
  重连不以 StarConvex 重叠度作为门槛；重叠度恢复后继续用于正常连接的分裂判断。
- `FormationEdgeEvidence` 按端点发布；两端 KEEP 且证据未过期才启用一条边。
  任一端 CUT 或超时，接收节点就移除该边。异步消息传播期间可能短暂不一致，
  这不是全局原子共识，不能据此宣称严格同时切换或形式化无碰撞保证。
- 默认租约 1.5 秒、普通重连持续 0.8 秒、弱重连持续 0.3 秒、切断保持 1 秒；序列号拒绝重复和乱序，
  进程 session 拒绝已见过的新 session 之前的旧报文。
- 固定 `drone_id == slot`，没有在线槽位重分配；generation 固定为 0。
- 每机 FSM 只接收本机 agent 的拓扑，并有独立 1.5 秒 watchdog。
  agent 失联后，本机取消全部编队边，普通障碍/机间轨迹避碰逻辑仍然保留。
- 分布式启动采用 fail-closed 屏障：每个 agent 必须先获得全部 10 条边的双端新鲜
  KEEP 证据，再等待 8 个 agent 都广播启动就绪；屏障释放前 FSM 停留在
  `WAIT_TARGET`，不会发布初始轨迹。屏障一旦释放即锁存，不干预飞行中的正常分裂。
- 固定墙和随机场景当前按实验要求将点云渲染器与分布式局部观测地图统一设为
  180 度水平视场。后向相邻边可能因端点缺少已知空闲证据而保持 CUT，需通过实测
  判断双端证据和启动屏障是否仍能满足。
- 完整图也使用相对边代价，避免切回原来的全体成员编队公式；
  内部兼容消息的 `mode=SPLIT` 表示采用边约束，不代表图一定不连通。
  判断分裂请查看 `components` 和 `edge_keep_mask`。
- 关闭分布式模式中的中央路径意图、全体进度同步、合并同步和顺序启动等待。
- 8 个规划器直接在 `/swarm/trajectories` 互换轨迹，无共享 UDP 中继进程。

本机拓扑话题 `/drone_i_formation_agent/topology` 是本机内部接口；
不同无人机的 epoch 不比较。FSM 应用状态发布在 `/drone_i/formation_applied`。
集中式 `/formation_backend/topology_command` 在此模式没有发布者或控制订阅者。

## 分布式连接线显示

分布式模式自动启动只读节点 `distributed_formation_visualizer`，默认 RViz 配置中
启用 `distributed_applied_topology`。它读取每架机的
`/drone_i/formation_applied` 与里程计，仅发布
`/distributed_formation_visualizer/markers`，不发布控制命令，关闭不影响规划。

- 青色粗实线 / ON：该边的两个端点都实际启用此边，与集中式相同。
- 黄色虚线 / MISMATCH：两个端点对该边的应用状态不同。
- 红色虚线 / CUT：两个端点都已断开此边，与集中式相同。
- 灰色虚线 / STALE：任一端点拓扑状态缺失/超时（默认 1.5 秒）或里程计超时
  （默认 1 秒）。位置过期时使用最后位置并标注 `last pose`；
  从未收到位置的边不绘制，缺失数显示在图例中。

红色断边可在运行中隐藏/恢复：

```bash
rosparam set /distributed_formation_visualizer/show_cut_edges false
rosparam set /distributed_formation_visualizer/show_cut_edges true
```

默认不显示逐边文字；总览位于编队中心上方，与集中式保持相同字号和位置偏移。
需要调试标签时执行 `rosparam set /distributed_formation_visualizer/show_edge_labels true`。
标签、图例也可在 RViz 此显示项的 Namespaces 中分别关闭。
状态只比较该边两个端点的掩码，不比较不同无人机的 epoch。
标记带有限生命周期，显示节点退出后旧线会自动消失。

已有仿真可在重新编译、source 后单独运行显示节点：

```bash
rosrun ego_planner distributed_formation_visualizer
```

若 RViz 使用的是此前已打开或另存的配置，添加 MarkerArray，话题选择
`/distributed_formation_visualizer/markers`，或重新加载默认配置。

在空的独立 ROS master 上执行显示集成测试（不能与实际实验共用 master）：

```bash
python3 src/planner/plan_manage/test/test_distributed_visualizer.py
```

显示功能已通过编译和独立 ROS 集成测试：四种颜色、仅比较边端点、隐藏/删除、
旧消息重放、非法端点 ID、超时恢复、位置过期和只读话题检查均通过。
无 GUI 的真实八机仿真中已检查到 10 条边标记及有效绿边；停止显示节点后，
8 个规划器继续发布轨迹。RViz 配置已接入，尚未进行 GUI 截图检查。

## 边界与后续工作

这是 ROS1 单 master 仿真中的分布式决策实现，尚不是完全无中心的实机部署：

1. ROS1 发现仍依赖 master；仿真环境真值地图仍集中生成，但编队节点只读取本机观测。
2. 跨机部署需要时钟同步、每机 ROS/桥接配置，以及新状态/证据消息的网络传输。
   本次没有把新消息加入现有 UDP 序列化协议。
3. 地图沿用静态环境假设。当前 8 机固定墙和随机场景均使用 180 度水平感知，后向边
   可能因缺少双方已知空闲证据而长期保持 CUT；未知区域不会被当作空闲来强行恢复
   完整编队。
4. 当前实现了 0.22 弱连接权重，但没有中央版本尚未启用的 0.25/0.30 多级权重爬升、
   前排到后排传播和动态 Hungarian 换槽。正常边仍使用 0.15 权重，不能假设与集中式
   具有相同轨迹或到达率。
5. 没有将断联无人机从避碰模型中直接删除。实际运动不再符合最后轨迹时，
   仍需要相对感知或有界不确定性/制动策略；不能保证任意网络分区下任务继续且绝对安全。
6. session 使用本机墙钟生成，要求进程重启时墙钟不倒退；长时间暂停后仅凭租约
   不能辨别从未见过的旧进程，实机版本应使用持久启动代号与明确的成员协议。

## 验证

```bash
catkin_make -j2 -l2 run_tests_ego_planner_gtest_test_distributed_edges
catkin_test_results build/test_results/ego_planner
```

协议测试覆盖双端授权、超时、重复/乱序、进程重启、非法边、过期/未来时间戳、
不同交付顺序的一致结果，以及单成员失联只切断相关边。

运行分布式仿真后可执行只读 smoke 检查：

```bash
python3 src/planner/plan_manage/test/distributed_smoke.py
```

脚本会验证 8 个 agent 都先发布 `DISTRIBUTED_STARTUP_WAIT`，随后仅在完整边掩码
`1023` 下释放为 `DISTRIBUTED_ENDPOINT_LEASES`，并确认 8 个规划器发布轨迹。

仅在专门的测试仿真中增加 `--allow-fault-injection`：
脚本会停止 `/drone_0_formation_agent`，验证本机 watchdog、其他节点租约过期、
以及 8 个规划器是否仍发布轨迹。该测试不等价于全航程成功或安全性证明。

### 2026-10-06 本地验证记录

- 完整 catkin 编译通过。
- 6 项分布式协议 gtest 通过。
- 固定墙分布式仿真：8 个规划器均发布轨迹，8 个局部地图的订阅隔离检查通过，
  实际观测产生有效边。关闭 0 号 agent 后，FSM 清除编队边，其他节点清除相关边，
  8 个规划器继续发布轨迹。
- 集中式回归：8 个规划器发布轨迹，中央拓扑命令和融合地图正常，
  不存在分布式证据发布者。
- 固定墙和随机地图的两种模式 launch 解析检查通过。
- 测试发现并修复 CPU 点云模拟器缺少时间戳及无邻近障碍时不发布空扫描的问题。
- 尚未验证全航程完成率、完整矩形恢复率、跨机无线网络和实飞。

### 2026-10-08 启动屏障验证记录

- 完整 catkin 编译及 6 项分布式边协议 gtest 通过；集中式、分布式 launch 解析通过。
- 运行时探针确认 8 架机均经历 `STARTUP_WAIT -> 完整掩码释放 -> 发布轨迹 -> 开始移动`，
  未出现拓扑尚未初始化便起飞的全 CUT 窗口。
- 固定墙全航程中 8/8 到达目标并进入 `WAIT_TARGET`，最终各机均报告单一连通分量
  `{0,1,2,3,4,5,6,7}` 和 10/10 条 KEEP 边（`edge_keep_mask=1023`）。
- 本次验证仍是单机 ROS1 仿真，不构成任意通信故障或实机条件下的安全证明。

### 2026-10-09 可见性弱重连验证记录

- 以下两轮结果使用当时的 360 度分布式水平视场，不代表改为 180 度后的运行结果。
- 分布式重连不再要求 StarConvex 重叠度达到合并阈值；3 米内双端可见时进入 0.22
  弱连接，1.7 米内转入正常连接。弱重连持续门限与集中式对齐为 0.3 秒。
- 两次从启动前即接入探针的固定墙全航程重复测试中，完整 10 边分别在机群平均
  `x=1.66`、`x=1.69` 恢复，正式合并分别在 `x=2.97`、`x=2.87` 完成；均早于
  第二堵墙 `x=6.5`。
- 两次测试均为 8/8 到达目标，最终位置约为前排 `x=10.5`、后排 `x=9.5`，未再出现
  到达第二堵墙仍未合并的情况。
- `distributed_formation_agent` 编译通过，8 项分布式边协议 gtest 通过；上述数据是
  单机 ROS1 仿真结果，不代表网络丢包或实机条件下的统计保证。
