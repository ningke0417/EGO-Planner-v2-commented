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
本次几何修订同时涉及 `plan_env`、`ego_planner` 和共享几何头，需完整执行
`catkin_make -j2 -l2`；只替换 agent 可执行文件会缺少新的本机占用快照。

## 本次实现

- 每架机独立观测地图，只订阅本机 odom/点云；不接收中央融合地图。
- 每架机发布自身状态，只计算与自己相连的矩形边。
- 复用 StarConvex 几何库，以本机规划器的完整膨胀占用快照和本机已观测空闲体素
  计算空间交叠，逐格检查端点间中心线。不重复膨胀、不额外叠加 0.1 米检测带。
  未知空间不能作为连接或重连证据。
- 断边后记录两端最大距离。只有距离曾超过断开阈值至少 0.2 米、从峰值回落至少
  0.25 米并连续靠近 0.6 秒，两端又各自在 3 米内确认已知空闲连线，
  才允许持续 0.3 秒后以 0.22 权重建立弱连接。这样向墙两侧继续分流时不会在
  1.8–3.0 米之间重新接边。进入 1.7 米以内并稳定 0.5 秒后转入正常连接；
  普通近距离重连仍保留 0.8 秒去抖。这两个时间分别由
  `weak_to_normal_hold` 和 `reconnect_hold` 控制，互不影响。
  重连不以 StarConvex 重叠度作为门槛；重叠度恢复后继续用于正常连接的分裂判断。
- `FormationEdgeEvidence` 按端点发布；两端 KEEP 且证据未过期才启用一条边。
  任一端 CUT 或超时，接收节点就移除该边。异步消息传播期间可能短暂不一致，
  这不是全局原子共识，不能据此宣称严格同时切换或形式化无碰撞保证。
- 端点证据带弱连接标记，拓扑命令和应用回执通过 `weak_edge_mask` 标明已接通但尚未
  达到 1.7 米正式连接条件的边；它必须是 `edge_keep_mask` 的子集。
- 默认租约 1.5 秒、普通重连持续 0.8 秒、弱重连持续 0.3 秒、切断保持 1 秒；序列号拒绝重复和乱序，
  进程 session 拒绝已见过的新 session 之前的旧报文。
- 启动时 `drone_id == slot`；物理 ID 始终固定，运行中的槽位占用可按下述协议变更。
- 可选的分布式同列前后换槽使用四个独立列版本号。每列仅交换两个槽位，
  `drone_id` 仍是物理身份，不会重编号；逻辑编队边会按新槽位占用者映射到物理无人机。
- 每机 FSM 只接收本机 agent 的拓扑，并有独立 1.5 秒 watchdog。
  agent 失联后，本机取消全部编队边，普通障碍/机间轨迹避碰逻辑仍然保留。
- 分布式启动采用 fail-closed 屏障：每个 agent 必须先获得全部 10 条边的双端新鲜
  KEEP 证据，再等待 8 个 agent 都广播启动就绪；屏障释放前 FSM 停留在
  `WAIT_TARGET`，不会发布初始轨迹。屏障一旦释放即锁存，不干预飞行中的正常分裂。
- 固定墙和随机场景的分布式局部观测地图当前使用 360 度水平视场；若改为
  180 度，后向相邻边可能因缺少已知空闲证据而保持 CUT，需要重新验证启动屏障。
- 完整图也使用相对边代价，避免切回原来的全体成员编队公式；
  内部兼容消息的 `mode=SPLIT` 表示采用边约束，不代表图一定不连通。
  判断分裂请查看 `components` 和 `edge_keep_mask`。
- 关闭分布式模式中的中央路径意图、全体进度同步、合并同步和顺序启动等待。
- 8 个规划器直接在 `/swarm/trajectories` 互换轨迹，无共享 UDP 中继进程。

本机拓扑话题 `/drone_i_formation_agent/topology` 是本机内部接口；
不同无人机的 epoch 不比较。FSM 应用状态发布在 `/drone_i/formation_applied`。
集中式 `/formation_backend/topology_command` 在此模式没有发布者或控制订阅者。

## 与本机规划器一致的几何判据

每机规划器在分布式点云模式下发布私有控制快照：
`/drone_i_ego_planner_node/grid_map/formation_occupancy`。
它来自规划器实际使用的膨胀占用缓冲区，约 5 Hz，覆盖整个地图、完整高度，
保留最近传感器更新的时间戳。不要用高度截断且时间戳为零的
`grid_map/occupancy_inflate` 可视化点云代替它。

agent 只订阅本机快照，并读取本机规划器初始化参数的栅格分辨率、原点和地图尺寸，
按与 `GridMap::posToIndex` 相同的 `floor((p-origin)*inverse_resolution)` 索引。
占用格已经包含规划器配置的安全距离（场景默认 0.2 米），agent 不再自行膨胀。
地图初始化参数需在启动前设置；改变初始化几何配置时应同时重启规划器与 agent。

本机 `/drone_i/observed_free` 仍保留原来的 0.2 米观测分辨率，仅作为是否观测过空闲的
独立证据，不将其细分为新增观测。占用查询使用规划器栅格；空闲认证使用原观测栅格。
两个网格都必须通过逐格线段遍历，占用始终优先于空闲；未知体素、地图边界外均拒绝。
这种分层避免把粗观测格的障碍再次膨胀成另一套安全边界。

快照失效和观测失效独立受同一 `map_timeout` 约束，取二者较早的传感器时间戳。
重复发布旧快照不会刷新有效期；无快照时启动屏障不会放行，飞行中快照过期则移除相关边。
原有距离、重叠分数、弱恢复靠近门槛和重连计时规则不因这次几何修订而放宽。
旧几何构造接口保留，集中式判据和规划器本身的障碍膨胀计算不变。

## 分布式同列换槽（实验功能）

`rectangle_8uav_fixed_split_merge_wall.launch` 与随机场景接受
`swap_enabled:=true|false`，默认关闭；要试验时显式设置 `swap_enabled:=true`。
这项功能只允许当前占据同一横向列的前、后槽位互换：
`0↔4、1↔5、2↔6、3↔7`。无人机的物理 ID 不变，交换的是编队槽位与最终航点。
四列互不争抢槽位，因此可分别维护版本号，无固定全局分配节点。

仅在启动屏障释放、完整边表曾经建立且随后发生断边后，agent 才评估换槽。
按当前矩形槽位的几何方向（前排槽位 x=+0.5、后排 x=-0.5），后槽位占用者
超过前槽位占用者时，计算交换对两机平方位置误差的收益。
默认收益至少 `0.15 m²`、候选持续 `0.5 s`，上次交换后冷却 `8 s`。
新槽位所涉及的边必须在其端点本机地图中具有已知自由走廊，且两端距离不超过
`3 m`。候选只使用现有的新鲜状态；额外通信仅在提出交换时进行。

较小物理 ID 的一端发出 `PREPARE`；8 个 agent 都验证并回 `ACK` 后才发布
`COMMIT`。未得到全员确认时保持原分配。提交消息在成员状态和规划器应用状态均
确认新列版本之前持续传播；已提交的版本链保留在进程内，发现重启成员落后时
按版本顺序重发。每个 agent 将槽位排列、列版本和边掩码作为同一条本机
拓扑快照交给规划器；受影响边的旧证据被清空，必须由新物理端点重新提供双端 KEEP。
其他列的边证据继续有效。接收节点会拒绝旧版本或错误物理端点的边证据。

这只解决两机互换造成的局部逆序，不解决三机轮换或全局最优分配。通信分区期间，
已经看到 COMMIT 的节点和尚未看到的节点可能暂时使用不同槽位；边证据按列版本
关闭受影响的约束，机间避碰仍由规划器负责。实机跨网部署仍需传输新
`FormationSlotSwap` 消息并验证时钟与丢包恢复。
默认仍不启用。已在一个随机地图种子上观察到实际提交，但仍需统计更多地图的
触发率、轨迹平滑性和不同通信故障场景。

## 分布式连接线显示

分布式模式自动启动只读节点 `distributed_formation_visualizer`，默认 RViz 配置中
启用 `distributed_applied_topology`。它读取每架机的
`/drone_i/formation_applied` 与里程计，仅发布
`/distributed_formation_visualizer/markers`，不发布控制命令，关闭不影响规划。

- 青色粗实线 / ON：该边的两个端点都实际启用正常连接。
- 紫色实线 / WEAK：两端都应用了弱连接，编队约束已经启用，仍在向正式连接距离靠近。
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
catkin_make -j2 -l2 run_tests_ego_planner_gtest_test_tristate_voxel_field
catkin_test_results build/test_results/ego_planner
```

协议测试覆盖双端授权、超时、重复/乱序、进程重启、非法边、过期/未来时间戳、
不同交付顺序的一致结果，以及单成员失联只切断相关边。
几何测试另覆盖规划器取整边界、不重复膨胀、占用覆盖粗空闲证据、未知格断开、
细小斜交障碍及高于 1.9 米的障碍。

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

### 2026-10-13 随机场景换槽验证记录

- 在 `rectangle_8uav_sparse_random.launch` 的 `random_seed:=17`、
  `coordination_mode:=distributed`、`swap_enabled:=true` 下，实际提交了第 1 列
  `1↔5` 换槽，提交收益为 `1.496 m²`。
- 8 个 agent 均记录第 1 列版本 `1`；随后 8 个规划器的应用状态均为
  `drone_to_slot=[0,5,2,3,4,1,6,7]`、`assignment_versions=[0,1,0,0]`、
  `edge_keep_mask=1023`，最终对应位置为 1 号机后排、5 号机前排。
- 这是单次种子的状态与日志核验，不是多种子完成率、轨迹平滑性或丢包恢复证明。
