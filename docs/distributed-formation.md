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
几何修订涉及 `plan_env`、`ego_planner` 和共享几何头；八机换槽又增加
`FormationAssignment` 并扩展状态、证据、拓扑及回执消息；执行状态修订另增加
`traj_utils/TrajectoryExecution`。因此更新后需完整执行
`catkin_make -j2 -l2`、重新 source 并重启所有仿真节点，不能只替换 agent 可执行文件，
也不能混用旧消息定义的规划器或显示节点。

## 本次实现

- 每架机独立观测地图，只订阅本机 odom/点云；不接收中央融合地图。
- 每架机发布自身状态，只计算与自己相连的矩形边。
- 复用 StarConvex 几何库，以本机规划器的完整膨胀占用快照和本机已观测空闲体素
  计算空间交叠，逐格检查端点间中心线。不重复膨胀、不额外叠加 0.1 米检测带。
  未知空间不能作为连接或重连证据。
- 断边后记录两端最大距离。只有距离曾超过断开阈值至少 0.2 米、从峰值回落至少
  0.25 米并连续靠近 0.6 秒，两端又各自在 3 米内确认已知空闲连线，
  才允许持续 0.3 秒后以 0.22 权重建立弱连接。这样向墙两侧继续分流时不会在
  1.8–3.0 米之间重新接边。进入 1.7 米以内并稳定 0.2 秒后转入正常连接；
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
- 可选换槽的默认范围为 `swap_scope:=all`：八机参与完整槽位排列的重新分配，
  允许跨列及多机轮换；`swap_scope:=column` 保留原四组前后交换。`drone_id` 始终
  是物理身份，不会重编号；逻辑编队边会按新槽位占用者映射到物理无人机。
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
不同无人机的拓扑消息 `epoch` 不比较；完整换槽的 `assignment_epoch` 则是全员共享的
分配版本，两者不要混淆。FSM 应用状态发布在 `/drone_i/formation_applied`。
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

## 轨迹绝对时间与执行状态

轨迹的“已经发布”不再等同于“执行器仍按这条轨迹运动”。每机 `traj_server` 在
`/swarm/execution_state` 发布 `traj_utils/TrajectoryExecution`，包含物理 ID、进程
session、递增序号、源时间戳、轨迹 ID/原始开始时间/时长，以及 `RUNNING`、
`HOLDING`、`FINISHED` 和 `require_replan`。位置、速度、加速度表示执行器当前
**发出的指令状态**，不是实际机体里程计；真实位置仍以本机 odom 为准。

尚未接受轨迹的执行器发布 `WAITING=0` 和 `heartbeat_fresh`，只表达进程及心跳就绪；
其中的零向量没有空间意义，FSM 将其保存在独立 readiness 状态中，不送入邻机位置预测。
单个执行器重启可通过新 session 的就绪握手重新规划；多个执行器同时重启时，默认
过期邻机保护可能使各机保守互等，本轮不承诺此类故障下的持续推进能力。

### 共用一个时钟，不在接收端重新起算

一次重规划在计算入口捕获候选轨迹的绝对开始时间；初始轨迹采样、不同候选优化、
优化内部重启和最终安装均保留这一时间，不再把计算结束时刻当成新的轨迹起点。
同一物理时刻在对方轨迹上的时间为：

```text
对方相对时间 = 本机轨迹开始时间 + 本机相对时间 - 对方轨迹开始时间
```

FSM、安全检查、规划管理器和优化器使用共享预测函数。只有新鲜 `RUNNING` 回执
与缓存轨迹的 ID、开始时间和时长匹配时，才按该轨迹预测运动；尚未开始时取起点，
轨迹结束后取终点并将预测速度、加速度置零，不再用末速度无限外推。
`HOLDING` 和 `FINISHED` 使用回执里的位置保持静止，即使缓存中已有另一条新轨迹，
也不会提前假定它已执行。到站和暂停的飞机仍参加机间避碰，不因旧轨迹过期而删除。

回执同时检查 ID、`world` 坐标系、有限数值、源时间戳、session 和序号，拒绝重复、
乱序、过期和超前报文。新鲜度按当前检查时刻判断，不用未来预测点的时间给回执
“加龄”，也不允许重复接收旧源时间戳来刷新有效期。身份检查依赖受信任 ROS1
发布者，不是加密身份认证。

### 暂停后必须有新轨迹，不能只恢复心跳

规划心跳超时，或 FSM 向私有 `/drone_i_traj_server/hold` 发送暂停请求后，执行器
锁存最后发出的指令位置并持续发布 `HOLDING + require_replan`。心跳恢复仅恢复
通信，**不会继续执行暂停前的旧轨迹**；必须收到创建时间晚于暂停栅栏的新轨迹，
并通过时效、身份、重复/过期及入口位置连续性检查，才允许重新运动。

规划器依据本机执行回执进行暂停握手；恢复重规划使用新鲜 odom，而不是把旧多项式
上已经偏离实际执行的位置当成起点。新候选计算耗时/轨迹年龄超预算、odom 过期、
入口偏差过大或未越过暂停栅栏时，不发布该候选，并恢复此前已安装的本机轨迹数据。
等待新的执行确认期间也不把“发了轨迹”当成“对方已接收”。这些检查不会解除导航
触发、分布式启动屏障或手动急停条件。

执行回执、本机 odom 与邻机 MINCO 使用独立接收队列和单独的轻量线程，每机只保留最新报文；该线程不修改
规划器或轨迹数据。主线程在检查和交接时统一取出并验证，避免重规划挤压接收队列，
恢复后反复处理过时输入；主线程先应用广播轨迹再核对执行回执，避免新回执与排队中的
旧轨迹长期不匹配。优化迭代及不同候选共享一次交接预算，超时会结束后续重试，
而不是全部算完再仅仅拒绝结果；单次代价函数、地图或搜索调用仍可能越过预算，
这不是硬实时抢占。规划心跳仍由主线程发送，不会因独立接收线程还活着就掩盖规划卡死。
安全检查回调不再直接执行优化：非紧急风险仅请求 FSM 重规划，临近碰撞先发保持，
避免安全回调与 FSM 在同一线程重复优化并抢占心跳。

当前主要启动参数如下；均为节点私有参数，不能只改变一个消费者的超时值：

| 参数 | 默认值 | 用途 |
|---|---|---|
| `execution_feedback_timeout` | 0.5 秒 | FSM、管理器、优化器共同使用的执行回执有效期 |
| `peer_trajectory_timeout` | 1.5 秒 | 从未收到执行回执时，兼容轨迹接收证据的有效期 |
| `fsm/odometry_timeout` | 0.25 秒 | 本机里程计最大年龄 |
| `fsm/planning_handoff_budget` | 0.25 秒 | 候选计算及交接的时间预算 |
| `fsm/planning_entry_gap` | 0.3 米 | 候选交接位置相对新鲜里程计的最大偏差 |
| 执行器 `heartbeat_timeout` | 0.5 秒 | 独立执行器检测规划心跳失效 |

`require_execution_feedback` 默认 **true**。已知邻机回执缺失或过期时，FSM 默认
向本机执行器请求暂停，而不是继续假定邻机在沿旧轨迹运动。预测函数仍冻结该邻机
最后报告的位置并标为不确定；从未收到回执的兼容路径只在轨迹接收证据新鲜时使用
正确的绝对时间，证据过期后冻结到最后接收时刻对应的位置或端点。这只是避免
编造持续运动的保守表示，**不是邻机真实位置或不确定性半径的保证**，也不意味着
默认允许无执行反馈继续飞行。

执行器在独立进程中运行，因此单线程优化阻塞期间仍能检测心跳失效并锁存保持；
但规划/检查本身仍可能被同线程优化阻塞，本轮没有实现独立实时安全控制器。
“发布保持位置、零速度指令”不等于实机瞬时刹停，不能替代独立定位、相对感知、
制动距离约束或底层飞控安全机制。本轮不改变弱连接计时、八机/同列换槽规则、
障碍膨胀、安全距离和 A* 搜索算法。

## 八机完整换槽（实验功能）

两种场景均接受 `swap_enabled:=true|false` 和 `swap_scope:=all|column`。
随机场景的 `swap_enabled` 默认 **true**，固定墙默认 **false**；范围默认均为
**all**。这两个开关只控制分布式换槽，不改变集中式协调路径。

```bash
# 随机环境：显式开启八机跨列换槽（也是当前分布式随机入口的默认配置）。
roslaunch ego_planner rectangle_8uav_sparse_random.launch \
  coordination_mode:=distributed swap_enabled:=true swap_scope:=all

# 固定墙：需显式开启，不能只设置 swap_scope。
roslaunch ego_planner rectangle_8uav_fixed_split_merge_wall.launch \
  coordination_mode:=distributed swap_enabled:=true swap_scope:=all

# 对照：退回原四组前后交换，或完全关闭换槽。
roslaunch ego_planner rectangle_8uav_sparse_random.launch \
  coordination_mode:=distributed swap_enabled:=true swap_scope:=column
roslaunch ego_planner rectangle_8uav_sparse_random.launch \
  coordination_mode:=distributed swap_enabled:=false
```

这些命令应分别运行，停止上一轮后再启动下一轮。启动时可从每机
`[ASSIGNMENT_CONFIG]` 日志检查实际开关与范围。全新独立实验应使用全新 ROS master；
不要在仍有节点运行时删除换槽事务日志。

### 分配目标、触发和代价

八个矩形槽位及其最终航点保持不变，但不再规定某个物理编号必须回到原槽位。
例如原属左侧的飞机绕障后已经位于右侧，可以接手右侧槽位，减少恢复原编号顺序
造成的横穿。重新分配必须是一对一的完整排列：每机一个槽位、每个槽位恰有一机。
它不要求队伍必须分成 4+4，也不会把物理 ID 改成槽位编号。

启动屏障释放、完整边表曾建立且随后出现断边后才激活评估；之后无需等待十条边
全部恢复，即可在末段为恢复队形寻找候选。还要求八机状态和规划器回执新鲜、
共同确认当前分配，且所有成员到各自当前最终目标的距离均不超过 **8 米**。
`reassign_goal_window=8.0` 限制在末段评估；进入
`reassign_lock_distance=0.8` 以内的成员只保留自己的当前槽位，其报价行的其他
槽位均不可用，避免到站前再换目标。这是逐机锁定，不是有一机到站就冻结全队：
其他尚未到站的成员仍可交换剩余槽位。稳定保持门槛只检查本轮真正改变槽位的
成员距离，并在确认时再次拒绝移动已进入锁定范围的本机。这不是识别地图中
“最后一个障碍物”的全局真值规则，也不保证一定在横向归位开始前触发。

每机只使用自己的新鲜局部地图，认证当前位置到八个候选目标的已知空闲直线走廊，
并上报本机的一行代价。未知区域、障碍或过期地图均不能出有效低价。
代价包括剩余直线距离和与当前速度方向不一致的惩罚；每架改变槽位的飞机另加
`reassign_switch_penalty=0.15`。旋转协调者收集八行后，用精确 8×8 最小代价分配
求解完整排列，平局优先保留现分配，而不是让多架飞机分别抢最近目标。

候选必须同时满足总净收益至少 `swap_gain_min=0.15`、相对收益至少
`reassign_relative_gain=0.05`（5%），且同一候选持续
`reassign_stable_hold=0.6` 秒；提交后冷却 `swap_cooldown=8.0` 秒。
以上是距离型代价，不是旧同列规则的平方距离 `m²`。评估周期由
`reassign_period=0.5` 控制。这些是 agent 的启动时私有参数，不支持运行中热切换。

直线认证是保守限制：即使规划器能绕路到达，只要当前直线穿过未知区域或障碍，
该槽位仍不参与此次有效报价；八机（包括已锁定的成员）的当前目标仍须认证为
有限代价基线，当前目标无法认证时，不据此宣称有有限收益。
因此本机制可能推迟或放弃换槽，不是全局绕障路径最优解。实际新轨迹仍由各机
规划器从当前位置、速度重新生成，不能把分配成功当作无碰撞证明。

### 协商、恢复与限制

完整换槽使用 `/swarm/assignment` 的 `FormationAssignment` 消息。初始完整分配
版本为 `assignment_epoch=1`，每轮协调者是物理 ID `assignment_epoch % 8`，
随提交版本轮换；没有新增固定中央分配节点。协议是分布式局部认证报价加轮换
协调求解，不是经典完全无协调者的价格迭代拍卖算法。

`REQUEST/BID` 固定本轮的成员 session、状态序号、位置、速度、槽位目标和代价矩阵；
`PREPARE` 给出完整候选。八机各自复核本机报价及目标认证，预留同一候选后回
`ACK`。只有匹配同一候选的八个确认构成完整证书，协调者才能决定 `COMMIT`。
少一个确认、状态失效或候选不一致时保持旧目标。参与者不能自行因超时清除已投票
的预留；需等待协调者持久记录并传播的 `ABORT` 才能取消并重试，避免旧提交消息与
新提案冲突。协调者不可恢复时允许保守等待，不保证任意故障下任务持续推进。

每机通过 ROS 参数服务器 `/distributed_assignment_journal/drone_i` 保存本机事务和
已提交分配，使同一 master 内的 agent 进程重启可恢复，而不是重新从身份排列投票。
提交决定会重传以修复成员状态和规划器应用状态的落后；该日志**不等于磁盘持久化**，
ROS master 丢失或整组状态丢失后的恢复不在此保证范围内。

提交后将槽位排列、`assignment_epoch` 和完整排列摘要一起送给本机规划器；
更新其槽位、最终航点与编队邻居，清空全部十条边的旧证据，按新物理端点重新授权。
边证据也携带分配版本，旧版本不能给新邻居接边。八机的物理轨迹不会全局原子切换：
通信传播期间目标应用可短暂不同步，机间避碰和急停仍由轨迹规划器负责。
这些保证依赖 ROS1 单 master、受信任成员和有效时钟，不是拜占庭共识或实机安全证明。

### 同列兼容模式

`swap_scope:=column` 保留原四组同列前后槽位交换：`0↔4、1↔5、2↔6、3↔7`，
按当前槽位占用者计算，不是永久绑定这四对物理 ID。四列分别维护版本号，并沿用
原 `FormationSlotSwap` 协议、平方位置误差收益至少 `0.15 m²`、候选持续 `0.5 s`、
冷却 `8 s` 以及新相邻边已知空闲且不超过 `3 m` 的检查。它只能改善同列前后逆序，
不能解决跨列或三机轮换，亦不具备新完整协议的参数服务器事务恢复语义。

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
3. 地图沿用静态环境假设。当前 8 机固定墙和随机场景的分布式观测使用 360 度水平
   感知；若改小视场，后向边可能缺少双方已知空闲证据。未知区域不会被当作空闲
   来强行恢复完整编队或给换槽目标报价。
4. 当前实现了 0.22 弱连接权重，但没有中央版本尚未启用的 0.25/0.30 多级权重爬升、
   前排到后排传播。当前末段八机分配是本节描述的独立局部认证协议，并不等价于中央
   动态换槽流程。正常边仍使用 0.15 权重，不能假设与集中式具有相同轨迹或到达率。
5. 没有将断联无人机从避碰模型中直接删除。实际运动不再符合最后轨迹时，
   仍需要相对感知或有界不确定性/制动策略；不能保证任意网络分区下任务继续且绝对安全。
6. session 使用本机墙钟生成，要求进程重启时墙钟不倒退；长时间暂停后仅凭租约
   不能辨别从未见过的旧进程，实机版本应使用持久启动代号与明确的成员协议。

## 验证

```bash
catkin_make -j2 -l2 run_tests_ego_planner_gtest_test_execution_state
catkin_make -j2 -l2 run_tests_ego_planner_gtest_test_distributed_edges
catkin_make -j2 -l2 run_tests_ego_planner_gtest_test_distributed_assignment
catkin_make -j2 -l2 run_tests_ego_planner_gtest_test_distributed_assignment_protocol
catkin_make -j2 -l2 run_tests_ego_planner_gtest_test_tristate_voxel_field
catkin_test_results build/test_results/ego_planner
```

协议测试覆盖双端授权、超时、重复/乱序、进程重启、非法边、过期/未来时间戳、
不同交付顺序的一致结果，以及单成员失联只切断相关边。
几何测试另覆盖规划器取整边界、不重复膨胀、占用覆盖粗空闲证据、未知格断开、
细小斜交障碍及高于 1.9 米的障碍。

执行状态单元测试 `test_execution_state` 覆盖起始时间不等时的符号、绝对时刻一致、
未来起点钳位、非零末速度不外推、暂停/完成、过期与轨迹不匹配、接收有效期、
非有限数据和 session/序号乱序。另有两个仅用于空独立 master 的运行回归：

- `test/test_traj_execution_runtime.py`：端口 **11341**，脚本自行启动真实执行器，
  用合成轨迹和心跳检查绝对时间、失联保持、旧轨迹拒绝、恢复心跳后不恢复旧运动、
  新轨迹恢复与终点持续保持，不运行飞行仿真。
- `test/test_execution_planner_runtime.py`：端口 **11342**，脚本自行启动一个真实
  规划器，检查非法/外机回执不能绕过启动屏障；显式增加
  `--allow-synthetic-flight` 时检查暂停后从新的合成里程计位置重规划。它不启动
  真实执行器、控制器或飞行器，不能证明物理避碰。

路径相对 `src/planner/plan_manage/`。分别另开终端以相同 `ROS_MASTER_URI` 启动
全新的 `roscore -p 11341` 或 `roscore -p 11342` 后再运行对应脚本；两种测试均不得
接入正在运行的实验 master。构建后需重新 source，以加载新增消息定义。

八机分配单元测试 `test_distributed_assignment` 覆盖完整排列、跨列/多机轮换、
不可用报价、平局保持、切换惩罚、收益门槛、稳定保持和冷却。新增 ROS 回归脚本：

- `test/test_distributed_assignment_planner.py`：专用空 master `11337`，启动一个真实
  规划器，在关闭飞行屏障的条件下验证槽位应用、目标更新和旧/非法版本拒绝。
- `test/test_distributed_assignment_runtime.py`：专用 master `11336`，需先单独启动
  八个真实 agent（正常话题重映射、`swap_enabled=true`、`swap_scope=all`，无真实规划器
  和传感器）。脚本注入合成地图、里程计与规划器回执，验证启动、分裂后的跨列分配、
  全员版本一致及提交证书；增加 `--allow-fault-injection` 会暂时停止发布 7 号里程计，
  验证成员状态过期时不提交、恢复后收敛。不得用于真实实验 master。
- `test_distributed_assignment_protocol_runtime`：手动运行的协议故障回归可执行文件，
  仅接受全新独立 master `11339`。使用真实协议对象和 ROS 消息验证缺少第八个确认时
  保留目标并持久 ABORT、重试后提交、协调者带未决预留重启后的取消恢复，以及多轮
  提交历史和取消黑名单恢复。可在编译、source 后使用以下命令运行；应先在另一个
  终端启动全新的 `roscore -p 11339`，不要复用已有实验 master：

  ```bash
  ROS_MASTER_URI=http://127.0.0.1:11339 \
    rosrun ego_planner test_distributed_assignment_protocol_runtime
  ```

上面两个 Python 脚本的路径相对 `src/planner/plan_manage/`；规划器脚本不验证飞行，
八 agent 脚本和协议可执行文件仅验证合成输入下的协商。三者均不能替代真实八机
轨迹、安全距离、末段横穿减少率及多种子到达率测试。

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
