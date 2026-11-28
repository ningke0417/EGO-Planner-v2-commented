# 可复用矩形编队

在 `baseline/official-8uav-scenes` 中，`rectangle_formation.launch` 将矩形编队的
规模与障碍物场景分开配置。编号和初始槽位规则参考
`/home/robot/Codes/EGO-Planner-criterion-validation` 的
`launch_distributed_formation.py`，规划节点使用当前项目的官方 EGO 编队规划器。

## 准备环境

已经编译过 `original_baseline` 时，加载当前工程的环境即可，新入口无需重新编译 C++。

```zsh
cd ~/Codes/EGO-Planner-v2-commented/swarm-playground/formation_ws
source /opt/ros/noetic/setup.zsh
source devel/original_baseline/setup.zsh
rospack find ego_planner
```

最后一条命令应指向当前项目的 `formation_ws/src/planner/plan_manage`。
首次编译命令：

```zsh
catkin_make --build build/original_baseline \
  -DCATKIN_DEVEL_PREFIX="$PWD/devel/original_baseline" -j2 -l2
source devel/original_baseline/setup.zsh
```

Python 启动器使用 Noetic 环境中的 `roslaunch`、`rospy`、`rospkg` 和 PyYAML。
不同规模、不同场景及参考项目应分别运行，每次先关闭上一组 ROS 仿真。

## 固定墙和稀疏随机启动命令

固定墙：

```zsh
# 两行三列：6 机
roslaunch ego_planner rectangle_formation.launch scene:=fixed_wall formation_rows:=2 formation_columns:=3
# 三行四列：12 机
roslaunch ego_planner rectangle_formation.launch scene:=fixed_wall formation_rows:=3 formation_columns:=4
# 四行四列：16 机
roslaunch ego_planner rectangle_formation.launch scene:=fixed_wall formation_rows:=4 formation_columns:=4
```

随机障碍物：

```zsh
# 两行三列：6 机
roslaunch ego_planner rectangle_formation.launch scene:=sparse_random formation_rows:=2 formation_columns:=3
# 三行四列：12 机
roslaunch ego_planner rectangle_formation.launch scene:=sparse_random formation_rows:=3 formation_columns:=4
# 四行四列：16 机
roslaunch ego_planner rectangle_formation.launch scene:=sparse_random formation_rows:=4 formation_columns:=4
```

RViz 默认打开，自动生成与无人机数量相符的模型、轨迹、路径和局部地图显示项。
仅改变上述行列参数即可复用全部实现，也支持 `2 × 4` 八机，允许的总数为 2–16。

## 密集随机方柱

`scene:=cluttered_random` 对应参考项目的 `distributed_cluttered_random.launch`，
使用相同地图生成器源码和地图参数。默认 40×18×3 m 地图、种子 17、60 根方柱，
横截面的 x/y 边长各自在 0.45–1.1 m 内采样，高度覆盖 z=-0.1 至 3 m。
中心起终点为 (-17, 0, 1.5) 和 (17, 0, 1.5)，障碍物在 x 方向两端各留出 5 m 空地；
横向中心范围为 ±8 m、边界净空 0.8 m、柱间最小几何间隔 0.6 m。
不添加大型障碍、堆叠块或窄通道墙。无人机模型、感知和单机规划配置复用现有模板。

```zsh
roslaunch ego_planner rectangle_formation.launch scene:=cluttered_random formation_rows:=4 formation_columns:=4
```

6、8、12 机只需分别改为 `2 / 3`、`2 / 4`、`3 / 4`；地图不会随无人机数量改变。
可覆盖 `random_seed`、`small_position_seed`、`small_obstacle_count`、`small_size_min`、
`small_size_max`、`small_center_y_limit`、`boundary_clearance`、`endpoint_clearance`
和 `obstacle_separation`。`rviz:=false`、`start_planning:=false` 和 `record_bag:=true`
用法与其他场景相同。生成地图和短时轨迹检查不代表完成全航程避碰验收。

## 坐标、编号和共有条件

行沿 x 从前向后排列，列沿 y 从负到正排列。物理编号采用逐行编号：
`drone_id = row * formation_columns + column`，row 和 column 均从 0 开始。
相对编队中心的偏移为：

```text
x = ((formation_rows - 1) / 2 - row) * row_spacing
y = (column - (formation_columns - 1) / 2) * column_spacing
z = 0
```

| 规模 | x 偏移（m） | 每行 y 偏移（m） |
|---|---|---|
| 2 × 3，6 机 | 0.5、-0.5 | -1、0、1 |
| 3 × 4，12 机 | 1、0、-1 | -1.5、-0.5、0.5、1.5 |
| 4 × 4，16 机 | 1.5、0.5、-0.5、-1.5 | -1.5、-0.5、0.5、1.5 |

| 条件 | 固定墙 | 随机障碍物 |
|---|---|---|
| 地图范围 | 24 × 14 × 3 m | 32 × 14 × 3 m |
| 中心起点 | (-6, 0, 1.5) | (-14, 0, 1.5) |
| 中心终点 | (10, 0, 1.5) | (14, 0, 1.5) |
| 相邻间距 | 纵向和横向均 1 m | 相同 |
| 速度 / 安全距离 / 编队权重 | 0.5 m/s / 0.2 m / 200 | 相同 |
| 感知 | 360°、半径 5 m，各机独立局部点云 | 相同 |
| 无人机模型 | `odom_visualization/meshes/fake_drone.dae`，比例 0.35 | 相同 |

固定墙与参考项目的 `distributed_fixed_wall.launch` 默认参数一致：第一堵墙
中心 x=-2、宽 6 m、厚 0.5 m；第二堵墙 x=6.5、总宽 10 m、中央开口 1.8 m。
原来的 `rectangle_8uav_fixed_split_merge_wall.launch` 继续默认第一堵墙宽 4 m。
新入口也可加 `wall_size_y:=4.0` 对齐原八机场景，或 `wall_size_y:=8.0` 调整宽度。

随机场景与参考项目的 `distributed_sparse_random.launch` 默认地图一致：种子 17，
4 个大型障碍、6 个小障碍、3 个堆叠块和两段窄通道墙。
同一地图参数下，6、12、16 机使用相同障碍物点云；数量不会改变地图生成参数。
例如，为三种规模都追加 `random_seed:=17 small_obstacle_count:=8 small_position_seed:=1`，
即可使用同一组 8 小障碍位置。`small_position_seed` 也可改为之前使用的 1–10。

## 可调参数

纵横间距、高度、速度及所有已有墙体和随机地图参数均可在入口处覆盖，例如：

```zsh
roslaunch ego_planner rectangle_formation.launch \
  scene:=fixed_wall formation_rows:=3 formation_columns:=4 \
  row_spacing:=0.8 column_spacing:=1.2 altitude:=1.5 wall_size_y:=8.0
```

先查看静态初始编队可追加 `start_planning:=false`，随后在另一已加载相同环境的终端触发：

```zsh
rostopic pub -1 /traj_start_trigger geometry_msgs/PoseStamped '{}'
```

无界面运行追加 `rviz:=false`。轻量录包追加
`record_bag:=true bag_prefix:=/tmp/rectangle_16_wall_trial01`，自动录制全部活动编号的
里程计、轨迹和位置指令，以及地图、几何和汇总日志。

## 文件复用方式

- `rectangle_formation.launch`：所有规模、三类场景的统一入口。
- `scripts/launch_baseline_formation.py`：验证参数、生成居中槽位、按数量组合单机节点，
  并扩展现有 `baseline_8uav.rviz`；生成文件仅存于本次运行的临时目录，退出后清理。
- `baseline_fixed_wall_map.launch`、`baseline_sparse_random_map.launch`：抽出的共有地图配置，
  原八机入口和新入口都引用它们；地图生成器源码与参考项目相同。
- `baseline_8uav_member.launch`、`baseline_8uav_advanced.xml`、`baseline_8uav_simulator.xml`：
  继续作为共有单机模板，新增可选编队 YAML 和高度传递，默认仍支持原八机命令。

RViz 沿用现有白底、`world` 坐标系、障碍物颜色、俯视角度和单机显示分组。
模型均由相同的里程计可视化节点发布；超过八机时自动增加编号显示，修正模板中
个别旧的交叉编号话题，轨迹与局部地图颜色沿用参考项目的十色循环。
相机中心和距离按航程设置，以覆盖整个场景。
参考项目的动态拓扑及换槽专用显示对应其研究算法，不作为本官方规划器的控制节点引入。

## 检查与短时仿真

在仓库根目录、已加载上述 Noetic 环境的终端执行：

```zsh
python3 -m unittest discover -s tools -p test_baseline_formation.py -v
```

检查六组实际解析的 ROS 参数和节点数、编号与槽位、共有地图、模型比例、RViz 话题，
以及自定义间距、高度、里程计后缀和全机录包参数。
同时检查密集方柱的十六机配置，并在参考工作目录可用时对比两项目的地图参数、
生成器源码和四种规模的槽位，覆盖默认设置及自定义方柱尺寸、数量和种子。

每组短时仿真使用独立 master，并在结束后关闭本次启动的进程。例如：

```zsh
python3 tools/smoke_original_8uav.py wall --rows 4 --columns 4 \
  --port 11337 --planning-seconds 12 --report /tmp/rectangle_16_wall.json
python3 tools/smoke_original_8uav.py random --rows 4 --columns 4 \
  --port 11338 --planning-seconds 12 --report /tmp/rectangle_16_random.json
python3 tools/smoke_original_8uav.py cluttered --rows 4 --columns 4 \
  --port 11339 --planning-seconds 12 --report /tmp/rectangle_16_cluttered.json
```

将行列改为 `2 / 3`、`3 / 4` 可检查另外两种规模。检查包含地图、静态初始位置、
全部局部扫描、无人机 mesh 与比例、有限值轨迹及每架飞机的实际位移。
此项验证覆盖启动与短时规划执行，完整航程的到达率和碰撞率需要另行实验统计。
容器的 OpenSSL 加载问题可使用 README 中的 `tools/with_noetic_runtime.sh` 包装运行。

此前固定墙和稀疏随机六组均通过 12 秒规划执行检查，16 机随机场景同时检查了 RViz 启动。
同一类场景在三种规模下的点云 SHA-256 完全一致，所有飞机均收到模型和局部扫描、
发布有限值轨迹并产生位移。与参考工作目录的六组环境参数和槽位对齐，
原八机四组参数与节点回归检查也通过；官方规划器原有 63 个文件逐字节未变。
记录见 [六组验证数据](rectangular_formation_validation.json)。

2026-11-28 密集方柱检查：8 项配置测试全部通过；两项目实际地图生成器发布的
60 根障碍物几何和 157,152 个点的点云数据逐字节一致，并检查了端点净空和柱间间隔。
十六机无界面运行通过 12 秒短时规划执行检查，全部 16 路扫描、模型和有限值轨迹均收到，
每机位移至少 4.01 m。此项结果仍只覆盖启动和短时执行。
记录及退出阶段日志说明见 [方柱验证数据](cluttered_random_validation.json)。
