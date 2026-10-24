# 官方 EGO-Planner-v2：八机同场景对照

本分支以官方提交 `9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010` 为基础。
对照 `EGO-Planner-criterion-validation` 当前工作目录，复现固定墙、随机障碍物、
八机初始编队和目标。官方 `formation_ws/src/planner` 中原有的源码、头文件、
构建配置和启动文件全部保持原样，新的启动入口单独存放。

## 编译与运行

请在新终端操作，两个项目分别运行。它们使用相同 ROS 话题，不能在同一个
ROS master 中同时启动。原有 `devel/setup.zsh` 可能包含早期修改版本的二进制，
本对照使用独立的构建目录和开发目录。

```zsh
cd /home/robot/Codes/EGO-Planner-v2-commented/swarm-playground/formation_ws
source /opt/ros/noetic/setup.zsh
catkin_make --build build/original_baseline \
  -DCATKIN_DEVEL_PREFIX="$PWD/devel/original_baseline" -j2 -l2
source devel/original_baseline/setup.zsh
rospack find ego_planner
```

`rospack find` 应指向当前目录内的 `formation_ws/src/planner/plan_manage`。

固定墙场景：

```zsh
roslaunch ego_planner rectangle_8uav_fixed_split_merge_wall.launch
```

随机障碍物场景：

```zsh
roslaunch ego_planner rectangle_8uav_sparse_random.launch
```

小障碍物重排，与验证项目使用完全相同的两个种子：

```zsh
roslaunch ego_planner rectangle_8uav_sparse_random.launch \
  random_seed:=17 small_position_seed:=42
```

无界面运行及记录数据：

```zsh
roslaunch ego_planner rectangle_8uav_fixed_split_merge_wall.launch \
  rviz:=false record_bag:=true bag_prefix:=/tmp/original_wall_trial01
roslaunch ego_planner rectangle_8uav_sparse_random.launch \
  rviz:=false record_bag:=true bag_prefix:=/tmp/original_random_trial01
```

以上命令分别运行。bag 包含全局地图、物理几何、八机里程计、实际位置指令、
规划轨迹和汇总日志。地图仅交给模拟传感器；规划器仍接收各机局部点云。

如需先观察静态初始场景，添加 `start_planning:=false`。准备好后从同一 ROS
环境的另一终端发送一次：

```zsh
rostopic pub -1 /traj_start_trigger geometry_msgs/PoseStamped '{}'
```

## 与验证项目对齐的条件

| 项目 | 固定墙 | 随机障碍物 |
|---|---|---|
| 地图范围 | 24 × 14 × 3 m | 32 × 14 × 3 m |
| 编队中心起点 | (-6, 0, 1.5) | (-14, 0, 1.5) |
| 编队中心目标 | (10, 0, 1.5) | (14, 0, 1.5) |
| 编队 | 2 × 4，纵横相邻间距均为 1 m | 相同 |
| 物理编号 0–3 的偏移 | (0.5, -1.5/-0.5/0.5/1.5, 0) | 相同 |
| 物理编号 4–7 的偏移 | (-0.5, -1.5/-0.5/0.5/1.5, 0) | 相同 |
| 速度 / 加速度限制 | 0.5 m/s / 6 m/s² | 相同 |
| 局部感知 | 360°，半径 5 m，保留相同垂直裁剪 | 相同 |
| 安全距离参数 | 0.2 m | 相同 |
| 编队代价权重 | 200 | 相同 |
| 轨迹约束采样 / 软障碍权重 | 8 / 10000 | 相同 |

固定墙的第一堵墙中心为 (-2, 0)，厚 0.5 m、宽 4 m；第二堵墙位于
x=6.5，厚 0.5 m、总宽 10 m、中间净几何开口 1.8 m。
墙体点云从 z=-0.1 到 z=3.0，分辨率 0.1 m。

随机场景默认 `random_seed:=17`、`small_position_seed:=-1`，包含 4 个大障碍、
6 个小障碍、3 个堆叠块和 2 段窄通道墙。其尺寸、位置采样、预留区、端点净空、
窄通道及小障碍位置种子功能均复制自验证项目当前源码。

## 实现边界和来源

- 两个地图生成器从验证工作目录逐字节复制，构建仅增加对应目标及消息依赖。
- 点云模拟传感器从验证项目复制，支持相同的 360° 视场、空扫描和测量时间戳。
- `poscmd_2_odom` 从验证项目复制，使用相同的 0.2 s 指令超时及悬停速度报告。
  这是共同的模拟器条件，不是官方规划算法的一部分。
- 新增的 `baseline_8uav_advanced.xml` 从官方参数文件派生，只调整场景和两组共有的
  参数；原 `advanced_param.xml` 等启动文件未改动。
- 原始规划器的编队代价、轨迹优化、栅格建图、重规划状态机、轨迹服务器和通信桥
  均保留官方源码。不包含星凸决策、分裂合并、换槽、拍卖、研究版本的附加防护和
  路由反馈节点。入口不接受 `coordination_mode` 或 `swap_enabled`。
- 默认槽位固定为物理编号 0–7。验证算法运行中的分裂、重组及换槽是被比较的方法行为，
  因此运行过程中的队形、飞行轨迹和最终编号分配不要求相同。

来源版本及逐文件 SHA-256 见 `original_8uav_scene_sources.json`。
来源工作目录当时有未提交修改，特别是随机小障碍物位置种子功能；本次复制包含这些修改。
此处记录的是本次来源快照，之后验证项目若改场景，需要重新对齐。

原版算法自身的行为也予以保留。例如，单航点任务结束后原状态机会再初始化同一航点，
可能产生零方向向量；未移植研究版本针对此问题的修复。正式比较请按实际里程计统计
到达、碰撞、耗时和路径长度，并明确统计截止条件；短时启动检查不代表任务安全完成。

## 可重复的核对

在仓库根目录执行：

```zsh
source /opt/ros/noetic/setup.zsh
python3 tools/check_original_8uav_scenes.py
```

核对内容包含：官方规划器已有 63 个文件逐字节未变；复制文件的 SHA-256；
两套实际解析后的固定墙/随机场景参数、八机编号与槽位、初始位置、目标、模拟器和
传感器参数；共有规划参数；默认地图和小障碍种子 42，以及自定义墙开口参数。
这个检查直接读取验证工作目录，若那边的来源文件变化会报错。

编译完成并加载本对照开发目录后，可在独立端口执行短时集成检查：

```zsh
python3 tools/smoke_original_8uav.py wall --port 11327 --report /tmp/original_wall_smoke.json
python3 tools/smoke_original_8uav.py random --port 11328 --report /tmp/original_random_smoke.json
```

集成检查自动启动和关闭无界面仿真，核实地图、初始八机槽位、八路局部点云、
八路有限值轨迹及实际运动。它不是完整航程成功率或安全性实验。

## 本次验证结果

独立目录全量编译通过。四组配置核对通过：固定墙默认参数、自定义墙开口、
随机种子 17，以及小障碍物位置种子 42。每组共有的 696 个规划参数一致，
固定墙的 214 个环境参数和随机场景的 232 个环境参数一致。

固定墙和默认随机场景分别完成 20 秒规划/执行检查：八机初始槽位正确、
八路局部点云收到、八机均发布并执行有限值轨迹。固定墙点云 24000 点，
随机场景点云 102368 点。仿真进程已关闭，验证项目未改动。

详细数据见 [original_8uav_validation.json](original_8uav_validation.json)。
上述检查不包含完整航程的成功率、碰撞率或安全净空统计。
