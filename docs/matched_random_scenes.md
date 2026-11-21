# 两工程一致的十组随机障碍物场景

当前官方基线的 `sparse_random_obstacle_map.cpp` 已同步为
`EGO-Planner-v2-commented` 中的小块独立位置采样实现。
`small_obstacle_count` 和 `small_position_seed` 现在具有相同语义和实际输出。

## 参数

| 参数 | 用途 | 本次十组场景 |
|---|---|---|
| `random_seed` | 基础地图种子，涉及大障碍物等固定部分 | 始终为 `17` |
| `small_obstacle_count` | 小块数量；`0` 表示去掉全部小块 | `8` |
| `small_position_seed` | 非负数独立采样小块位置，相同参数可复现 | 依次为 `1`–`10` |

保持基础种子和其余地图参数相同，改变小块数量/位置不会改变 4 个大障碍物、
3 个堆叠块、2 段窄缝墙，共 9 个固定实体。非负位置种子下，每个小块的尺寸
独立于位置采样；相同数量、不同位置种子的各小块尺寸相同。

位置种子为负数时使用原始采样序列，`small_obstacle_count:=6 small_position_seed:=-1`
恢复默认布局。此前录制的默认随机场景仍可继续使用；本次核对会验证默认几何保留。
非默认位置种子改用当前 `commented` 实现，应以新清单作为后续对比实验依据。

## 本机环境

宿主机的 ROS Noetic 位于 `split-merge` 容器。进入一个新的 Noetic 终端：

```bash
docker exec -it split-merge bash --noprofile --norc
source /opt/ros/noetic/setup.bash
```

在官方基线终端加载：

```bash
cd /home/rise-lab/split-merge/EGO-Planner-v2-official-baseline/swarm-playground/formation_ws
source devel/original_baseline/setup.bash
rospack find ego_planner
```

`rospack find` 应指向本工程 `formation_ws/src/planner/plan_manage`。
在另一个干净终端中运行 `commented` 工程时加载其环境：

```bash
source /opt/ros/noetic/setup.bash
cd /home/rise-lab/split-merge/EGO-Planner-v2-commented/swarm-playground/formation_ws
source devel/setup.bash
rospack find ego_planner
```

两个工程共用同名 ROS 包，应分别加载相应工作空间。交替运行实验时，先 Ctrl+C
结束上一场景，再启动另一工程；两边的官方 UDP 通信桥还有相同的固定端口。

## 第 1 组

官方基线：

```bash
roslaunch ego_planner rectangle_8uav_sparse_random.launch \
  random_seed:=17 small_obstacle_count:=8 small_position_seed:=1
```

`commented` 分布式模式：

```bash
roslaunch ego_planner rectangle_8uav_sparse_random.launch \
  coordination_mode:=distributed random_seed:=17 \
  small_obstacle_count:=8 small_position_seed:=1
```

## 第 2–10 组

只将两条命令中的 `small_position_seed` 同时改为对应的 `2`、`3`、…、`10`。
例如第 10 组在官方基线中为：

```bash
roslaunch ego_planner rectangle_8uav_sparse_random.launch \
  random_seed:=17 small_obstacle_count:=8 small_position_seed:=10
```

每组均有 17 个实体：9 个固定实体加 8 个小块。两边同组的地图应完全一致，
不同组的小块位置不同。固定清单 [random_scene_suite.json](random_scene_suite.json)
逐组保存两工程完整命令、实体坐标/尺寸、几何及点云 SHA-256，便于实验登记和复核。

## 调整数量和恢复默认

```bash
# 去掉全部小块，保留其余 9 个实体
roslaunch ego_planner rectangle_8uav_sparse_random.launch \
  random_seed:=17 small_obstacle_count:=0 small_position_seed:=1

# 恢复原默认：6 个小块，原始布局
roslaunch ego_planner rectangle_8uav_sparse_random.launch \
  random_seed:=17 small_obstacle_count:=6 small_position_seed:=-1
```

## 录包脚本

录包入口也支持数量开关。在已加载本基线的 Noetic 环境中，第 1 组录 10 次：

```bash
python3 /home/rise-lab/split-merge/EGO-Planner-v2-official-baseline/tools/record_random_benchmark.py \
  --runs 10 --random-seed 17 --small-obstacle-count 8 --small-position-seed 1 \
  --output /home/rise-lab/split-merge/EGO-Planner-v2-official-baseline/recordings/random_scene01_10runs
```

其他组同样修改位置种子和输出目录。录包默认仍为 6 块、位置种子 -1；
每次的实际种子、数量、参数及实体几何都会存入 metadata。

## 一致性验证

`tools/check_random_scene_suite.py` 只启动地图生成器，不运行无人机。它分别解析
两工程实际 launch 参数，并直接执行各自已编译的地图程序，比较实体几何与点云字节。
覆盖默认布局、0/6/8 个小块、位置种子 1–10、重复种子复现；同时验证 9 个固定实体
不变、不同位置种子下的小块尺寸不变，以及已录默认 bag 的几何保留。

在本工程已加载的 Noetic 环境中复核：

```bash
python3 /home/rise-lab/split-merge/EGO-Planner-v2-official-baseline/tools/check_random_scene_suite.py \
  --reference /home/rise-lab/split-merge/EGO-Planner-v2-commented \
  --port 11565 \
  --output /home/rise-lab/split-merge/EGO-Planner-v2-official-baseline/recordings/random_scene_recheck
```

输出目录必须尚不存在，端口应空闲。新清单记录本次实际测试的程序结果；以后任一
工程调整地图生成逻辑或地图参数，重新核对后再进行成组对比。
早期 `original_8uav_scene_sources.json` 是原场景来源快照，本次随机场景同步记录在新清单中。
