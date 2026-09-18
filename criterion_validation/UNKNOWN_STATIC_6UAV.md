# 六机未知静态障碍物验证

## 数据边界

- `/map_generator/global_cloud` 是仿真真值，只允许六个 `pcl_render_node` 订阅。
- `swarm_observed_map_6uav` 融合六机当前视场中的最近回波，发布：
  - `/swarm/observed_occupied`：已观测占用体素；
  - `/swarm/observed_free`：射线穿过的已知自由体素；
  - 未出现在以上两层中的体素保持未知。
- `starconvex_shadow_6uav` 只读取上述两层在线地图。未知、占用和超时地图都不能作为合并依据。
- 自动合并不再依赖预设的 `merge_wall_x`，而要求编队边连接周围的已知自由走廊成立。

当前融合地图是静态持久地图，因此本阶段只验证未知静态障碍物，不适用于移动障碍物或在线清除已消失障碍物。

## 固定场景

```bash
cd swarm-playground/formation_ws
source devel/setup.zsh
roslaunch ego_planner formation_6uav_criterion_auto_split_wall.launch
```

## 随机隐藏障碍物场景

```bash
roslaunch ego_planner formation_6uav_unknown_random.launch random_seed:=7
```

建议至少测试种子 `1 2 3 4 5 7 11 19 23 31`。地图生成器会在日志中输出该种子的实际墙体尺寸和位置，便于复现失败。

## 验收检查

1. `rosnode info /starconvex_shadow_6uav` 的订阅列表中不应出现 `/map_generator/global_cloud`。
2. `/swarm/observed_occupied` 与 `/swarm/observed_free` 应持续更新且时间戳一致。
3. 未观察区域应令相应边的 `visibility_weight_*` 为 `0`，不能触发合并。
4. 障碍物进入传感器视场后，编队应按判据分裂；通过障碍后，只有连接走廊被观测为自由且距离满足阈值时才允许合并。
5. 任一失败应记录随机种子、判据 summary、拓扑命令和六机里程计。
