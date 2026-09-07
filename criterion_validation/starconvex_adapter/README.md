# Original StarConvex offline adapter

This executable is deliberately outside both ROS workspaces.  It links the
unmodified `libstar_cvx.a` from `/home/robot/Codes/StarConvex` and includes the
original `CorridorBuilder.h` and `MeshTable.hpp`.

The adapter supplies only the missing experiment plumbing:

1. reconstruct each eight-UAV snapshot from the existing score CSV;
2. reproduce `FreeSpaceFactory`'s three radial batches and vertical ray layers
   against an analytic inflated cylinder;
3. reproduce `FreeSpace::create_convexhull`'s two `Corridor::build` passes;
4. query the resulting meshes with the original `MeshTable::isInStarHull`;
5. estimate pairwise 3-D intersection-over-smaller-volume on a common grid.

It does not compile or copy `FreeSpaceFactory` because that class brings in the
complete ROS exploration and mapping stack.  The real construction and point
membership code are still used unchanged.  Analytic cylinder ray casting
replaces `OccGridMap::isVisible`; hit distances are rounded to the same 0.1 m
ray-marching resolution.

Build from the validation repository root:

```bash
source /home/robot/Codes/StarConvex/devel/setup.zsh
cmake -S criterion_validation/starconvex_adapter \
  -B criterion_validation/starconvex_adapter/build
cmake --build criterion_validation/starconvex_adapter/build -j2
```

The checked experiment uses the `advanced_param_5_sim.xml` free-space
parameters: 5 m range, 0.1 rad azimuth step, 0.2 m vertical layers, a 0.5--1.5
m vertical domain, and 0.1 m ray marching.

