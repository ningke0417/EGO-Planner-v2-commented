cmake_minimum_required(VERSION 2.8.3)
project(ego_planner)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  quadrotor_msgs
  plan_env
  path_searching
  traj_opt
  traj_utils
  message_generation
  cv_bridge
)

add_message_files(
  FILES
  FormationTopologyCommand.msg
  FormationTopologyAck.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

set(STARCONVEX_ROOT "/home/robot/Codes/StarConvex" CACHE PATH
    "Root of the unmodified StarConvex checkout")
set(CRITERION_STARCONVEX_INCLUDE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../criterion_validation/starconvex_adapter/include")

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ego_planner
 CATKIN_DEPENDS plan_env path_searching traj_opt traj_utils message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${CRITERION_STARCONVEX_INCLUDE}
  ${STARCONVEX_ROOT}/src/planner/generating_corridor/include
  ${STARCONVEX_ROOT}/src/planner/generating_corridor/ThirdParty
)


add_executable(ego_planner_node
  src/ego_planner_node.cpp 
  src/ego_replan_fsm.cpp
  src/planner_manager.cpp
  )
target_link_libraries(ego_planner_node 
  ${catkin_LIBRARIES}
  )
add_dependencies(ego_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES})
# add_dependencies(traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(starconvex_shadow_4uav_node src/starconvex_shadow_4uav_node.cpp)
target_link_libraries(starconvex_shadow_4uav_node
  ${STARCONVEX_ROOT}/devel/lib/libstar_cvx.a
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(starconvex_shadow_6uav_node src/starconvex_shadow_6uav_node.cpp)
target_link_libraries(starconvex_shadow_6uav_node
  ${STARCONVEX_ROOT}/devel/lib/libstar_cvx.a
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(swarm_observed_map_6uav_node
  src/swarm_observed_map_6uav_node.cpp)
target_link_libraries(swarm_observed_map_6uav_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(formation_backend_visualizer_4uav_node
  src/formation_backend_visualizer_4uav_node.cpp)
target_link_libraries(formation_backend_visualizer_4uav_node
  ${catkin_LIBRARIES}
)

add_executable(formation_backend_visualizer_6uav_node
  src/formation_backend_visualizer_6uav_node.cpp)
target_link_libraries(formation_backend_visualizer_6uav_node
  ${catkin_LIBRARIES}
)

add_executable(formation_auto_split_bridge_4uav_node
  src/formation_auto_split_bridge_4uav_node.cpp)
target_link_libraries(formation_auto_split_bridge_4uav_node
  ${catkin_LIBRARIES}
)
add_dependencies(formation_auto_split_bridge_4uav_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(initial_formation_assignment_4uav_node
  src/initial_formation_assignment_4uav_node.cpp)
target_link_libraries(initial_formation_assignment_4uav_node
  ${catkin_LIBRARIES}
)

add_executable(initial_formation_assignment_6uav_node
  src/initial_formation_assignment_6uav_node.cpp)
target_link_libraries(initial_formation_assignment_6uav_node
  ${catkin_LIBRARIES}
)
add_dependencies(initial_formation_assignment_6uav_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(formation_auto_split_bridge_6uav_node
  src/formation_auto_split_bridge_6uav_node.cpp)
target_link_libraries(formation_auto_split_bridge_6uav_node
  ${catkin_LIBRARIES}
)
add_dependencies(formation_auto_split_bridge_6uav_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_tristate_voxel_field
    test/test_tristate_voxel_field.cpp)
  if(TARGET test_tristate_voxel_field)
    target_link_libraries(test_tristate_voxel_field
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
    )
  endif()
endif()

add_executable(subgroup_route_intent_4uav_node
  src/subgroup_route_intent_4uav_node.cpp)
target_link_libraries(subgroup_route_intent_4uav_node
  ${STARCONVEX_ROOT}/devel/lib/libstar_cvx.a
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(subgroup_route_intent_6uav_node
  src/subgroup_route_intent_4uav_node.cpp)
target_compile_definitions(subgroup_route_intent_6uav_node PRIVATE
  ROUTE_INTENT_UAV_COUNT=6)
target_link_libraries(subgroup_route_intent_6uav_node
  ${STARCONVEX_ROOT}/devel/lib/libstar_cvx.a
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
