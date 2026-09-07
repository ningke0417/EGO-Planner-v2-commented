cmake_minimum_required(VERSION 2.8.3)
project(map_generator)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable (random_forest src/random_forest_sensing.cpp )
target_link_libraries(random_forest
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable(fixed_pillar_map src/fixed_pillar_map.cpp)
target_link_libraries(fixed_pillar_map
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable(fixed_wall_map src/fixed_wall_map.cpp)
target_link_libraries(fixed_wall_map
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable(fixed_split_merge_wall_map src/fixed_split_merge_wall_map.cpp)
target_link_libraries(fixed_split_merge_wall_map
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})
