cmake_minimum_required(VERSION 3.10)
project(starconvex_criterion_adapter LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type")

set(STARCONVEX_ROOT "/home/robot/Codes/StarConvex" CACHE PATH
    "Root of the unmodified StarConvex checkout")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

add_executable(starconvex_score_adapter starconvex_score_adapter.cpp)
target_include_directories(starconvex_score_adapter PRIVATE
  "${STARCONVEX_ROOT}/src/planner/generating_corridor/include"
  "${STARCONVEX_ROOT}/src/planner/generating_corridor/ThirdParty"
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(starconvex_score_adapter PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(starconvex_score_adapter PRIVATE
  "${STARCONVEX_ROOT}/devel/lib/libstar_cvx.a"
  ${PCL_LIBRARIES}
)

