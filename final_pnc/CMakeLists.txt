cmake_minimum_required(VERSION 3.0.2)
project(final_pnc)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
nav_msgs
geometry_msgs
roscpp
message_generation
# graph_planner
# global_planner
)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  
)

message(WARNING "catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")

add_message_files(
  FILES
  ReachGoal.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
INCLUDE_DIRS include
# LIBRARIES rrt_planner
CATKIN_DEPENDS
  nav_msgs
  geometry_msgs
  roscpp
  message_runtime
)


add_executable(rrt_planner_node src/rrt_planner_node.cpp src/rrt_planner.cpp)
target_link_libraries(rrt_planner_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

# add_executable(global_planner_node src/global_planner_node.cpp
# src/graph_planner/a_star.cpp
# src/graph_planner/jump_point_search.cpp
# src/graph_planner/d_star.cpp
# src/graph_planner/lpa_star.cpp
# src/graph_planner/d_star_lite.cpp
# src/graph_planner/voronoi.cpp
# src/graph_planner/theta_star.cpp
# src/graph_planner/lazy_theta_star.cpp
# src/graph_planner/s_theta_star.cpp
# src/graph_planner/hybrid_a_star.cpp)
# target_link_libraries(global_planner_node
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
# )

