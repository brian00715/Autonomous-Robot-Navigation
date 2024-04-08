cmake_minimum_required(VERSION 3.0.2)
project(final_pnc)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
nav_msgs
geometry_msgs
roscpp
message_generation
)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)

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
LIBRARIES rrt_planner
CATKIN_DEPENDS
  nav_msgs
  geometry_msgs
  roscpp
  message_runtime
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/rrt_planner.cpp
)

add_executable(${PROJECT_NAME}_node src/rrt_planner_node.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)
