cmake_minimum_required(VERSION 2.8.3)
project(exploration_manager)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS
  core
  imgproc
  highgui
  features2d
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  plan_env
  path_searching
  bspline
  bspline_opt
  traj_utils
  active_perception
  lkh_tsp_solver
  lkh_mtsp_solver
  plan_manage
  cv_bridge
  message_generation
)

add_message_files(
  FILES
  DroneState.msg
  PairOpt.msg
  PairOptResponse.msg
  HGrid.msg
  GridTour.msg
  IslandBox.msg
  IslandArray.msg
  CamSwitch.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES exploration_manager
 CATKIN_DEPENDS plan_env path_searching bspline bspline_opt active_perception traj_utils lkh_tsp_solver lkh_mtsp_solver plan_manage message_runtime
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(exploration_node
  src/exploration_node.cpp
  src/fast_exploration_fsm.cpp
  src/fast_exploration_manager.cpp
  src/island_finder.cpp
  src/occmap_processing.cpp
  )
target_link_libraries(exploration_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

add_executable(ground_node
  src/ground_node.cpp
  src/fast_exploration_manager.cpp
  src/island_finder.cpp 
  src/occmap_processing.cpp
  )
target_link_libraries(ground_node
  ${catkin_LIBRARIES}
  )
