cmake_minimum_required(VERSION 2.8.3)
project(plan_env)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake)
find_package(OpenCV REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  message_filters
  message_generation
)

add_message_files(
  FILES
  IdxList.msg
  ChunkStamps.msg
  ChunkData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_env
 CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})


add_library( plan_env 
    src/sdf_map.cpp 
    src/obj_predictor.cpp 
    src/edt_environment.cpp 
    src/raycast.cpp
    src/map_ros.cpp
    src/multi_map_manager.cpp
    )
target_link_libraries( plan_env
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    )  
    
add_dependencies(plan_env ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(obj_generator
    src/obj_generator.cpp 
    )
target_link_libraries(obj_generator 
    ${catkin_LIBRARIES})