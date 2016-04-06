cmake_minimum_required(VERSION 2.8.3)
project(ros_copter)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  cmake_modules
  sensor_msgs
  roscpp
  rospy
  eigen_conversions
  std_msgs
  fcu_common
  cv3_bridge
  dynamic_reconfigure
  tf
)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)


set(CMAKE_CXX_FLAGS "-std=c++0x")

add_message_files(
  FILES
  FilterState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/gains.cfg
  # more files
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_copter
  CATKIN_DEPENDS roscpp rospy nav_msgs std_msgs
)

###########
## Build ##
###########
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

## Declare cpp libraries
add_library(ros_copter
  include/lib/eigen.h
  src/lib/eigen.cpp
)
add_dependencies(ros_copter ros_copter_generate_messages_cpp fcu_common_generate_messages_cpp)
target_link_libraries(ros_copter ${catkin_LIBRARIES})

add_executable(ekf
  src/ekf/ekf_node.cpp
  src/ekf/ekf.cpp
  include/ekf/ekf.h
)
add_dependencies(ekf ros_copter_generate_messages_cpp fcu_common_generate_messages_cpp)
target_link_libraries(ekf ${catkin_LIBRARIES})

## Declare cpp executables
add_executable(mono_vo
  src/mono_vo/mono_vo_node.cpp
  src/mono_vo/mono_vo.cpp
  include/mono_vo/mono_vo.h
)
add_dependencies(mono_vo ros_copter_generate_messages_cpp fcu_common_generate_messages_cpp)
target_link_libraries(mono_vo ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(controller
  src/controller/controller_node.cpp
  src/controller/controller.cpp
  include/controller/controller.h
)
add_dependencies(controller ros_copter_generate_messages_cpp fcu_common_generate_messages_cpp)
target_link_libraries(controller ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(joy_teleop
  src/teleop/joy.cpp
  include/teleop/joy.h
)
add_dependencies(joy_teleop ros_copter_generate_messages_cpp fcu_common_generate_messages_cpp)
target_link_libraries(joy_teleop ${catkin_LIBRARIES})
