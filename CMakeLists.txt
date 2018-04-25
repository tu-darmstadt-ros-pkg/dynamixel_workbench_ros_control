cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_workbench_ros_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dynamixel_workbench_toolbox
  hardware_interface
  controller_manager
)


set(HEADERS
  include/${PROJECT_NAME}/dynamixel_hardware_interface.h
  include/${PROJECT_NAME}/timing.h
)

set(SOURCES
  src/dynamixel_hardware_interface.cpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamixel_workbench_ros_control
  CATKIN_DEPENDS roscpp std_msgs dynamixel_workbench_toolbox hardware_interface controller_manager
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_library(dynamixel_workbench_ros_control
 ${HEADERS} ${SOURCES}
)
add_dependencies(dynamixel_workbench_ros_control 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(dynamixel_workbench_ros_control
  ${catkin_LIBRARIES}
)

add_executable(dynamixel_controller_manager_node src/dynamixel_controller_manager_node.cpp)
add_dependencies(dynamixel_controller_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamixel_controller_manager_node dynamixel_workbench_ros_control)
