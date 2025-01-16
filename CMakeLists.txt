cmake_minimum_required(VERSION 3.0.2)
project(vesc_interface)

add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sgtdv_msgs
)

#add_message_files(
#  FILES
#)

#generate_messages(
#        DEPENDENCIES
#)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sgtdv_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

add_executable(vesc_interface src/vesc_interface_node.cpp src/vesc_interface.cpp)
# add_executable(joy_vesc_interface src/joy_vesc_interface_node.cpp src/joy_vesc_interface.cpp)

target_link_libraries(vesc_interface ${catkin_LIBRARIES})
# target_link_libraries(joy_vesc_interface ${catkin_LIBRARIES})

add_dependencies(vesc_interface ${catkin_EXPORTED_TARGETS})
# add_dependencies(joy_vesc_interface ${catkin_EXPORTED_TARGETS})
