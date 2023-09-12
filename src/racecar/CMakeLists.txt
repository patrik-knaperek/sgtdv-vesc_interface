cmake_minimum_required(VERSION 3.0.2)
project(racecar)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sgtdv_msgs
        )

#add_message_files(
#        FILES
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

add_executable(sgt_vesc_interface src/sgt_vesc_interface_node.cpp src/sgt_vesc_interface.cpp)

target_link_libraries(sgt_vesc_interface
        ${catkin_LIBRARIES}
        )

add_dependencies(sgt_vesc_interface ${catkin_EXPORTED_TARGETS})
