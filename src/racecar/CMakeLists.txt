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

add_executable(${PROJECT_NAME} src/RaceCarNODE.cpp src/RaceCar.cpp)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
