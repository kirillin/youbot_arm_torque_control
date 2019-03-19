cmake_minimum_required(VERSION 2.8.3)
project(youbot_arm_torque_control)

add_compile_options(-std=c++11 -I /usr/include/eigen3/)


find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        sensor_msgs
        message_generation
        geometry_msgs
        brics_actuator
        youbot_arm_kinematics
        tf
)

catkin_package(CATKIN_DEPENDS youbot_arm_kinematics message_runtime std_msgs)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(
        control_node
        src/control_node.cpp
        src/ControlNode.cpp
)

target_link_libraries(control_node ${catkin_LIBRARIES})