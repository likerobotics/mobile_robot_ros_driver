cmake_minimum_required(VERSION 2.8.3)
project(mobile_robot_ros_driver)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    genmsg
    joint_state_controller
    robot_state_publisher
    roscpp
    rospy
    std_msgs
    tf
    geometry_msgs
    nav_msgs
)

catkin_package(
    CATKIN_DEPENDS 
)

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

if (CMAKE_VERSION VERSION_LESS 3.0)
    PROJECT(RapidJSON CXX)
else()
    cmake_policy(SET CMP0048 NEW)
    PROJECT(RapidJSON VERSION "${LIB_VERSION_STRING}" LANGUAGES CXX)
endif()



# Declare a C++ library
# add_library(${PROJECT_NAME}
#     src/serial_communication.cpp
#     src/omnicar_ros.cpp 
# )

# target_link_libraries(${PROJECT_NAME}
#   ${catkin_LIBRARIES} ${EXTRA_LIBS}
# )

# add_executable(mobile_robot_ros_driver src/main.cpp)
# target_link_libraries(mobile_robot_ros_driver ${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS scripts/range_sensor_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})