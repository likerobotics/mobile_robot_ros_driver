cmake_minimum_required(VERSION 2.8.3)
project(omnicar_ros_driver)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    genmsg
    joint_state_controller
    robot_state_publisher
    roscpp
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

# Declare a C++ library
add_library(${PROJECT_NAME}
    src/serial_communication.cpp
    src/omnicar_ros.cpp 
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${EXTRA_LIBS}
)

add_executable(omnicar_driver src/main.cpp)
target_link_libraries(omnicar_driver ${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS scripts/range_sensor_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})