cmake_minimum_required(VERSION 3.5)
project(turtlebot3_move)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/move_turtlebot3.py
DESTINATION lib/${PROJECT_NAME})

ament_package()
