cmake_minimum_required(VERSION 3.5)
project(bbot_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
