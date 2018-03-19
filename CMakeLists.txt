cmake_minimum_required(VERSION 2.8.3)
project(cartographer_hokuyo3d)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY configuration_files/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/configuration_files
)
