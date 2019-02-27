cmake_minimum_required(VERSION 2.8.3)
project(cartographer_hokuyo3d)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
)

add_compile_options(-std=c++11)

include_directories(${catkin_INCLUDE_DIRS} src)

add_executable(imu_compensate src/imu_compensate.cpp)
target_link_libraries(imu_compensate ${catkin_LIBRARIES})

install(TARGETS imu_compensate
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY configuration_files/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/configuration_files
)
