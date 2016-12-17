cmake_minimum_required(VERSION 2.8.3)
project(ur_nmea_driver)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  )

catkin_package(
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${serial_INCLUDE_DIRS}
  )

add_executable(ur_nmea_driver_node
  src/nmea_driver_config.cpp
  src/nmea_driver_node.cpp
  src/nmea_parser.cpp
  src/main.cpp
  )

target_link_libraries(ur_nmea_driver_node
  ${catkin_LIBRARIES}
  serial
  )
