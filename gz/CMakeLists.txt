cmake_minimum_required(VERSION 3.0)

find_package(Protobuf REQUIRED)
# pkg_check_modules(PROTOBUF protobuf)

message("------------")
message(${CMAKE_SOURCE_DIR})
SET(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR})
message("------------")

set(sensor_msgs
  msgs/OpticalFlow.proto
  )

PROTOBUF_GENERATE_CPP(SEN_PROTO_SRCS SEN_PROTO_HDRS ${sensor_msgs})
add_library(sensor_msgs SHARED ${SEN_PROTO_SRCS})
add_subdirectory(plugins)