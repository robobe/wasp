
set(CMAKE_BUILD_TYPE Debug)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


add_library(hello_world SHARED hello_world/hello_world.cc)
target_link_libraries(hello_world ${GAZEBO_LIBRARIES})
install(TARGETS hello_world DESTINATION bin)

add_library(hello_model SHARED hello_model/model_push.cc)
target_link_libraries(hello_model ${GAZEBO_LIBRARIES})
install(TARGETS hello_model DESTINATION bin)

add_library(optical_flow_mockup SHARED optical_flow_mockup/optical_flow_mockup.cpp)
target_link_libraries(optical_flow_mockup ${GAZEBO_LIBRARIES} sensor_msgs)
install(TARGETS optical_flow_mockup DESTINATION bin)

#KeysToJoints
add_library(KeysToJointsPlugin SHARED KeysToJoints/KeysToJointsPlugin.cc)
target_link_libraries(KeysToJointsPlugin ${GAZEBO_LIBRARIES})
install(TARGETS KeysToJointsPlugin DESTINATION bin)

# Velodyne
add_library(VelodynePlugin SHARED VelodynePlugin/velodyne_plugin.cc)
target_link_libraries(VelodynePlugin ${GAZEBO_LIBRARIES})
install(TARGETS VelodynePlugin DESTINATION bin)

add_subdirectory(hello_gui)