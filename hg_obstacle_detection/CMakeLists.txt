cmake_minimum_required(VERSION 3.0.2)
project(hg_obstacle_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  pcl_conversions
  message_generation
  geometry_msgs
)


add_message_files(
  FILES
  msgVelodyne.msg
  ParkPoint.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hg_obstacle_detection
  CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(velodyne_parking src/velodyne_parking.cpp)
add_dependencies(velodyne_parking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(velodyne_parking ${catkin_LIBRARIES})

add_executable(velodyne_zone src/velodyne_zone.cpp)
add_dependencies(velodyne_zone ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(velodyne_zone ${catkin_LIBRARIES})
