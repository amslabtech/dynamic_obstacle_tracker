cmake_minimum_required(VERSION 3.16.3)
project(dynamic_obstacle_tracker)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf2_eigen
  tf2_geometry_msgs
  visualization_msgs
)
find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(dynamic_obstacle_tracker src/dynamic_obstacle_tracker.cpp)
target_link_libraries(dynamic_obstacle_tracker ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    find_package(roslint REQUIRED)
    roslint_cpp()
    roslint_add_test()
endif()
