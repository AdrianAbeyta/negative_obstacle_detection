cmake_minimum_required(VERSION 3.0.2)
project(negative_obstacle_detection)

find_package(catkin REQUIRED
 roscpp
 std_msgs
 tf
 tf2_ros
 pcl_conversions
 pcl_ros
 laser_geometry
 message_filters
)

catkin_package()


include_directories(
include
 ${catkin_INCLUDE_DIRS}
pcl_conversions 
tf2_ros
)


add_executable(${PROJECT_NAME}_node src/negative_obstacle_detection_main.cpp src/PointCloudToLaser.cpp src/FilterPCL.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

