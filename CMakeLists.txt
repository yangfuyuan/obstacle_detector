cmake_minimum_required(VERSION 2.8.3)
project(obstacle_detector)

set(CMAKE_CXX_FLAGS "-std=c++0x -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")

find_package(catkin REQUIRED COMPONENTS roscpp tf sensor_msgs visualization_msgs geometry_msgs message_generation)

add_message_files(FILES Obstacles.msg)
generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES obstacle_detector
   CATKIN_DEPENDS roscpp tf sensor_msgs visualization_msgs geometry_msgs message_runtime
   DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS} include)

add_executable(obstacle_detector src/obstacle_detector.cpp include/obstacle_detector.h
               include/point.h include/segment.h include/circle.h include/figure_fitting.h)
target_link_libraries(obstacle_detector ${catkin_LIBRARIES} /usr/lib/libarmadillo.so)
add_dependencies(obstacle_detector ${PROJECT_NAME}_gencpp)

add_executable(static_scan_publisher src/static_scan_pub.cpp)
target_link_libraries(static_scan_publisher ${catkin_LIBRARIES})
add_dependencies(static_scan_publisher ${PROJECT_NAME}_gencpp)

add_executable(scans2pcl src/scans2pcl.cpp include/scans2pcl.h)
target_link_libraries(scans2pcl ${catkin_LIBRARIES})
add_dependencies(scans2pcl ${PROJECT_NAME}_gencpp)

