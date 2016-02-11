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
target_link_libraries(obstacle_detector ${catkin_LIBRARIES} /usr/local/lib/libarmadillo.so)
add_dependencies(obstacle_detector ${PROJECT_NAME}_gencpp)


add_executable(static_scan_publisher src/static_scan_publisher.cpp)
target_link_libraries(static_scan_publisher ${catkin_LIBRARIES})
add_dependencies(static_scan_publisher ${PROJECT_NAME}_gencpp)


add_executable(scans_merger src/scans_merger.cpp include/scans_merger.h)
target_link_libraries(scans_merger ${catkin_LIBRARIES})
add_dependencies(scans_merger ${PROJECT_NAME}_gencpp)


#add_executable(obstacle_tracker src/obstacle_tracker.cpp include/obstacle_tracker.h)
#target_link_libraries(obstacle_tracker ${catkin_LIBRARIES})
#add_dependencies(obstacle_tracker ${PROJECT_NAME}_gencpp)

