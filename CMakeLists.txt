cmake_minimum_required(VERSION 2.8.3)
project(obstacle_detector)

set(CMAKE_CXX_FLAGS "-std=c++0x -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs sensor_msgs visualization_msgs tf message_generation)
find_package(Armadillo REQUIRED)

add_message_files(FILES CircleObstacle.msg SegmentObstacle.msg Obstacles.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES obstacle_detector
   CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs visualization_msgs tf message_runtime
   DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${ARMADILLO_INCLUDE_DIRS})

add_executable(static_scan_publisher src/static_scan_publisher.cpp)
target_link_libraries(static_scan_publisher ${catkin_LIBRARIES})
add_dependencies(static_scan_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(scans_merger src/scans_merger.cpp include/scans_merger.h)
target_link_libraries(scans_merger ${catkin_LIBRARIES})
add_dependencies(scans_merger ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(obstacle_detector src/obstacle_detector.cpp include/obstacle_detector.h
               include/point.h include/segment.h include/circle.h include/figure_fitting.h)
target_link_libraries(obstacle_detector ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})
add_dependencies(obstacle_detector ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(obstacle_tracker src/obstacle_tracker.cpp include/obstacle_tracker.h include/kalman.h)
target_link_libraries(obstacle_tracker ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})
add_dependencies(obstacle_tracker ${PROJECT_NAME}_gencpp)

add_executable(obstacle_visualizer src/obstacle_visualizer.cpp include/obstacle_visualizer.h)
target_link_libraries(obstacle_visualizer ${catkin_LIBRARIES})
add_dependencies(obstacle_visualizer ${PROJECT_NAME}_gencpp)

add_executable(virtual_obstacle_publisher src/virtual_obstacle_publisher.cpp)
target_link_libraries(virtual_obstacle_publisher ${catkin_LIBRARIES})
add_dependencies(virtual_obstacle_publisher ${PROJECT_NAME}_gencpp)
