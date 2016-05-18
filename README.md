## The obstacle_detector package 

The obstacle_detector package provides utilities to detect obstacles from a 2D laser scan or an ordered point cloud. Detected obstacles come in a form of segments and circles representing groups of points.

### The nodes

The package contains several nodes, which are:

* `obstacle_detector` - The main node which converts messages of type `sensor_msgs/LaserScan` from topic `scan` or messages of type `sensor_msgs/PointCloud` from topic `pcl` into obstacles, which are published as messages of custom type `obstacles_detector/Obstacles` under topic `obstacles`. 
* `obstacle_visualizer` - The auxiliary node which converts messages of type `obstacles_detector/Obstacles` from topic `obstacles` into Rviz markers of type `visualization_msgs/MarkerArray`, published under topic `obstacles_markers`.
* `scans_merger` - The auxiliary node which converts two laser scans of type `sensor_msgs/LaserScan` from topics `front_scan` and `rear_scan` into a single point cloud of type `sensor_msgs/PointCloud`, published under topic `pcl`.
* `static_scan_publisher` - The auxiliary node which publish a static, 360 deg laser scan of type `sensor_msgs/LaserScan` under topic `scan`.

