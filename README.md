# This is a Robrex Obstacle Detection package #

The package contains a single node called segments_detector which subscribes to sensor_msgs/LaserScan messages under topic /scan and publishes visualization markers with detected segments.

# The obstacle_detector.launch file #

The obstacle_detector.launch runs several laser scanner tools and determine transformations between specific coordinate frames. The pipeline of the scanner tools goes like this:

1. Two hokuyo nodes collects the data from front and read laser scanners respectively. They publish them under topic /front_scan and /rear_scan with attached coordinate frames /front_laser and /rear_laser.
2. The laserscan_multi_merger node collects both of these messages and merges them into one 360 degree scan attached to coordinate frame /laser and published under topic /scan.
3. The 360 degree scan is being subscribed by the laser_scan_matcher node, which converts it into a transformation between /world and /base coordinate frames, and thus providing the pose estimate.
4. The transformation between /world and /base provided by laser_scan_matcher as well as /scan message serve as input for gmapping node, which builds the map of the robot's environment.
5. Additionally, the /world to /base transformation and /scan message server as input for segments_detector node, which turns the scan into a collection of segments that represent the environment. The segments are published as marker_arrays for Rviz. One marker_array contains line list which shows the actual segments, and the other array contain triangle list which shows the area covered by the scans that make up the segment.
