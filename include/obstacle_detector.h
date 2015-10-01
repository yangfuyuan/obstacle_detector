#ifndef OBSTACLE_DETECTOR_H
#define OBSTACLE_DETECTOR_H

#include <list>
#include <vector>
#include <string>
#include <fstream>

#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <obstacle_detector/Obstacles.h>

#include "point.h"
#include "segment.h"
#include "circle.h"
#include "figure_fitting.h"

namespace obstacle_detector
{

class ObstacleDetector
{
public:
  ObstacleDetector();

private:
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
  void pclCallback(const sensor_msgs::PointCloud::ConstPtr& pcl);
  void updateParams(const ros::TimerEvent& event);

  void publishObstacles();
  void publishMarkers();
  void saveSnapshot();

  void processPoints();
  void groupAndDetectSegments();
  void detectSegments(std::list<Point>& point_set);
  void mergeSegments();
  void detectCircles();

  bool compareAndMerge(Segment &s1, Segment &s2);

  // ROS handlers
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  ros::Subscriber scan_sub_;
  ros::Subscriber pcl_sub_;
  ros::Timer      params_tim_;

  ros::Publisher markers_pub_;
  ros::Publisher obstacles_pub_;

  // Detector variables
  std::vector<Point> initial_points_;
  std::list<Segment> segments_;
  std::list<Circle>  circles_;

  // Parameters
  std::string p_frame_id_;        // Name of the coordinate frame for markers
  std::string p_scan_topic_;      // Name of the topic of scans subscription
  std::string p_pcl_topic_;       // Name of the topic of scans subscription
  std::string p_obstacle_topic_;  // Name of the topic of obstacles publishing
  std::string p_marker_topic_;    // Name of the topic of markers publishing

  bool p_use_scan_;               // Use data from scans
  bool p_use_pcl_;                // Use data from point clouds
  bool p_publish_markers_;        // If true, visualisation_markers will be published
  bool p_save_snapshot_;          // If true, the obstacles data is saved in .txt file

  int    p_min_group_points_;     // Miminal number of points in a set to process it further
  double p_distance_proportion_;  // Proportion of allowable distances to the range of a point (based on scan angle increment)
  double p_max_group_distance_;   // Maximal allowable distance between two points in a set

  double p_max_split_distance_;   // Maximal allowable distance between a point and a leading line in the splitting process
  double p_max_merge_separation_; // Maximal allowable distance between two segments when merging
  double p_max_merge_spread_;     // Maximal allowable spread of initial segments around merged segment
  double p_max_circle_radius_;    // Maximal allowable radius of a detected circle
  double p_radius_enlargement_;   // Additional boundary for the obstacle
};

}

#endif // OBSTACLE_DETECTOR_H
