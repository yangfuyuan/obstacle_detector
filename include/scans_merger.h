#ifndef SCANS_MERGER_H
#define SCANS_MERGER_H

#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

namespace obstacle_detector
{

class ScansMerger
{
public:
  ScansMerger();

private:
  void frontScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
  void rearScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
  void publishPCL();
  void updateParams(const ros::TimerEvent& event);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  ros::Subscriber front_scan_sub_;
  ros::Subscriber rear_scan_sub_;
  ros::Publisher  pcl_pub_;
  ros::Timer      params_tim_;

  sensor_msgs::PointCloud pcl_msg_;

  bool first_scan_received_;
  bool second_scan_received_;
  int unreceived_scans1_;
  int unreceived_scans2_;

  // Parameters
  std::string p_frame_id_;          // TF frame name for the pcl message
  std::string p_front_scan_topic_;  // Name of the front scan messages topic
  std::string p_rear_scan_topic_;   // Name of the rear scan messages topic
  std::string p_pcl_topic_;         // Name of the pcl messages topic

  bool p_omit_overlapping_scans_;   // Omit the points which project onto area of the other scanner
  double p_scanners_separation_;    // Distance between scanner centers
  int p_max_unreceived_scans_;      // Maximum allowable unreceived scans to start publishing one scan
};

}

#endif // SCANS_MERGER_H
