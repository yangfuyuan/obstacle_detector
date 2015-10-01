#ifndef SCANS2PCL_H
#define SCANS2PCL_H

#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <obstacle_detector/Point2D.h>
#include <obstacle_detector/PointCloud.h>
#include <obstacle_detector/defines.h>

namespace obstacle_detector
{

class Scans2PCL
{
public:
  Scans2PCL(ros::NodeHandle n);

  ros::NodeHandle nh;
  ros::Subscriber scan1_sub;
  ros::Subscriber scan2_sub;
  ros::Publisher pcl_pub;
  ros::Publisher alert_pub;

  PointCloud pcl_msg, alert_msg;

  bool omit_overlapping_scans;
  bool first_scan_received;
  bool second_scan_received;
  int unreceived_scans1, unreceived_scans2;

  void publishPCL();
  void publishAlert();
  void scan1Callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
  void scan2Callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
};

}

#endif // SCANS2PCL_H
