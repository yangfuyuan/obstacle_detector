#include "obstacle_detector/scans2pcl.h"

using namespace obstacle_detector;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan2pcl_node");
  ros::NodeHandle n;
  Scans2PCL *S2P = new Scans2PCL(n);

  ros::spin();

  delete S2P;
  return 0;
}


Scans2PCL::Scans2PCL(ros::NodeHandle n) : nh(n)
{
  scan1_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan_1", 1, &Scans2PCL::scan1Callback, this);
  scan2_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan_2", 1, &Scans2PCL::scan2Callback, this);
  pcl_pub = nh.advertise<PointCloud>("/obstacle/pcl", 1);
  alert_pub = nh.advertise<PointCloud>("/obstacle/alerts", 1);

  omit_overlapping_scans = OMIT_OVERLAPPING_SCANS;
  first_scan_received = false;
  second_scan_received = false;
  unreceived_scans1 = 0;
  unreceived_scans2 = 0;

  pcl_msg.header.frame_id = "base";
  alert_msg.header.frame_id = "base";
}


// Scanner 1 is the front one
void Scans2PCL::scan1Callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  Point2D point;

  float phi = scan_msg->angle_min;

  for (auto &r : scan_msg->ranges)
  {
    if (r > scan_msg->range_min && r < scan_msg->range_max)
    {
      point.x = r*cos(phi) + SCANNERS_SEPARATION/2; // Y_|X
      point.y = r*sin(phi);

      if (r < ALERT_DISTANCE)
        alert_msg.points.push_back(point);

      if (!(omit_overlapping_scans && point.x < 0.0))
        pcl_msg.points.push_back(point);
    }

    phi += scan_msg->angle_increment;
  }

  first_scan_received = true;

  if (second_scan_received || unreceived_scans2 > MAX_UNRECEIVED_SCANS)
  {
    publishPCL();
    publishAlert();

    unreceived_scans1 = 0;
  }
  else unreceived_scans2++;
}


// Scanner 2 is the rear one
void Scans2PCL::scan2Callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  Point2D point;

  float phi = scan_msg->angle_min;

  for (auto &r : scan_msg->ranges)
  {
    if (r > scan_msg->range_min && r < scan_msg->range_max)
    {
      point.x = -r*cos(phi) - SCANNERS_SEPARATION/2; // Y_|X
      point.y = -r*sin(phi);

      if (r < ALERT_DISTANCE)
        alert_msg.points.push_back(point);

      if (!(omit_overlapping_scans && point.x > 0.0))
        pcl_msg.points.push_back(point);
    }

      phi += scan_msg->angle_increment;
  }

  second_scan_received = true;

  if (first_scan_received || unreceived_scans1 > MAX_UNRECEIVED_SCANS)
  {
    publishPCL();
    publishAlert();

    unreceived_scans2 = 0;
  }
  else unreceived_scans1++;
}


void Scans2PCL::publishPCL()
{
  pcl_msg.header.stamp = ros::Time::now();

  pcl_pub.publish(pcl_msg);

  pcl_msg.points.clear();

  // There is no need to omit overlapping scans from one scan
  omit_overlapping_scans = (unreceived_scans1 <= MAX_UNRECEIVED_SCANS && unreceived_scans1 <= MAX_UNRECEIVED_SCANS);

  first_scan_received = false;
  second_scan_received = false;
}


void Scans2PCL::publishAlert()
{
  alert_msg.header.stamp = ros::Time::now();

  alert_pub.publish(alert_msg);

  alert_msg.points.clear();
}
