#define ARMA_DONT_USE_CXX11

#include <list>
#include <string>
#include <fstream>
#include <armadillo>
#include <boost/bind.hpp>

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "obstacle_detector/Obstacles.h"

#include "Point.hpp"
#include "Segment.hpp"
#include "Circle.hpp"

using namespace std;
using namespace obstacle_detector;

namespace obstacle_detector
{
  string p_frame_id;              // Name of the coordinate frame for markers
  string p_scan_topic;            // Name of the topic of scans subscription

  bool p_split_and_merge;         // If false, iterative end point fit algorithm will be used
  bool p_detect_circles;          // If false, only line segments will be detected
  bool p_publish_markers;         // If true, visualisation_markers will be published
  bool p_save_snapshot;           // If true, the obstacles are saved in .txt file

  double p_max_scan_range;        // Artificial limitation of scanner range (does not matter if > actual scanner range)
  int    p_min_group_points;      // Miminal number of points in a set to process it further
  double p_max_group_distance;    // Maximal allowable distance between two points in a set
  double p_max_split_distance;    // Maximal allowable distance between a point and a leading line in the splitting process
  double p_max_merge_distance;    // Maximal allowable distance between a point and a line in the merging process
  double p_max_circle_radius;     // Maximal allowable radius of a detected circle
  double p_radius_enlargement;    // Additional boundary for the obstacle

  // Parameters not changeable by parameter server
  double p_distance_proportion;   // Proportion of allowable distances to the range of a point (based on scan angle increment)
  double p_time_increment;        // Duration between two scans
}

void publishObstacles(const list<Segment>& segments, const list<Circle>& circles, ros::Publisher o_pub)
{
  Obstacles obst_msg;

  for (const Segment& s : segments)
  {
    geometry_msgs::Point p;
    p.x = s.first_point.x;
    p.y = s.first_point.y;
    obst_msg.first_points.push_back(p);

    p.x = s.last_point.x;
    p.y = s.last_point.y;
    obst_msg.last_points.push_back(p);
  }

  for (const Circle& c : circles)
  {
    geometry_msgs::Point p;
    p.x = c.centre.x;
    p.y = c.centre.y;
    obst_msg.centre_points.push_back(p);
    obst_msg.radii.push_back(c.radius);
  }

  o_pub.publish(obst_msg);
}

void publishMarkers(const list<Segment>& segments, const list<Circle>& circles, ros::Publisher m_pub)
{
  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker circle_marker;
  circle_marker.header.frame_id = p_frame_id;
  circle_marker.header.stamp = ros::Time::now();
  circle_marker.ns = "circles";
  circle_marker.id = 0;
  circle_marker.type = visualization_msgs::Marker::CYLINDER;
  circle_marker.action = visualization_msgs::Marker::ADD;
  circle_marker.pose.position.x = 0.0;
  circle_marker.pose.position.y = 0.0;
  circle_marker.pose.position.z = -0.1;
  circle_marker.pose.orientation.x = 0.0;
  circle_marker.pose.orientation.y = 0.0;
  circle_marker.pose.orientation.z = 0.0;
  circle_marker.pose.orientation.w = 1.0;
  circle_marker.scale.x = 0.01;
  circle_marker.scale.y = 0.01;
  circle_marker.scale.z = 0.1;
  circle_marker.color.r = 1.0;
  circle_marker.color.g = 0.0;
  circle_marker.color.b = 1.0;
  circle_marker.color.a = 0.2;
  circle_marker.lifetime = ros::Duration(p_time_increment);

  int i = 0;
  for (const Circle& circle : circles)
  {
    circle_marker.pose.position.x = circle.centre.x;
    circle_marker.pose.position.y = circle.centre.y;
    circle_marker.scale.x = 2.0 * circle.radius;
    circle_marker.scale.y = 2.0 * circle.radius;
    circle_marker.id = i++;

    marker_array.markers.push_back(circle_marker);
  }

  visualization_msgs::Marker segments_marker;
  segments_marker.header.frame_id = p_frame_id;
  segments_marker.header.stamp = ros::Time::now();
  segments_marker.ns = "segments";
  segments_marker.id = 0;
  segments_marker.type = visualization_msgs::Marker::LINE_LIST;
  segments_marker.action = visualization_msgs::Marker::ADD;
  segments_marker.pose.position.x = 0.0;
  segments_marker.pose.position.y = 0.0;
  segments_marker.pose.position.z = 0.1;
  segments_marker.pose.orientation.x = 0.0;
  segments_marker.pose.orientation.y = 0.0;
  segments_marker.pose.orientation.z = 0.0;
  segments_marker.pose.orientation.w = 1.0;
  segments_marker.scale.x = 0.01;
  segments_marker.scale.y = 0.01;
  segments_marker.scale.z = 0.01;
  segments_marker.color.r = 1.0;
  segments_marker.color.g = 0.0;
  segments_marker.color.b = 0.0;
  segments_marker.color.a = 1.0;
  segments_marker.lifetime = ros::Duration(p_time_increment);

  for (const Segment& segment : segments)
  {
    geometry_msgs::Point pt;
    pt.x = segment.first_point.x;
    pt.y = segment.first_point.y;
    segments_marker.points.push_back(pt);

    pt.x = segment.last_point.x;
    pt.y = segment.last_point.y;
    segments_marker.points.push_back(pt);
  }

  marker_array.markers.push_back(segments_marker);

  m_pub.publish(marker_array);
}

void saveSnapshot(const list<Segment>& segments, const list<Circle>& circles)
{
  fstream file;
  file.open("/home/tysik/Obstacles.txt", fstream::out);

  std::cout.precision(6);
  file << fixed;

  file << "Segments: (" << segments.size() << ")" << endl;
  file << "first.x \t first.y \t last.x \t last.y" << endl << "---" << endl;
  for (const Segment& segment : segments)
    file << segment << endl;

  file << endl;
  file << "Circles: (" << circles.size() << ")" << endl;
  file << "centre.x \t centre.y \t radius" << endl << "---" << endl;
  for (const Circle& circle : circles)
    file << circle << endl;

  file.close();

  ros::NodeHandle n("~");
  n.setParam("save_snapshot", false);
  p_save_snapshot = false;
}

Circle fitCircle(const list<Point>& point_set)
{
  int N = point_set.size();

  arma::mat input  = arma::mat(N, 3).zeros();   // [x_i, y_i, 1]
  arma::vec output = arma::vec(N).zeros();      // [-(x_i^2 + y_i^2)]
  arma::vec params = arma::vec(3).zeros();      // [a_1 ; a_2 ; a_3]

  int i = 0;
  for (const Point& point : point_set)
  {
    input(i, 0) = point.x;
    input(i, 1) = point.y;
    input(i, 2) = 1.0;

    output(i) = -(pow(point.x, 2) + pow(point.y, 2));
    i++;
  }

  // Find a_1, a_2 and a_3 coefficients from linear regression
  params = arma::pinv(input) * output;

  Circle circle;
  circle.centre.x = -params(0) / 2.0;
  circle.centre.y = -params(1) / 2.0;
  circle.radius   =  sqrt((params(0) * params(0) + params(1) * params(1)) / 4.0 - params(2));

  return circle;
}

Segment fitSegment(const list<Point>& point_set)
{
  int N = point_set.size();

  arma::mat input  = arma::mat(N, 2).zeros();  // [x_i, y_i]
  arma::vec output = arma::vec(N).ones();      // [-C]
  arma::vec params = arma::vec(2).zeros();     // [A ; B]

  int i = 0;
  for (const Point& point : point_set)
  {
    input(i, 0) = point.x;
    input(i, 1) = point.y;
    ++i;
  }

  // Find A and B coefficients from linear regression (assuming C = -1.0)
  params = arma::pinv(input) * output;

  Segment s;
  s.A = params(0);
  s.B = params(1);
  s.C = -1.0;

  s.normal = Point(-s.A, -s.B) / norm(Point(s.A, s.B));
  s.normalize();

  s.first_point = s.getProjectedPoint(point_set.front());
  s.last_point  = s.getProjectedPoint(point_set.back());

  return s;
}

void detectCircles(const list<Segment>& segments, list<Circle>& circles)
{
  for (const Segment& s : segments)
  {
    Circle c(s);
    c.radius += p_radius_enlargement;

    // Check if two circles are intersecting
    // TODO: Check if circle can be merged with ANY of circles in the list not just the last one
    if ((circles.size() > 0) &&
        (c.radius > distanceBetween(circles.back(), c.centre)))
    {
      Circle cx = merge(circles.back(), c);

      if (cx.radius < p_max_circle_radius)
      {
        c = cx;
        circles.pop_back();
      }
    }

    if (c.radius < p_max_circle_radius)
      circles.push_back(c);
  }
}

void detectSegments(list<Point>& input_set, list<Segment>& output_segments)
{
  if (input_set.size() < p_min_group_points)
    return;

  Segment segment;

  if (p_split_and_merge)
    segment = fitSegment(input_set);    // Use split-and-merge algorithm
  else
    segment = Segment(input_set.front(), input_set.back()); // Use iterative end-point-fit algorithm

  list<Point>::iterator set_divider;
  double max_distance = 0.0;
  double distance     = 0.0;

  // Omit first and last point of the set
  for (auto point_itr = ++input_set.begin(); point_itr != --input_set.end(); ++point_itr)
  {
    if ((distance = distanceBetween(segment, *point_itr)) >= max_distance);
    {
      double r = distanceBetween(*point_itr, Point());
      if (distance > p_max_split_distance + r * p_distance_proportion)
      {
        max_distance = distance;
        set_divider = point_itr;
      }
    }
  }

  if (max_distance > 0.0)  // Split the set
  {
    input_set.insert(set_divider, *set_divider);  // Clone the dividing point for each group

    list<Point> subset1, subset2;
    subset1.splice(subset1.begin(), input_set, input_set.begin(), set_divider);
    subset2.splice(subset2.begin(), input_set, set_divider, input_set.end());

    detectSegments(subset1, output_segments);
    detectSegments(subset2, output_segments);
  }
  else // Add the segment
  {
    if (!p_split_and_merge)
      segment = fitSegment(input_set);

    // TODO: Check if segment can be merged with ANY of segments in the list not just the last one
    if (output_segments.back() == segment)
    {
      segment = merge(output_segments.back(), segment);
      output_segments.pop_back();
    }

    output_segments.push_back(segment);
  }
}

void groupScanPoints(const sensor_msgs::LaserScan* scan, list<list <Point> >& output_sets, ros::Publisher scan_pub)
{
  list<Point> point_set;
  Point point;

  double phi = scan->angle_min - scan->angle_increment;

  for (const float& r : scan->ranges)
  {
    phi += scan->angle_increment;

    if (r >= scan->range_min && r <= scan->range_max && r <= p_max_scan_range)
    {
      point.fromPoolarCoords(r, phi);

      if (point_set.size() != 0)
      {
        if (distanceBetween(point, point_set.back()) > (p_max_group_distance + r * p_distance_proportion))
        {
          output_sets.push_back(point_set);
          point_set.clear();
        }
      }
      point_set.push_back(point);
    }
  }
  output_sets.push_back(point_set);
}

void medianFilter(const vector<float, std::allocator<float> >& input_data, vector<float, std::allocator<float> >& filtered_data)
{
  filtered_data.clear();
  filtered_data.push_back(input_data[0]);
  filtered_data.push_back(input_data[1]);

  // Median window is 5 elements wide
  for (size_t i = 2; i < input_data.size() - 2; ++i)
  {
    vector<double> filter_window;
    filter_window.push_back(input_data[i-2]);
    filter_window.push_back(input_data[i-1]);
    filter_window.push_back(input_data[i-0]);
    filter_window.push_back(input_data[i+1]);
    filter_window.push_back(input_data[i+2]);

    sort(filter_window.begin(), filter_window.end());
    filtered_data.push_back(filter_window[2]);
  }
  filtered_data.push_back(input_data[input_data.size() - 2]);
  filtered_data.push_back(input_data[input_data.size() - 1]);
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan, ros::Publisher m_pub, ros::Publisher o_pub, ros::Publisher scan_pub)
{
  p_distance_proportion = scan->angle_increment;
  p_time_increment = scan->scan_time;

  list<Segment> segment_list;
  list<Circle>  circle_list;

  // Filter scan with median filter
  sensor_msgs::LaserScan filtered_scan = *scan;

  medianFilter(scan->ranges, filtered_scan.ranges);
  scan_pub.publish(filtered_scan);

  list<list <Point> > point_sets;
  groupScanPoints(&filtered_scan, point_sets, scan_pub);

  for (auto& point_set : point_sets)
    detectSegments(point_set, segment_list);

  if (p_detect_circles)
    detectCircles(segment_list, circle_list);

  if (p_save_snapshot)
    saveSnapshot(segment_list, circle_list);

  if (p_publish_markers)
    publishMarkers(segment_list, circle_list, m_pub);

  publishObstacles(segment_list, circle_list, o_pub);
}

void initParams(const ros::TimerEvent& event)
{
  ros::NodeHandle nh_priv("~");
  static bool first_call = true;

  if (first_call)
  {
    if (!nh_priv.getParam("frame_id", p_frame_id))
      p_frame_id = "/base";

    if (!nh_priv.getParam("scan_topic", p_scan_topic))
      p_scan_topic = "/scan";

    first_call = false;
  }

  if (!nh_priv.getParam("split_and_merge", p_split_and_merge))
    p_split_and_merge = true;

  if (!nh_priv.getParam("detect_circles", p_detect_circles))
    p_detect_circles = true;

  if (!nh_priv.getParam("publish_markers", p_publish_markers))
    p_publish_markers = true;

  if (!nh_priv.getParam("save_snapshot", p_save_snapshot))
    p_save_snapshot = false;

  if (!nh_priv.getParam("max_scan_range", p_max_scan_range))
    p_max_scan_range = 5.599;

  if (!nh_priv.getParam("min_group_points", p_min_group_points))
    p_min_group_points = 3;

  if (!nh_priv.getParam("max_group_distance", p_max_group_distance))
    p_max_group_distance = 0.100;

  if (!nh_priv.getParam("max_split_distance", p_max_split_distance))
    p_max_split_distance = 0.100;

  if (!nh_priv.getParam("max_merge_distance", p_max_merge_distance))
    p_max_merge_distance = 0.070;

  if (!nh_priv.getParam("max_circle_radius", p_max_circle_radius))
    p_max_circle_radius = 0.300;

  if (!nh_priv.getParam("radius_enlargement", p_radius_enlargement))
    p_radius_enlargement = 0.050;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_detector_node");
  ros::NodeHandle n;

  ROS_INFO("Starting Obstacle Detector");

  {
    ros::TimerEvent e;
    initParams(e);
  }

  ros::Publisher  mark_pub = n.advertise<visualization_msgs::MarkerArray>("obstacle_markers", 5);
  ros::Publisher  scan_pub = n.advertise<sensor_msgs::LaserScan>("filtered_scan", 5);
  ros::Publisher  obst_pub = n.advertise<obstacle_detector::Obstacles>("obstacles", 5);
  ros::Subscriber scan_sub = n.subscribe<sensor_msgs::LaserScan>(p_scan_topic, 5,
                             boost::bind(scanCallback, _1, mark_pub, obst_pub, scan_pub));
  ros::Timer      prms_tim = n.createTimer(ros::Duration(1.0), initParams);

  ros::spin();

  return 0;
}
