/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#include "../include/obstacle_detector.h"

using namespace std;
using namespace obstacle_detector;

void ObstacleDetector::updateParams(const ros::TimerEvent& event) {
  static bool first_call = true;

  if (first_call) {
    if (!nh_local_.getParam("world_frame", p_world_frame_))
      p_world_frame_ = "world";

    if (!nh_local_.getParam("scanner_frame", p_scanner_frame_))
      p_scanner_frame_ = "scanner";

    if (!nh_local_.getParam("scan_topic", p_scan_topic_))
      p_scan_topic_ = "scan";

    if (!nh_local_.getParam("pcl_topic", p_pcl_topic_))
      p_pcl_topic_ = "pcl";

    if (!nh_local_.getParam("obstacle_topic", p_obstacle_topic_))
      p_obstacle_topic_ = "obstacles";

    if (!nh_local_.getParam("marker_topic", p_marker_topic_))
      p_marker_topic_ = "obstacle_markers";

    if (!nh_local_.getParam("use_scan", p_use_scan_))
      p_use_scan_ = true;

    if (!nh_local_.getParam("use_pcl", p_use_pcl_))
      p_use_pcl_ = false;

    if (!nh_local_.getParam("transform_to_world", p_transform_to_world))
      p_transform_to_world = true;

    if (!nh_local_.getParam("publish_markers", p_publish_markers_))
      p_publish_markers_ = true;

    first_call = false;
  }

  if (!nh_local_.getParam("save_snapshot", p_save_snapshot_))
    p_save_snapshot_ = false;

  if (!nh_local_.getParam("use_split_and_merge", p_use_split_and_merge_))
    p_use_split_and_merge_ = false;

  if (!nh_local_.getParam("min_group_points", p_min_group_points_))
    p_min_group_points_ = 3;

  if (!nh_local_.getParam("max_group_distance", p_max_group_distance_))
    p_max_group_distance_ = 0.100;

  if (!nh_local_.getParam("distance_proportion", p_distance_proportion_))
    p_distance_proportion_ = 0.006136;

  if (!nh_local_.getParam("max_split_distance", p_max_split_distance_))
    p_max_split_distance_ = 0.100;

  if (!nh_local_.getParam("max_merge_separation", p_max_merge_separation_))
    p_max_merge_separation_ = 0.200;

  if (!nh_local_.getParam("max_merge_spread", p_max_merge_spread_))
    p_max_merge_spread_ = 0.070;

  if (!nh_local_.getParam("max_circle_radius", p_max_circle_radius_))
    p_max_circle_radius_ = 0.300;

  if (!nh_local_.getParam("radius_enlargement", p_radius_enlargement_))
    p_radius_enlargement_ = 0.050;
}

ObstacleDetector::ObstacleDetector() : nh_(), nh_local_("~") {
  updateParams(ros::TimerEvent());

  if (p_use_scan_)
    scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(p_scan_topic_, 5, &ObstacleDetector::scanCallback, this);
  else if (p_use_pcl_)
    pcl_sub_  = nh_.subscribe<sensor_msgs::PointCloud>(p_pcl_topic_, 5, &ObstacleDetector::pclCallback, this);

  if (p_publish_markers_)
    markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(p_marker_topic_, 5);

  obstacles_pub_ = nh_.advertise<obstacle_detector::Obstacles>(p_obstacle_topic_, 5);

  params_tim_ = nh_.createTimer(ros::Duration(1.0), &ObstacleDetector::updateParams, this);
}

void ObstacleDetector::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  initial_points_.clear();

  double phi = scan->angle_min - scan->angle_increment;

  for (const float r : scan->ranges) {
    phi += scan->angle_increment;

    if (r >= scan->range_min && r <= scan->range_max)
      initial_points_.push_back(Point::fromPoolarCoords(r, phi));
  }

  processPoints();
}

void ObstacleDetector::pclCallback(const sensor_msgs::PointCloud::ConstPtr& pcl) {
  initial_points_.clear();

  for (const geometry_msgs::Point32& p : pcl->points)
    initial_points_.push_back(Point(p.x, p.y));

  processPoints();
}

void ObstacleDetector::processPoints() {
  segments_.clear();
  circles_.clear();

  groupPointsAndDetectSegments();
  mergeSegments();
  detectCircles();
//  mergeCircles();

  if (p_transform_to_world)
    transformToWorld();

  if (p_save_snapshot_)
    saveSnapshot();

  if (p_publish_markers_)
    publishMarkers();

  publishObstacles();
}

void ObstacleDetector::groupPointsAndDetectSegments() {
  list<Point> point_set;

  for (const Point& point : initial_points_) {
    if (point_set.size() != 0) {
      double r = point.length();

      if ((point - point_set.back()).lengthSquared() > pow(p_max_group_distance_ + r * p_distance_proportion_, 2.0)) {
        detectSegments(point_set);
        point_set.clear();
      }
    }
    point_set.push_back(point);
  }

  detectSegments(point_set); // Check the last point set too!
}

void ObstacleDetector::detectSegments(list<Point>& point_set) {
  if (point_set.size() < p_min_group_points_)
    return;

  Segment segment(Point(0.0, 0.0), Point(1.0, 0.0));
  if (p_use_split_and_merge_)
    segment = fitSegment(point_set);
  else // Use Iterative End Point Fit
    segment = Segment(point_set.front(), point_set.back());

  list<Point>::iterator set_divider;
  double max_distance = 0.0;
  double distance     = 0.0;

  // Seek the point of division; omit first and last point of the set
  for (auto point_itr = ++point_set.begin(); point_itr != --point_set.end(); ++point_itr) {
    if ((distance = segment.distanceTo(*point_itr)) >= max_distance) {
      double r = (*point_itr).length();

      if (distance > p_max_split_distance_ + r * p_distance_proportion_) {
        max_distance = distance;
        set_divider = point_itr;
      }
    }
  }

  if (max_distance > 0.0) { // Split the set
    point_set.insert(set_divider, *set_divider);  // Clone the dividing point for each group

    list<Point> subset1, subset2;
    subset1.splice(subset1.begin(), point_set, point_set.begin(), set_divider);
    subset2.splice(subset2.begin(), point_set, set_divider, point_set.end());

    detectSegments(subset1);
    detectSegments(subset2);
  } else {  // Add the segment
    if (!p_use_split_and_merge_)
      segment = fitSegment(point_set);

    segments_.push_back(segment);
    segments_.back().point_set().assign(point_set.begin(), point_set.end());
  }
}

void ObstacleDetector::mergeSegments() {
  bool merged = false;

  // TODO: Check this
  for (auto i = segments_.begin(); i != --segments_.end(); ++i) {
    if (merged) {
      --i;   // Check the new segment again
      merged = false;
    }

    auto j = i;
    for (++j; j != segments_.end(); ++j) {
      if (compareAndMergeSegments(*i, *j)) {  // If merged - a new segment appeared at the end of the list
        auto temp_ptr = i;
        i = segments_.insert(i, segments_.back()); // Copy new segment in place; i now points to new segment
        segments_.pop_back();       // Remove the new segment from the back of the list
        segments_.erase(temp_ptr);  // Remove the first merged segment
        segments_.erase(j);         // Remove the second merged segment
        merged = true;
        break;
      }
    }
  }
}

bool ObstacleDetector::compareAndMergeSegments(Segment& s1, Segment& s2) {
  if (&s1 == &s2)
    return false;

  // Segments must be provided counter-clockwise
  if (s1.first_point().cross(s2.first_point()) < 0.0)
    return compareAndMergeSegments(s2, s1);

  if ((s1.last_point() - s2.first_point()).length() < p_max_merge_separation_) {
    list<Point> merged_points;
    merged_points.insert(merged_points.begin(), s1.point_set().begin(), s1.point_set().end());
    merged_points.insert(merged_points.end(), s2.point_set().begin(), s2.point_set().end());

    Segment s = fitSegment(merged_points);

    if (s.distanceTo(s1.first_point()) < p_max_merge_spread_ &&
        s.distanceTo(s1.last_point())  < p_max_merge_spread_ &&
        s.distanceTo(s2.first_point()) < p_max_merge_spread_ &&
        s.distanceTo(s2.last_point())  < p_max_merge_spread_) {
      segments_.push_back(s);
      segments_.back().point_set().assign(merged_points.begin(), merged_points.end());
      return true;
    }
  }

  return false;
}

void ObstacleDetector::detectCircles() {
  for (const Segment& s : segments_) {
    Circle c(s);
    c.setRadius(c.radius() + p_radius_enlargement_);

    if (c.radius() < p_max_circle_radius_)
      circles_.push_back(c);
  }
}

void ObstacleDetector::mergeCircles() {
  bool merged = false;

  for (auto i = circles_.begin(); i != circles_.end(); ++i) {
    if (merged) {
      --i;   // Check the new circle too
      merged = false;
    }

    auto j = i;
    for (++j; j != circles_.end(); ++j) {
      if (compareAndMergeCircles(*i, *j)) {  // If merged - a new circle appeared at the end of the list
        auto temp_ptr = i;
        i = circles_.insert(i, circles_.back()); // i now points to new segment
        circles_.pop_back();
        circles_.erase(temp_ptr);
        circles_.erase(j);
        merged = true;
        break;
      }
    }
  }
}

bool ObstacleDetector::compareAndMergeCircles(Circle& c1, Circle& c2) {
  if (pow(c1.radius() + c2.radius(), 2) < (c2.center() - c1.center()).lengthSquared()) {
    Segment s(c1.center(), c2.center());
    Circle c(s);
    c.setRadius(c.radius() + max(c1.radius(), c2.radius()));

    if (c.radius() < p_max_circle_radius_) {
      circles_.push_back(c);
      return true;
    }
  }
  return false;
}

void ObstacleDetector::transformToWorld() {
  geometry_msgs::PointStamped point_l;  // Point in local (scanner) coordinate frame
  geometry_msgs::PointStamped point_w;  // Point in global (world) coordinate frame

  point_l.header.stamp = ros::Time::now();
  point_l.header.frame_id = p_scanner_frame_;

  point_w.header.stamp = ros::Time::now();
  point_w.header.frame_id = p_world_frame_;

  try {
    tf_listener_.waitForTransform(p_world_frame_, p_scanner_frame_, ros::Time::now(), ros::Duration(3.0));

    for (Circle& circle : circles_) {
      point_l.point.x = circle.center().x;
      point_l.point.y = circle.center().y;
      tf_listener_.transformPoint(p_world_frame_, point_l, point_w);
      circle.setCenter(point_w.point.x, point_w.point.y);
    }

    for (Segment& s : segments_) {
      point_l.point.x = s.first_point().x;
      point_l.point.y = s.first_point().y;
      tf_listener_.transformPoint(p_world_frame_, point_l, point_w);
      s.setFirstPoint(point_w.point.x, point_w.point.y);

      point_l.point.x = s.last_point().x;
      point_l.point.y = s.last_point().y;
      tf_listener_.transformPoint(p_world_frame_, point_l, point_w);
      s.setLastPoint(point_w.point.x, point_w.point.y);
    }
  }
  catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
}

void ObstacleDetector::publishObstacles() {
  Obstacles obst_msg;
  obst_msg.header.stamp = ros::Time::now();

  if (p_transform_to_world)
    obst_msg.header.frame_id = p_world_frame_;
  else
    obst_msg.header.frame_id = p_scanner_frame_;

  for (const Segment& s : segments_) {
    geometry_msgs::Point p;
    p.x = s.first_point().x;
    p.y = s.first_point().y;
    obst_msg.first_points.push_back(p);

    p.x = s.last_point().x;
    p.y = s.last_point().y;
    obst_msg.last_points.push_back(p);
  }

  for (const Circle& c : circles_) {
    geometry_msgs::Point p;
    p.x = c.center().x;
    p.y = c.center().y;
    obst_msg.centre_points.push_back(p);
    obst_msg.radii.push_back(c.radius());
  }

  obstacles_pub_.publish(obst_msg);
}

void ObstacleDetector::publishMarkers() {
  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker circle_marker;
  circle_marker.header.stamp = ros::Time::now();

  if (p_transform_to_world)
    circle_marker.header.frame_id = p_world_frame_;
  else
    circle_marker.header.frame_id = p_scanner_frame_;

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
  circle_marker.lifetime = ros::Duration(0.1);

  int i = 0;
  for (const Circle& circle : circles_) {
    circle_marker.pose.position.x = circle.center().x;
    circle_marker.pose.position.y = circle.center().y;
    circle_marker.scale.x = 2.0 * circle.radius();
    circle_marker.scale.y = 2.0 * circle.radius();
    circle_marker.id = i++;

    marker_array.markers.push_back(circle_marker);
  }

  visualization_msgs::Marker segments_marker;
  segments_marker.header.stamp = ros::Time::now();

  if (p_transform_to_world)
    segments_marker.header.frame_id = p_world_frame_;
  else
    segments_marker.header.frame_id = p_scanner_frame_;

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
  segments_marker.lifetime = ros::Duration(0.1);

  for (const Segment& segment : segments_) {
    geometry_msgs::Point pt;
    pt.x = segment.first_point().x;
    pt.y = segment.first_point().y;
    segments_marker.points.push_back(pt);

    pt.x = segment.last_point().x;
    pt.y = segment.last_point().y;
    segments_marker.points.push_back(pt);
  }

  marker_array.markers.push_back(segments_marker);

  markers_pub_.publish(marker_array);
}

void ObstacleDetector::saveSnapshot() {
  fstream file;
  file.open("/home/tysik/Obstacles.txt", fstream::out);

  std::cout.precision(6);
  file << fixed;

  file << "Segments: (" << segments_.size() << ")" << endl;
  file << "first.x \t first.y \t last.x \t last.y" << endl << "---" << endl;
  for (const Segment& segment : segments_)
    file << segment << endl;

  file << endl;
  file << "Circles: (" << circles_.size() << ")" << endl;
  file << "centre.x \t centre.y \t radius" << endl << "---" << endl;
  for (const Circle& circle : circles_)
    file << circle << endl;

  file.close();

  nh_local_.setParam("save_snapshot", false);
  p_save_snapshot_ = false;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle_detector");
  ObstacleDetector od;

  ROS_INFO("Starting Obstacle Detector");
  ros::spin();

  return 0;
}
