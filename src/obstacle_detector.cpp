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

ObstacleDetector::ObstacleDetector() : nh_(""), nh_local_("~") {
  updateParams();

  if (p_use_scan_)
    scan_sub_ = nh_.subscribe("scan", 10, &ObstacleDetector::scanCallback, this);
  else if (p_use_pcl_)
    pcl_sub_ = nh_.subscribe("pcl", 10, &ObstacleDetector::pclCallback, this);

  obstacles_pub_ = nh_.advertise<obstacle_detector::Obstacles>("obstacles", 5);

  ROS_INFO("Obstacle Detector [OK]");
  ros::spin();
}

void ObstacleDetector::updateParams() {
  nh_local_.param<std::string>("world_frame", p_world_frame_, "world");
  nh_local_.param<std::string>("scanner_frame", p_scanner_frame_, "scanner");

  nh_local_.param<bool>("use_scan", p_use_scan_, true);
  nh_local_.param<bool>("use_pcl", p_use_pcl_, false);
  nh_local_.param<bool>("transform_to_world", p_transform_to_world, true);
  nh_local_.param<bool>("use_split_and_merge", p_use_split_and_merge_, false);

  nh_local_.param<int>("min_group_points", p_min_group_points_, 3);

  nh_local_.param<double>("max_group_distance", p_max_group_distance_, 0.100);
  nh_local_.param<double>("distance_proportion", p_distance_proportion_, 0.006136);
  nh_local_.param<double>("max_split_distance", p_max_split_distance_, 0.100);
  nh_local_.param<double>("max_merge_separation", p_max_merge_separation_, 0.200);
  nh_local_.param<double>("max_merge_spread", p_max_merge_spread_, 0.070);
  nh_local_.param<double>("max_circle_radius", p_max_circle_radius_, 0.300);
  nh_local_.param<double>("radius_enlargement", p_radius_enlargement_, 0.050);

  nh_local_.param<double>("max_scanner_range", p_max_scanner_range_, 5.0);
  nh_local_.param<double>("max_x_range", p_max_x_range_, 2.0);
  nh_local_.param<double>("min_x_range", p_min_x_range_, -2.0);
  nh_local_.param<double>("max_y_range", p_max_y_range_, 2.0);
  nh_local_.param<double>("min_y_range", p_min_y_range_, -2.0);
}

void ObstacleDetector::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  initial_points_.clear();

  double phi = scan->angle_min - scan->angle_increment;

  for (const float r : scan->ranges) {
    phi += scan->angle_increment;

    if (r >= scan->range_min && r <= scan->range_max && r <= p_max_scanner_range_)
      initial_points_.push_back(Point::fromPoolarCoords(r, phi));
  }

  processPoints();
}

void ObstacleDetector::pclCallback(const sensor_msgs::PointCloud::ConstPtr& pcl) {
  initial_points_.clear();

  for (const geometry_msgs::Point32& p : pcl->points)
    if (Point(p.x, p.y).lengthSquared() <= pow(p_max_scanner_range_, 2.0))
      initial_points_.push_back(Point(p.x, p.y));

  processPoints();
}

void ObstacleDetector::processPoints() {
  segments_.clear();
  circles_.clear();

  groupPointsAndDetectSegments();
  mergeSegments();

  detectCircles();
  mergeCircles();

  if (p_transform_to_world)
    transformToWorld();

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

    if (segment.length() > 0.0) {
      segments_.push_back(segment);
      segments_.back().point_set().assign(point_set.begin(), point_set.end());
    }
  }
}

void ObstacleDetector::mergeSegments() {
  for (auto i = segments_.begin(); i != segments_.end(); ++i) {
    auto j = i;
    for (++j; j != segments_.end(); ++j) {
      if (compareAndMergeSegments(*i, *j)) {  // If merged - a new segment appeared at the end of the list
        auto temp_ptr = i;
        i = segments_.insert(i, segments_.back()); // Copy new segment in place; i now points to new segment
        segments_.pop_back();       // Remove the new segment from the back of the list
        segments_.erase(temp_ptr);  // Remove the first merged segment
        segments_.erase(j);         // Remove the second merged segment
        if (i != segments_.begin())
          i--;                      // The for loop will increment i so let it point to new segment in next iteration
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
  for (auto i = circles_.begin(); i != circles_.end(); ++i) {
    auto j = i;
    for (++j; j != circles_.end(); ++j) {
      if (compareAndMergeCircles(*i, *j)) {      // If merged - a new circle appeared at the end of the list
        auto temp_ptr = i;
        i = circles_.insert(i, circles_.back()); // i now points to new circle
        circles_.pop_back();
        circles_.erase(temp_ptr);
        circles_.erase(j);
        if (i != circles_.begin())
          --i;
        break;
      }
    }
  }
}

bool ObstacleDetector::compareAndMergeCircles(Circle& c1, Circle& c2) {
  // If circle c1 is fully inside c2 - merge and leave as c2
  if (c1.radius() + (c2.center() - c1.center()).length() <= c2.radius()) {
    circles_.push_back(c2);
    return true;
  }

  // If circle c2 is fully inside c1 - merge and leave as c1
  if (c2.radius() + (c2.center() - c1.center()).length() <= c1.radius()) {
    circles_.push_back(c1);
    return true;
  }

  // If circles intersect and are 'small' - merge
  if (c1.radius() + c2.radius() >= (c2.center() - c1.center()).length()) {
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

    for (auto it = circles_.begin(); it != circles_.end(); ++it) {
      if (it->center().x < p_max_x_range_ && it->center().x > p_min_x_range_ &&
          it->center().y < p_max_y_range_ && it->center().y > p_min_y_range_)
      {
        point_l.point.x = it->center().x;
        point_l.point.y = it->center().y;
        tf_listener_.transformPoint(p_world_frame_, point_l, point_w);
        it->setCenter(point_w.point.x, point_w.point.y);
      }
      else {
        it = circles_.erase(it);
        --it;
      }
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
  Obstacles obstacles;
  obstacles.header.stamp = ros::Time::now();

  if (p_transform_to_world)
    obstacles.header.frame_id = p_world_frame_;
  else
    obstacles.header.frame_id = p_scanner_frame_;

  for (const Segment& s : segments_) {
    obstacle_detector::SegmentObstacle segment;

    segment.first_point.x = s.first_point().x;
    segment.first_point.y = s.first_point().y;
    segment.last_point.x = s.last_point().x;
    segment.last_point.y = s.last_point().y;

    obstacles.segments.push_back(segment);
  }

  for (const Circle& c : circles_) {
    obstacle_detector::CircleObstacle circle;

    circle.center.x = c.center().x;
    circle.center.y = c.center().y;
    circle.radius = c.radius();

    obstacles.circles.push_back(circle);
  }

  obstacles_pub_.publish(obstacles);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle_detector");
  ObstacleDetector od;
  return 0;
}
