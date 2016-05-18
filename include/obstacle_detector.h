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

#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <obstacle_detector/Obstacles.h>

#include "../include/point.h"
#include "../include/segment.h"
#include "../include/circle.h"
#include "../include/figure_fitting.h"

namespace obstacle_detector
{

class ObstacleDetector
{
public:
  ObstacleDetector();

private:
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
  void pclCallback(const sensor_msgs::PointCloud::ConstPtr& pcl);
  void updateParams();

  void processPoints();
  void groupPointsAndDetectSegments();
  void detectSegments(std::list<Point>& point_set);
  void mergeSegments();
  bool compareAndMergeSegments(Segment& s1, Segment& s2);

  void detectCircles();
  void mergeCircles();
  bool compareAndMergeCircles(Circle& c1, Circle& c2);

  void publishObstacles();
  void transformToWorld();

  // ROS handlers
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  ros::Subscriber scan_sub_;
  ros::Subscriber pcl_sub_;
  ros::Publisher  obstacles_pub_;

  tf::TransformListener tf_listener_;

  // Detector variables
  std::vector<Point> initial_points_;
  std::list<Segment> segments_;
  std::list<Circle>  circles_;

  // Parameters
  std::string p_world_frame_;     // Name of the world coordinate frame
  std::string p_scanner_frame_;   // Name of the scanner coordinate frame

  bool p_use_scan_;               // Use data from scans
  bool p_use_pcl_;                // Use data from point clouds
  bool p_use_split_and_merge_;    // If false, iterative closest point is used instead of split and merge
  bool p_transform_to_world;      // Transform obstacles to world coordinate frame

  int    p_min_group_points_;     // Miminal number of points in a set to process it further
  double p_distance_proportion_;  // Proportion of allowable distances to the range of a point (based on scan angle increment)
  double p_max_group_distance_;   // Maximal allowable distance between two points in a set

  double p_max_split_distance_;   // Maximal allowable distance between a point and a leading line in the splitting process
  double p_max_merge_separation_; // Maximal allowable distance between two segments when merging
  double p_max_merge_spread_;     // Maximal allowable spread of initial segments around merged segment
  double p_max_circle_radius_;    // Maximal allowable radius of a detected circle
  double p_radius_enlargement_;   // Additional boundary for the obstacle

  double p_max_scanner_range_;    // Restrictions on laser scanner
  double p_max_x_range_;          // Restrictions on world coordinates
  double p_min_x_range_;
  double p_max_y_range_;
  double p_min_y_range_;
};

} // namespace obstacle_detector
