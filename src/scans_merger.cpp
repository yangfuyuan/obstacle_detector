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

#include "../include/scans_merger.h"

using namespace obstacle_detector;

ScansMerger::ScansMerger() : nh_(""), nh_local_("~") {
  updateParams();

  front_scan_sub_ = nh_.subscribe("front_scan", 10, &ScansMerger::frontScanCallback, this);
  rear_scan_sub_ = nh_.subscribe("rear_scan", 10, &ScansMerger::rearScanCallback, this);
  pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud>("pcl", 10);

  first_scan_received_ = false;
  second_scan_received_ = false;
  unreceived_front_scans_ = 0;
  unreceived_rear_scans_ = 0;

  ROS_INFO("Scans Merger [OK]");
  ros::spin();
}

void ScansMerger::updateParams() {
  nh_local_.param<std::string>("frame_id", p_frame_id_, "base");
  nh_local_.param<int>("max_unreceived_scans", p_max_unreceived_scans_, 1);
  nh_local_.param<bool>("omit_overlapping_scans", p_omit_overlapping_scans_, true);
  nh_local_.param<double>("scanners_separation", p_scanners_separation_, 0.45);
}

void ScansMerger::frontScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  geometry_msgs::Point32 point;

  float phi = scan->angle_min;

  for (const float r : scan->ranges) {
    if (r > scan->range_min && r < scan->range_max) {
      point.x = r * cos(phi) + p_scanners_separation_ / 2.0; // Y_|X
      point.y = r * sin(phi);

      if (!(p_omit_overlapping_scans_ && point.x < 0.0))
        pcl_msg_.points.push_back(point);
    }
    phi += scan->angle_increment;
  }

  first_scan_received_ = true;

  if (second_scan_received_ || unreceived_rear_scans_ > p_max_unreceived_scans_) {
    publishPCL();

    unreceived_front_scans_ = 0;
  }
  else unreceived_rear_scans_++;
}

void ScansMerger::rearScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  geometry_msgs::Point32 point;

  float phi = scan->angle_min;

  for (const float r : scan->ranges) {
    if (r > scan->range_min && r < scan->range_max) {
      point.x = -r * cos(phi) - p_scanners_separation_ / 2.0; // Y_|X
      point.y = -r * sin(phi);

      if (!(p_omit_overlapping_scans_ && point.x > 0.0))
        pcl_msg_.points.push_back(point);
    }
    phi += scan->angle_increment;
  }

  second_scan_received_ = true;

  if (first_scan_received_ || unreceived_front_scans_ > p_max_unreceived_scans_) {
    publishPCL();

    unreceived_rear_scans_ = 0;
  }
  else unreceived_front_scans_++;
}

void ScansMerger::publishPCL() {
  pcl_msg_.header.frame_id = p_frame_id_;
  pcl_msg_.header.stamp = ros::Time::now();
  pcl_pub_.publish(pcl_msg_);
  pcl_msg_.points.clear();

  // There is no need to omit overlapping scans from one scan
  p_omit_overlapping_scans_ = (unreceived_front_scans_ <= p_max_unreceived_scans_ && unreceived_rear_scans_ <= p_max_unreceived_scans_);

  first_scan_received_ = false;
  second_scan_received_ = false;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "scans_merger");
  ScansMerger SM;
  return 0;
}
