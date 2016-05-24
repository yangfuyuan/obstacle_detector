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

#define ARMA_DONT_USE_CXX11

#include <armadillo>
#include <ros/ros.h>
#include <obstacle_detector/Obstacles.h>

#include "../include/circle.h"

namespace obstacle_detector
{

double costFunction(const CircleObstacle& c1, const CircleObstacle& c2) {
  return sqrt(pow(c1.center.x - c2.center.x, 2.0) + pow(c1.center.y - c2.center.y, 2.0) + pow(c1.radius - c2.radius, 2.0));
}

struct TrackData {
  int fade_counter; // If the fade counter reaches 0, remove the obstacle from the list
  double v_x;       // Velocity of the obstacle
  double v_y;
};

struct TrackedObstacle {
  CircleObstacle obstacle;
  TrackData track_data;
};

class ObstacleTracker {
public:
  ObstacleTracker();

private:
  void obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr& obstacles);

  // ROS handlers
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  ros::Subscriber obstacles_sub_;
  ros::Publisher obstacles_pub_;

  std::vector<CircleObstacle> tracked_obstacles_;
  std::vector<CircleObstacle> untracked_obstacles_;
};

} // namespace obstacle_detector
