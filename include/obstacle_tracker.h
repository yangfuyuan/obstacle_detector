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
#include "../include/kalman.h"

namespace obstacle_detector
{

double costFunction(const CircleObstacle& c1, const CircleObstacle& c2) {
  return sqrt(pow(c1.center.x - c2.center.x, 2.0) + pow(c1.center.y - c2.center.y, 2.0) + pow(c1.radius - c2.radius, 2.0));
}

class TrackedObstacle {
public:
  TrackedObstacle(const CircleObstacle& init_obstacle, int fade_counter_max) : kf(0, 3, 6) {
    obstacle = init_obstacle;
    fade_counter_size = fade_counter_max;

    double TP = 0.01;

    kf.A(0, 1) = TP;
    kf.A(2, 3) = TP;
    kf.A(4, 5) = TP;

    kf.C(0, 0) = 1;
    kf.C(1, 2) = 1;
    kf.C(2, 4) = 1;

    kf.R *= 0.0001;

    kf.q_pred(0) = obstacle.center.x;
    kf.q_pred(2) = obstacle.center.y;
    kf.q_pred(4) = obstacle.radius;

    kf.q_est(0) = obstacle.center.x;
    kf.q_est(2) = obstacle.center.y;
    kf.q_est(4) = obstacle.radius;
  }

  void updateMeasurement(const CircleObstacle& new_obstacle) {
    kf.y(0) = new_obstacle.center.x;
    kf.y(1) = new_obstacle.center.y;
    kf.y(2) = new_obstacle.radius;

    fade_counter = fade_counter_size;
  }

  void updateTracking() {
    kf.updateState();

    obstacle.center.x = kf.q_est(0);
    obstacle.center.y = kf.q_est(2);
    obstacle.radius = kf.q_est(4);

    fade_counter--;
  }

  CircleObstacle obstacle;
  int fade_counter;       // If the fade counter reaches 0, remove the obstacle from the list

private:
  KalmanFilter kf;
  int fade_counter_size;
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

  // Obstacle Tracker specific fields
  Obstacles tracked_obstacles_msg_;

  std::vector<TrackedObstacle> tracked_obstacles_;
  std::vector<CircleObstacle> untracked_obstacles_;

  int p_fade_counter_; // After this many iteration without update, the obstacle will be discarded
};

} // namespace obstacle_detector
