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

#include "../include/kalman.h"

namespace obstacle_detector
{

double costFunction(const CircleObstacle& c1, const CircleObstacle& c2) {
  return sqrt(pow(c1.center.x - c2.center.x, 2.0) + pow(c1.center.y - c2.center.y, 2.0) + pow(c1.radius - c2.radius, 2.0));
}

CircleObstacle mergeCircObstacles(const CircleObstacle& c1, const CircleObstacle& c2) {
  CircleObstacle c;
  c.center.x = (c1.center.x + c2.center.x) / 2.0;
  c.center.y = (c1.center.y + c2.center.y) / 2.0;
  c.radius = (c1.radius + c2.radius) / 2.0;
  return c;
}

class TrackedObstacle {
public:
  TrackedObstacle(const CircleObstacle& init_obstacle, int fade_counter_max) : kf_(0, 3, 6) {
    obstacle = init_obstacle;
    fade_counter_size_ = fade_counter_max;
    fade_counter = fade_counter_max;

    double TP = 0.01; // Sampling time in sec.

    kf_.A(0, 1) = TP;
    kf_.A(2, 3) = TP;
    kf_.A(4, 5) = TP;

    kf_.C(0, 0) = 1;
    kf_.C(1, 2) = 1;
    kf_.C(2, 4) = 1;

    kf_.q_pred(0) = obstacle.center.x;
    kf_.q_pred(2) = obstacle.center.y;
    kf_.q_pred(4) = obstacle.radius;

    kf_.q_est(0) = obstacle.center.x;
    kf_.q_est(2) = obstacle.center.y;
    kf_.q_est(4) = obstacle.radius;
  }

  void setCovariances(double pose_m_var, double pose_p_var, double radius_m_var, double radius_p_var) {
    kf_.R(0, 0) = pose_m_var;
    kf_.R(1, 1) = pose_m_var;
    kf_.R(2, 2) = radius_m_var;

    kf_.Q(0, 0) = pose_p_var;
    kf_.Q(2, 2) = pose_p_var;
    kf_.Q(4, 4) = radius_p_var;
  }

  void updateMeasurement(const CircleObstacle& new_obstacle) {
    kf_.y(0) = new_obstacle.center.x;
    kf_.y(1) = new_obstacle.center.y;
    kf_.y(2) = new_obstacle.radius;

    fade_counter = fade_counter_size_;
  }

  void updateTracking() {
    kf_.updateState();

    obstacle.center.x = kf_.q_est(0);
    obstacle.center.y = kf_.q_est(2);
    obstacle.radius = kf_.q_est(4);

    fade_counter--;
  }

  CircleObstacle obstacle;
  int fade_counter; // If the fade counter reaches 0, remove the obstacle from the list

private:
  KalmanFilter kf_;
  int fade_counter_size_;
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
  ros::Publisher tracked_obstacles_pub_;
  ros::Publisher untracked_obstacles_pub_;

  // Obstacle Tracker specific fields
  Obstacles tracked_obstacles_msg_;
  Obstacles untracked_obstacles_msg_;

  std::vector<TrackedObstacle> tracked_obstacles_;
  std::vector<CircleObstacle> untracked_obstacles_;

  int p_fade_counter_; // After this many iterations without update, the obstacle will be discarded
  double p_min_correspondence_cost_;
  double p_pose_measure_variance_;
  double p_pose_process_variance_;
  double p_radius_measure_variance_;
  double p_radius_process_variance_;
};

} // namespace obstacle_detector
