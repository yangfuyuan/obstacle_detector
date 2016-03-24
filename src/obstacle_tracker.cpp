#include "../include/obstacle_tracker.h"

using namespace  obstacle_detector;

ObstacleTracker::ObstacleTracker() : nh_(""), nh_local_("~") {
//  obstacles_sub_ = nh_.subscribe<obstacle_detector::Obstacles>("obstacles", 5, obstaclesCallback);

//  ros::spin();
}

void ObstacleTracker::obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr& obstacles) {
  if (tracked_obstacles_.size() == 0 && untracked_obstacles_.size() == 0) {
//    for (int i = 0; i < obstacles->centre_points.size()) {

//    }
  }
    // 1. Find the corresponding obstacles
    // 2. Update the tracked obstacles
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle_tracker");
  ObstacleTracker ot;
  return 0;
}
