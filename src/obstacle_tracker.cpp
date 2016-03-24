#include "../include/obstacle_tracker.h"

using namespace  obstacle_detector;
using namespace arma;

ObstacleTracker::ObstacleTracker() : nh_(""), nh_local_("~") {
  obstacles_sub_ = nh_.subscribe<obstacle_detector::Obstacles>("obstacles", 5, &ObstacleTracker::obstaclesCallback, this);

  ros::spin();
}

void ObstacleTracker::obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr& obstacles) {
  int K = obstacles->circles.size();
  int L = tracked_obstacles_.size();
  int M = untracked_obstacles_.size();

  if (L == 0 && M == 0) {
    untracked_obstacles_.assign(obstacles->circles.begin(), obstacles->circles.end());
    return;
  }

  mat distances = mat(K, L + M, fill::zeros);
  for (int k = 0; k < K; ++k) {
    for (int l = 0; l < L; ++l) {
      distances(k, l) = CircleDistance(obstacles->circles[k], tracked_obstacles_[l]);
    }

    for (int m = 0; m < M; ++m) {
      distances(k, m + L) = CircleDistance(obstacles->circles[k], untracked_obstacles_[m]);
    }
  }

  imat correspondences = imat(K, L + M, fill::zeros);
  findCorrespondences(distances, correspondences);
}

void ObstacleTracker::findCorrespondences(const arma::mat &distances, arma::imat &correspondences) {

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle_tracker");
  ObstacleTracker ot;
  return 0;
}
