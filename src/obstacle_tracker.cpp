#include "../include/obstacle_tracker.h"

using namespace  obstacle_detector;
using namespace arma;
using namespace std;

ObstacleTracker::ObstacleTracker() : nh_(""), nh_local_("~") {
  obstacles_sub_ = nh_.subscribe("obstacles", 10, &ObstacleTracker::obstaclesCallback, this);
  obstacles_pub_ = nh_.advertise<obstacle_detector::Obstacles>("tracked_obstacles", 10);

  ROS_INFO("Obstacle Tracker [OK]");
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

  /*
   * Cost between two obstacles represents their difference.
   * The bigger cost, the less similar they are.
   * K rows of cost_matrix represent new obstacles.
   * L+M columns of cost matrix represent old tracked and untracked obstacles.
   */
  mat cost_matrix = mat(K, L + M, fill::zeros);
  vector<int> min_indices(K, -1);

  for (int k = 0; k < K; ++k) {
    double min_cost = 1000;

    for (int l = 0; l < L; ++l) {
      cost_matrix (k, l) = costFunction(obstacles->circles[k], tracked_obstacles_[l]);

      if (cost_matrix(k, l) < min_cost) {
        min_cost = cost_matrix(k, l);
        min_indices[k] = l;
      }
    }

    for (int m = 0; m < M; ++m) {
      cost_matrix (k, m + L) = costFunction(obstacles->circles[k], untracked_obstacles_[m]);

      if (cost_matrix(k, m + L) < min_cost) {
        min_cost = cost_matrix(k, m + L);
        min_indices[k] = m + L;
      }
    }
  }

  for (int i : min_indices)
    cout << i << " ";
  cout << endl;

  cout << cost_matrix << endl;

  for (int k = 0; k < min_indices.size(); ++k) {
    // Check fo conflicts - two old obstacles pretending to merge with the new one
    for (int j = k+1; j < min_indices.size(); ++j) {
      if (min_indices[k] == min_indices[j])
        cout << "Conflict" << endl;
    }

    cout << "Merging: " << k << "-th new with " << min_indices[k] << "-th old obstacle." << endl;

  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle_tracker");
  ObstacleTracker ot;
  return 0;
}
