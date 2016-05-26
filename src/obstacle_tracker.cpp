#include "../include/obstacle_tracker.h"

using namespace  obstacle_detector;
using namespace arma;
using namespace std;

ObstacleTracker::ObstacleTracker() : nh_(""), nh_local_("~") {
  obstacles_sub_ = nh_.subscribe("obstacles", 10, &ObstacleTracker::obstaclesCallback, this);
  tracked_obstacles_pub_ = nh_.advertise<obstacle_detector::Obstacles>("tracked_obstacles", 10);
  untracked_obstacles_pub_ = nh_.advertise<obstacle_detector::Obstacles>("untracked_obstacles", 10);

  nh_local_.param("fade_counter", p_fade_counter_, 50);
  nh_local_.param("pose_measure_variance", p_pose_measure_variance_, 1.0);
  nh_local_.param("pose_process_variance", p_pose_process_variance_, 1.0);
  nh_local_.param("radius_measure_variance", p_radius_measure_variance_, 1.0);
  nh_local_.param("radius_process_variance", p_radius_process_variance_, 1.0);

  ROS_INFO("Obstacle Tracker [OK]");
  ros::Rate rate(100);

  while (ros::ok()) {
    ros::spinOnce();

    tracked_obstacles_msg_.header.stamp = ros::Time::now();
    tracked_obstacles_msg_.circles.clear();

    for (auto it = tracked_obstacles_.begin(); it != tracked_obstacles_.end(); ++it) {
      if (it->fade_counter > 0) {
        it->updateTracking();
        tracked_obstacles_msg_.circles.push_back(it->obstacle);
      }
      else {
        it = tracked_obstacles_.erase(it);
        --it;
      }
    }

    tracked_obstacles_pub_.publish(tracked_obstacles_msg_);

    rate.sleep();
  }
}

void ObstacleTracker::obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr& obstacles) {
  tracked_obstacles_msg_.header.frame_id = obstacles->header.frame_id;
  untracked_obstacles_msg_.header.frame_id = obstacles->header.frame_id;

  int K = obstacles->circles.size();
  int L = tracked_obstacles_.size();
  int M = untracked_obstacles_.size();

  if (L + M == 0) {
    untracked_obstacles_.assign(obstacles->circles.begin(), obstacles->circles.end());
    return;
  }

  /*
   * Cost between two obstacles represents their difference.
   * The bigger the cost, the less similar they are.
   * K rows of cost_matrix represent new obstacles.
   * L+M columns of cost matrix represent old tracked and untracked obstacles.
   */
  mat cost_matrix = mat(K, L + M, fill::zeros);

  for (int k = 0; k < K; ++k) {
    for (int l = 0; l < L; ++l)
      cost_matrix(k, l) = costFunction(obstacles->circles[k], tracked_obstacles_[l].obstacle);

    for (int m = 0; m < M; ++m)
      cost_matrix(k, m + L) = costFunction(obstacles->circles[k], untracked_obstacles_[m]);
  }

  /*
   * Vector of row minimal indices keeps the indices of old obstacles (tracked and untracked)
   * that have the minimum cost related to each of new obstacles, i.e. row_min_indices[k]
   * keeps the index of old obstacle that has the minimum cost with k-th new obstacle.
   */
  vector<int> row_min_indices(K, -1); // Minimum index -1 means no correspondence has been found

  for (int k = 0; k < K; ++k) {
    double min_cost = 1.0;   // TODO: Find minimum cost allowable and set it here (if cost is bigger - k-th new obstacle goes to untracked)

    for (int l = 0; l < L; ++l) {
      if (cost_matrix(k, l) < min_cost) {
        min_cost = cost_matrix(k, l);
        row_min_indices[k] = l;
      }
    }

    for (int m = 0; m < M; ++m) {
      if (cost_matrix(k, m + L) < min_cost) {
        min_cost = cost_matrix(k, m + L);
        row_min_indices[k] = m + L;
      }
    }
  }

  /*
   * Vector of column minimal indices keeps the indices of new obstacles that has the minimum
   * cost related to each of old (tracked and untracked) obstacles, i.e. col_min_indices[i]
   * keeps the index of new obstacle that has the minimum cost with i-th old obstacle.
   */
  vector<int> col_min_indices(L + M, -1); // Minimum index -1 means no correspondence has been found

  for (int l = 0; l < L; ++l) {
    double min_cost = 1.0;

    for (int k = 0; k < K; ++k) {
      if (cost_matrix(k, l) < min_cost) {
        min_cost = cost_matrix(k, l);
        col_min_indices[l] = k;
      }
    }
  }

  for (int m = 0; m < M; ++m) {
    double min_cost = 1.0;

    for (int k = 0; k < K; ++k) {
      if (cost_matrix(k, m + L) < min_cost) {
        min_cost = cost_matrix(k, m + L);
        col_min_indices[m + L] = k;
      }
    }
  }

  /*
   * Possible situations:
   * If no correspondence between a new obstacle and any old one was found - save it as untracked.
   * If new obstacle corresponds with tracked one - update it.
   * If new obstacle corresponds with untracked one - save it as tracked and update it.
   *
   * If two old obstacles connect into one, we call it a fusion.
   * If one old obstacle splits into two, we call it a fission.
   * A fusion occurs if two old (tracked or not) obstacles have the same corresponding new obstacle - check columnwise.
   * A fission occurs if two new obstacles have the same corresponding old (tracked or not) obstacle - check rowswise.
   * If a fusion occured - create a tracked obstacle from the two old obstacles, update it with the new one, and remove the two old ones.
   * If a fission occured - create two tracked obstacles from the single old obstacle and update them with the new ones.
   */

  // Check for fusion
  for (int i = 0; i < col_min_indices.size(); ++i) {
    for (int j = i+1; j < col_min_indices.size(); ++j) {
      if (col_min_indices[i] == col_min_indices[j]) {
        cout << "Fusion" << endl;
        break;
      }
    }
  }

  // Check for fission
  for (int i = 0; i < row_min_indices.size(); ++i) {
    for (int j = i+1; j < row_min_indices.size(); ++j) {
      if (row_min_indices[i] == row_min_indices[j]) {
        cout << "Fission" << endl;
        break;
      }
    }
  }

  vector<CircleObstacle> new_untracked_obstacles;
  for (int k = 0; k < row_min_indices.size(); ++k) {
    if (row_min_indices[k] == -1) {
      new_untracked_obstacles.push_back(obstacles->circles[k]);
    }
    else if (row_min_indices[k] < L) {
      tracked_obstacles_[row_min_indices[k]].updateMeasurement(obstacles->circles[k]);
    }
    else if (row_min_indices[k] >= L) {
      TrackedObstacle to = TrackedObstacle(untracked_obstacles_[row_min_indices[k] - L], p_fade_counter_);
      to.setCovariances(p_pose_measure_variance_, p_pose_process_variance_, p_radius_measure_variance_, p_radius_process_variance_);
      to.updateMeasurement(obstacles->circles[k]);
      tracked_obstacles_.push_back(to);
    }
  }

  untracked_obstacles_.clear();
  untracked_obstacles_.assign(new_untracked_obstacles.begin(), new_untracked_obstacles.end());

  untracked_obstacles_msg_.header.stamp = ros::Time::now();
  untracked_obstacles_msg_.circles.clear();
  untracked_obstacles_msg_.circles.assign(untracked_obstacles_.begin(), untracked_obstacles_.end());
  untracked_obstacles_pub_.publish(untracked_obstacles_msg_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle_tracker");
  ObstacleTracker ot;
  return 0;
}
