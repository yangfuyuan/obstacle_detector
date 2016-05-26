#include "../include/obstacle_tracker.h"

using namespace  obstacle_detector;
using namespace arma;
using namespace std;

ObstacleTracker::ObstacleTracker() : nh_(""), nh_local_("~") {
  obstacles_sub_ = nh_.subscribe("obstacles", 10, &ObstacleTracker::obstaclesCallback, this);
  obstacles_pub_ = nh_.advertise<obstacle_detector::Obstacles>("tracked_obstacles", 10);

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

    obstacles_pub_.publish(tracked_obstacles_msg_);

    rate.sleep();
  }
}

void ObstacleTracker::obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr& obstacles) {
  tracked_obstacles_msg_.header.frame_id = obstacles->header.frame_id;

  int K = obstacles->circles.size();
  int L = tracked_obstacles_.size();
  int M = untracked_obstacles_.size();

  if (L + M == 0) {
    untracked_obstacles_.assign(obstacles->circles.begin(), obstacles->circles.end());
    return;
  }

  /*
   * Cost between two obstacles represents their difference.
   * The bigger cost, the less similar they are.
   * K rows of cost_matrix represent new obstacles.
   * L+M columns of cost matrix represent old tracked and untracked obstacles.
   * Vector of minimal indices keeps the index of obstacle with minimum
   * cost related to k-th new obstacle.
   */
  mat cost_matrix = mat(K, L + M, fill::zeros);
  vector<int> min_indices(K, -1); // Minimum index -1 means no correspondence has been found

  for (int k = 0; k < K; ++k) {
    double min_cost = 1.0;   // TODO: Find minimum cost allowable and set it here (if cost is bigger - k-th new obstacle goes to untracked)

    for (int l = 0; l < L; ++l) {
      cost_matrix(k, l) = costFunction(obstacles->circles[k], tracked_obstacles_[l].obstacle);

      if (cost_matrix(k, l) < min_cost) {
        min_cost = cost_matrix(k, l);
        min_indices[k] = l;
      }
    }

    for (int m = 0; m < M; ++m) {
      cost_matrix(k, m + L) = costFunction(obstacles->circles[k], untracked_obstacles_[m]);

      if (cost_matrix(k, m + L) < min_cost) {
        min_cost = cost_matrix(k, m + L);
        min_indices[k] = m + L;
      }
    }
  }

  vector<CircleObstacle> new_untracked_obstacles;

  for (int k = 0; k < min_indices.size(); ++k) {
    /*
     * If two obstacles connect into one, we call it a fusion.
     * If one obstacle divides into two, we call it a fission.
     * A fusion occurs if one new obstacle has two old (tracked or not) correspondents.
     * A fission occurs if two new obstacles have one old (tracked or not) correspondent.
     */

    //    for (int j = k+1; j < min_indices.size(); ++j) {
    //      if (min_indices[k] == min_indices[j])
    //        cout << "Fission" << endl;
    //    }

    // TODO: search for minimal indices over old obstacles to find possible fusions.

    if (min_indices[k] == -1) {
      new_untracked_obstacles.push_back(obstacles->circles[k]);
    }
    else if (min_indices[k] < L) {
      tracked_obstacles_[min_indices[k]].updateMeasurement(obstacles->circles[k]);
    }
    else if (min_indices[k] >= L) {
      TrackedObstacle to = TrackedObstacle(untracked_obstacles_[min_indices[k] - L], p_fade_counter_);
      to.setCovariances(p_pose_measure_variance_, p_pose_process_variance_, p_radius_measure_variance_, p_radius_process_variance_);
      to.updateMeasurement(obstacles->circles[k]);
      tracked_obstacles_.push_back(to);
    }
  }

  untracked_obstacles_.clear();
  untracked_obstacles_.assign(new_untracked_obstacles.begin(), new_untracked_obstacles.end());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle_tracker");
  ObstacleTracker ot;
  return 0;
}
