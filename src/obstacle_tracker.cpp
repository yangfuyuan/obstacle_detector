#include "../include/obstacle_tracker.h"

using namespace  obstacle_detector;
using namespace arma;
using namespace std;

ObstacleTracker::ObstacleTracker() : nh_(""), nh_local_("~") {
  obstacles_sub_ = nh_.subscribe("obstacles", 10, &ObstacleTracker::obstaclesCallback, this);
  tracked_obstacles_pub_ = nh_.advertise<obstacle_detector::Obstacles>("tracked_obstacles", 10);
  untracked_obstacles_pub_ = nh_.advertise<obstacle_detector::Obstacles>("untracked_obstacles", 10);

  nh_local_.param("fade_counter_size", p_fade_counter_size_, 50);
  nh_local_.param("min_correspondence_cost", p_min_correspondence_cost_, 0.1);
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
      if (it->hasFaded()) {
        it = tracked_obstacles_.erase(it);
        --it;
      }
      else {
        it->updateTracking();
        tracked_obstacles_msg_.circles.push_back(it->obstacle);
      }
    }

    tracked_obstacles_pub_.publish(tracked_obstacles_msg_);

    rate.sleep();
  }
}

void ObstacleTracker::obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr& new_obstacles) {
  tracked_obstacles_msg_.header.frame_id = new_obstacles->header.frame_id;
  untracked_obstacles_msg_.header.frame_id = new_obstacles->header.frame_id;

  int N = new_obstacles->circles.size();
  int T = tracked_obstacles_.size();
  int U = untracked_obstacles_.size();

  cout << "---" << endl;
  cout << "New: " << N << endl;
  cout << "Tracked: " << T << endl;
  cout << "Untracked: " << U << endl;

  if (T + U == 0) {
    untracked_obstacles_.assign(new_obstacles->circles.begin(), new_obstacles->circles.end());
    return;
  }

  /*
   * Cost between two obstacles represents their difference.
   * The bigger the cost, the less similar they are.
   * N rows of cost_matrix represent new obstacles.
   * T+U columns of cost matrix represent old tracked and untracked obstacles.
   */
  mat cost_matrix = mat(N, T + U, fill::zeros);

  for (int n = 0; n < N; ++n) {
    for (int t = 0; t < T; ++t)
      cost_matrix(n, t) = costFunction(new_obstacles->circles[n], tracked_obstacles_[t].obstacle);

    for (int u = 0; u < U; ++u)
      cost_matrix(n, u + T) = costFunction(new_obstacles->circles[n], untracked_obstacles_[u]);
  }

  cout << "Cost matrix:" << endl;
  cout << endl << cost_matrix << endl;

  /*
   * Vector of row minimal indices keeps the indices of old obstacles (tracked and untracked)
   * that have the minimum cost related to each of new obstacles, i.e. row_min_indices[n]
   * keeps the index of old obstacle that has the minimum cost with n-th new obstacle.
   */
  vector<int> row_min_indices(N, -1); // Minimum index -1 means no correspondence has been found

  for (int n = 0; n < N; ++n) {
    double min_cost = p_min_correspondence_cost_;

    for (int t = 0; t < T; ++t) {
      if (cost_matrix(n, t) < min_cost) {
        min_cost = cost_matrix(n, t);
        row_min_indices[n] = t;
      }
    }

    for (int u = 0; u < U; ++u) {
      if (cost_matrix(n, u + T) < min_cost) {
        min_cost = cost_matrix(n, u + T);
        row_min_indices[n] = u + T;
      }
    }
  }

  cout << "Row min indices: ";
  for (int idx : row_min_indices)
    cout << idx << " ";
  cout << endl;

  /*
   * Vector of column minimal indices keeps the indices of new obstacles that has the minimum
   * cost related to each of old (tracked and untracked) obstacles, i.e. col_min_indices[i]
   * keeps the index of new obstacle that has the minimum cost with i-th old obstacle.
   */
  vector<int> col_min_indices(T + U, -1); // Minimum index -1 means no correspondence has been found

  for (int t = 0; t < T; ++t) {
    double min_cost = p_min_correspondence_cost_;

    for (int n = 0; n < N; ++n) {
      if (cost_matrix(n, t) < min_cost) {
        min_cost = cost_matrix(n, t);
        col_min_indices[t] = n;
      }
    }
  }

  for (int u = 0; u < U; ++u) {
    double min_cost = p_min_correspondence_cost_;

    for (int n = 0; n < N; ++n) {
      if (cost_matrix(n, u + T) < min_cost) {
        min_cost = cost_matrix(n, u + T);
        col_min_indices[u + T] = n;
      }
    }
  }

  cout << "Col min indices: ";
  for (int idx : col_min_indices)
    cout << idx << " ";
  cout << endl;

  /*
   * Possible situations:
   * If new obstacle does not correspond with any of old obstacles - save it as untracked.
   * If new obstacle corresponds with one and only one tracked obstacle - update it.
   * If new obstacle corresponds with one and only one untracked obstacle - save it as tracked and update it.
   *
   * If two old obstacles connect into one, we call it a fusion.
   * If one old obstacle splits into two, we call it a fission.
   * A fusion occurs if two old (tracked or not) obstacles have the same corresponding new obstacle - check columnwise.
   * A fission occurs if two new obstacles have the same corresponding old (tracked or not) obstacle - check rowswise.
   * If a fusion occured - create a tracked obstacle from the two old obstacles, update it with the new one, and remove the two old ones.
   * If a fission occured - create two tracked obstacles from the single old obstacle and update them with the new ones.
   */

  // Vector of indcises of tracked obstacles that will be removed
  vector<int> erase_indices;

  // Check for fusion
  for (int i = 0; i < T + U; ++i) {
    for (int j = i+1; j < T + U; ++j) {
      if (col_min_indices[i] == col_min_indices[j] && col_min_indices[i] >= 0) {
        cout << "Fusion" << endl;

        CircleObstacle c;

        if (i < T && j < T) {
          c = mergeCircObstacles(tracked_obstacles_[i].obstacle, tracked_obstacles_[j].obstacle);
          erase_indices.push_back(i);
          erase_indices.push_back(j);
        }
        else if (i < T && j >= T) {
          c = mergeCircObstacles(tracked_obstacles_[i].obstacle, untracked_obstacles_[j - T]);
          erase_indices.push_back(i);
        }
        else if (i >= T && j < T) {
          c = mergeCircObstacles(untracked_obstacles_[i - T], tracked_obstacles_[j].obstacle);
          erase_indices.push_back(j);
        }
        else if (i >= T && j >= T) {
          c = mergeCircObstacles(untracked_obstacles_[i - T], untracked_obstacles_[j - T]);
        }

        TrackedObstacle to = TrackedObstacle(c, p_fade_counter_size_);
        to.setCovariances(p_pose_measure_variance_, p_pose_process_variance_, p_radius_measure_variance_, p_radius_process_variance_);
        to.updateMeasurement(new_obstacles->circles[col_min_indices[i]]);

        tracked_obstacles_.push_back(to);

        // Mark both old obstacles as fused (correspondence index -2)
        col_min_indices[i] = col_min_indices[j] = -2;

        break;
      }
    }
  }

  // Check for fission
  for (int i = 0; i < N; ++i) {
    for (int j = i+1; j < N; ++j) {
      if (row_min_indices[i] == row_min_indices[j] && row_min_indices[i] >= 0) {
        cout << "Fission" << endl;

        CircleObstacle c1;
        CircleObstacle c2;

        if (row_min_indices[i] < T) {
          c1 = mergeCircObstacles(new_obstacles->circles[i], tracked_obstacles_[row_min_indices[i]].obstacle);
          c2 = mergeCircObstacles(new_obstacles->circles[j], tracked_obstacles_[row_min_indices[j]].obstacle);

          erase_indices.push_back(row_min_indices[i]);
        }
        else if (row_min_indices[i] >= T) {
          c1 = mergeCircObstacles(new_obstacles->circles[i], untracked_obstacles_[row_min_indices[i] - T]);
          c2 = mergeCircObstacles(new_obstacles->circles[j], untracked_obstacles_[row_min_indices[j] - T]);
        }

        TrackedObstacle to1 = TrackedObstacle(c1, p_fade_counter_size_);
        TrackedObstacle to2 = TrackedObstacle(c2, p_fade_counter_size_);

        to1.setCovariances(p_pose_measure_variance_, p_pose_process_variance_, p_radius_measure_variance_, p_radius_process_variance_);
        to2.setCovariances(p_pose_measure_variance_, p_pose_process_variance_, p_radius_measure_variance_, p_radius_process_variance_);

        to1.updateMeasurement(new_obstacles->circles[i]);
        to2.updateMeasurement(new_obstacles->circles[j]);

        tracked_obstacles_.push_back(to1);
        tracked_obstacles_.push_back(to2);

        // Mark both new obstacles as fissed (correspondence index -3)
        row_min_indices[i] = row_min_indices[j] = -3;

        break;
      }
    }
  }

  // Check for other possibilities
  vector<CircleObstacle> new_untracked_obstacles;
  for (int n = 0; n < N; ++n) {
    if (row_min_indices[n] == -1) {
      new_untracked_obstacles.push_back(new_obstacles->circles[n]);
    }
    else if (row_min_indices[n] < T && row_min_indices[n] >= 0) {
      tracked_obstacles_[row_min_indices[n]].updateMeasurement(new_obstacles->circles[n]);
    }
    else if (row_min_indices[n] >= T) {
      CircleObstacle c = mergeCircObstacles(new_obstacles->circles[n], untracked_obstacles_[row_min_indices[n] - T]);
      TrackedObstacle to = TrackedObstacle(c, p_fade_counter_size_);
      to.setCovariances(p_pose_measure_variance_, p_pose_process_variance_, p_radius_measure_variance_, p_radius_process_variance_);
      to.updateMeasurement(new_obstacles->circles[n]);
      tracked_obstacles_.push_back(to);
    }
  }

  // Remove tracked obstacles that are no longer existent due to fusion or fission
  for (int idx : erase_indices)
    tracked_obstacles_.erase(tracked_obstacles_.begin() + idx);

  // Remove old untracked obstacles and save new ones
  untracked_obstacles_.clear();
  untracked_obstacles_.assign(new_untracked_obstacles.begin(), new_untracked_obstacles.end());

  // Prepare and publish untracked obstacles message
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
