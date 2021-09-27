//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <vector>

#include "eigen3/Eigen/Dense"
#include "car.h"

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace navigation {

struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  float dist_to_goal;
  float cost;
  float alpha;
  float beta;
  float radius;
  Eigen::Vector2f goal;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  Eigen::Vector2f end_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct Obstacle {
  Eigen::Vector2f loc;
  double timestamp;
};

// struct BestPath {
//   float curvature;
//   float clearance;
//   float free_path_length;
//   float dist_to_goal;
//   float cost;
//   Eigen::Vector2f obstruction;
//   Eigen::Vector2f closest_point;
//   Eigen::Vector2f end_point;
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
// };

struct Odometry {
  double x;
  double y;
  double theta;
  double vx;
  double vy;
  double omega;
};

class Navigation {
 public:

   // Constructor
  explicit Navigation(const std::string& map_file, ros::NodeHandle* n);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc,
                      float angle,
                      const Eigen::Vector2f& vel,
                      float ang_vel);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                         double time);

  // Main function called continously from main
  void Run();
  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);

  void one_d_toc_calc(float x_total);

  // Assignment 1 Additions
  float TOC(float dt, float vel_current, float arc_length, float dist_traveled);

  

  
  // Transform from local to global
  Eigen::Vector2f BaseLink2Odom(Eigen::Vector2f p);

  // Transform from global to local
  Eigen::Vector2f Odom2BaseLink(Eigen::Vector2f p);

 private:

  /* ------- Local Planning ---------*/

  // List of all obstacles
  std::list<Obstacle> ObstacleList_;
  // List of all paths
  std::vector<PathOption> Paths_;
  // free path length weight
  float free_path_length_weight_;
  // dist to goal weight
  float distance_to_goal_weight_;
  // clearance weight
  float clearance_weight_;

  // Whether odometry has been initialized.
  bool odom_initialized_;
  // Whether localization has been initialized.
  bool localization_initialized_;
  // Current robot location.
  Eigen::Vector2f robot_loc_;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_;
  // Odometry-reported robot starting location.
  Eigen::Vector2f odom_start_loc_;
  // Odometry-reported robot starting angle.
  float odom_start_angle_;
  // Latest observed point cloud.
  std::vector<Eigen::Vector2f> point_cloud_;

  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;

  std::vector<std::vector<double>> record_motion_;

  // Frank Additions
  // rotation matrix
  Eigen::Matrix2f R_odom2base_;

  // Visualization of Obstacles
  void VisObstacles();

  // Sample Paths
  void samplePaths(float num);

  void trimPath(PathOption &path, Eigen::Vector2f goal);

  void predictCollisions(PathOption& path);

  void calculateClearance(PathOption& path);

  Eigen::Vector2f P_center;

  PathOption getBestPath(Eigen::Vector2f goal_loc);

  //std::vector<float> free_path_length_vec;

  //std::vector<float> dist_to_goal_vec;

  // Definition of Car
  Car car_;

  Odometry LatencyCompensation(float observation_duration_, float actuation_duration_, float dt, float x, float y, float theta, float xdot, float ydot, float omega, float start_time);

  

};

}  // namespace navigation

#endif  // NAVIGATION_H
