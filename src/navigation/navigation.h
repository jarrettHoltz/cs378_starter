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
#include "controller.h"

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
  float distance_to_goal;
  float theta_max;
  //Eigen::Vector2f obstruction;
  //Eigen::Vector2f closest_point;
  //EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class Navigation {
 public:

   // Constructor
  explicit Navigation(const std::string& map_file, ros::NodeHandle* n, const float carrot);

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

 private:

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

  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;

  // private point cloud variable
  std::vector<Eigen::Vector2f> point_cloud;

  // flag used to indicate whether the odometry has just started up or not
  bool startup;

  // distance to move forward for 1-D TOC
  float distance_travelled;

  // distance for receding goal
  float carrot;

  // best path to take 
  PathOption* bestOption;

  // logic to calculate best path
  void CalculateFreePathLength(PathOption* p);
  void FindBestPath();
  void CalculatePath(PathOption* p);
  void ComputeClearance(PathOption* p);
  void ComputeDistanceToGoal(PathOption* p);
  // Car specs + additional margin for actuation error
  static constexpr float car_width = 0.281;
  static constexpr float car_length = 0.535;
  static constexpr float h_safety_margin = 0.01;
  static constexpr float w_safety_margin = 0.07;


  // max clearance and weights for scoring function
  static constexpr float max_clearance = 0.15;
  static constexpr float w1 = 1.0;
  static constexpr float w2 = 0.0;
  // 1-D TOC
  Controller* toc;
};

}  // namespace navigation

#endif  // NAVIGATION_H
