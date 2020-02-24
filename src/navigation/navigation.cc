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
  \file    navigation.cc
  \brief   Starter code for navigation.
  \author  Joydeep Biswas, (C) 2019
  */
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "f1tenth_course/AckermannCurvatureDriveMsg.h"
#include "f1tenth_course/Pose2Df.h"
#include "f1tenth_course/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using f1tenth_course::AckermannCurvatureDriveMsg;
using f1tenth_course::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

namespace {
  ros::Publisher drive_pub_;
  ros::Publisher viz_pub_;
  VisualizationMsg local_viz_msg_;
  VisualizationMsg global_viz_msg_;
  AckermannCurvatureDriveMsg drive_msg_;
  // Epsilon value for handling limited numerical precision.
  const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

  Navigation::Navigation(const string& map_file, ros::NodeHandle* n, const float carrot) :
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    odom_loc_(0,0),
    nav_complete_(false),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0),
    startup(true),
    distance_travelled(0.0) {
      drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
          "ackermann_curvature_drive", 1);
      viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
      local_viz_msg_ = visualization::NewVisualizationMessage(
          "base_link", "navigation_local");
      global_viz_msg_ = visualization::NewVisualizationMessage(
          "map", "navigation_global");
      InitRosHeader("base_link", &drive_msg_.header);
      toc = new Controller();
      this->carrot = carrot;
    }

  void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  }

  void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  }

  void Navigation::UpdateOdometry(const Vector2f& loc,
      float angle,
      const Vector2f& vel,
      float ang_vel) {
    // std::cout << "loc: " << loc << std::endl;
    // std::cout << "vel: " << vel << std::endl;
    if (startup) {
      odom_loc_ = loc;
      startup = !startup;
    }
    distance_travelled += (loc - odom_loc_).norm();
    robot_loc_ += loc - odom_loc_;
    robot_angle_ = angle;
    robot_vel_ = vel;
    robot_omega_ = ang_vel;
    odom_loc_ = loc;
  }

  void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
      double time) { 
    // save point cloud to global vector
    point_cloud = cloud;
  }

  void Navigation::FindBestPath() {
    float best_score = -std::numeric_limits<float>::infinity();
    for (float current_curvature=-1.0; current_curvature<=1.0; current_curvature+=0.1) {
      PathOption* currentOption = new PathOption();
      currentOption->curvature = current_curvature;
      CalculateFreePathLength(currentOption);
      ComputeClearance(currentOption);
      ComputeDistanceToGoal(currentOption);
      float current_path_score = currentOption->free_path_length + w1 * currentOption->clearance + w2 * currentOption->distance_to_goal;
      if (current_path_score > best_score){
        bestOption = currentOption; 
        best_score = current_path_score;
      }
    }
  }

  void Navigation::ComputeDistanceToGoal(PathOption* option) {
    // straight case
    if (std::abs(option->curvature) <= kEpsilon) {
      option->distance_to_goal = carrot - option->free_path_length; 
    } else {
      // dist
      float r = 1 / option->curvature;
      float x = r * std::sin(option->free_path_length/r);
      float y = r - r * std::cos(option->free_path_length/r);
      Vector2f fpl_point(x, y);
      Vector2f local_goal(0, carrot);
      option->distance_to_goal = (local_goal - fpl_point).norm();
    } 
  }

  void Navigation::ComputeClearance(PathOption* option) {
    // straight case
    option->clearance = max_clearance;
    float clearance = max_clearance;
    float w = (car_width / 2) + w_safety_margin;
    float h = car_length + h_safety_margin;
    for (std::vector<Vector2f>::const_iterator i = point_cloud.begin(); i != point_cloud.end(); i++) {
      Vector2f p = *i;
      float x = p[0];
      float y = p[1];
      // straight case
      if (std::abs(option->curvature) <= kEpsilon) {
        if (x > 0 && x < option->free_path_length + h && std::abs(y) < max_clearance) {
          clearance = std::abs(y) - w; 
        }
      } else {
        // arc case
        float r = 1/option->curvature;
        float abs_r = std::abs(r);
        Vector2f c(0.0, r);
        float theta = atan2(x, std::abs(r) - std::abs(y));
        if (std::abs((c-p).norm()-r) < max_clearance && theta > 0 && theta < option->theta_max) {
          // compute clearance 
          if ((c - p).norm() > r) {
            float r2 = std::pow((abs_r+w)*(abs_r+w) + h*h, 0.5); 
            clearance = (c - p).norm() - r2;
          } else {
            float r1 = abs_r - w;
            clearance = r1 - (c - p).norm();
            if (clearance < 0) {
              clearance = option->clearance;
            }
          }
        }
      } 

      // set option clearance if less than current
      if (clearance < option->clearance) {
        option->clearance = clearance;
      }
    }
  }

void Navigation::CalculateFreePathLength(PathOption* path) {
  // calculate the free path length using the point cloud data
  // default free path length = range of the sensor
  path->free_path_length = 10;
  float w = (car_width / 2) + w_safety_margin;
  float h = car_length + h_safety_margin;
  if (std::abs(path->curvature) <= kEpsilon){
    // std::cout<<"Going straight"<<std::endl;
    for (std::vector<Vector2f>::const_iterator i = point_cloud.begin(); i != point_cloud.end(); ++i){
      Vector2f p = *i;
      float x = p[0];
      float y = p[1];
      if (std::abs(y) < w) {
        if ((x - h) < path->free_path_length) {
          path->free_path_length = x - h;
        }
      }
    }
  } else {
    float radius = 1/path->curvature;
    Vector2f c(0, radius);
    float abs_radius = std::abs(radius);
    float r1 = abs_radius - w;
    float r2 = std::pow((abs_radius+w)*(abs_radius+w) + h*h, 0.5);
    for (std::vector<Vector2f>::const_iterator i = point_cloud.begin(); i != point_cloud.end(); ++i) {
      Vector2f p = *i;
      float x = p[0];
      float y = std::abs(p[1]);
      float theta = atan2(x, abs_radius-y);
      float p_norm = (p-c).norm();
      // Obstacle detected
      if (p_norm >= r1 && p_norm <= r2 && theta > 0){
        // recalculate free path length 
        float omega = atan2(h, abs_radius - w);
        float phi = theta - omega;
        if (abs_radius * phi < path->free_path_length) {
          path->free_path_length = abs_radius * phi;
          path->theta_max = theta;
        }
      }
    }
  }
}

void Navigation::Run() {
  // Create Helper functions here
  // Milestone 1 will fill out part of this class.
  // Milestone 3 will complete the rest of navigation.
  while (!nav_complete_) {
    float current_speed = robot_vel_.norm();
    FindBestPath();


    AckermannCurvatureDriveMsg msg;
    // using odometry calculations for 1-D TOC
    msg.velocity = toc->getVelocity(distance_travelled, current_speed, bestOption->free_path_length);
    msg.curvature = bestOption->curvature;
    drive_pub_.publish(msg);
  }
}

}  // namespace navigation
