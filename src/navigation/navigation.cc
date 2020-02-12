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

Navigation::Navigation(const string& map_file, ros::NodeHandle* n, const float distance_forward, const float curvature) :
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    odom_loc_(0,0),
    nav_complete_(true),
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
  toc = new Controller(distance_forward, std::max(std::min(curvature, 1.0f), -1.0f));
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

  // default free path length ?
  float free_path_length = 100;
  float curvature = toc->getCurvature();
  if (std::abs(curvature) < 0.01){
    // TODO: special case
    std::cout<<"Going straight"<<std::endl;
  } else {
    float radius = 1/curvature;
    Vector2f c(0, radius);
    float abs_r = std::abs(radius);
    float r1 = abs_r - car_width;
    float r2 = std::pow((abs_r+car_width)*(abs_r+car_width) + car_length*car_length, 0.5);
    for (std::vector<Vector2f>::const_iterator i = cloud.begin(); i != cloud.end(); ++i){
      Vector2f p = *i;
      // break;
      float x = p[0];
      float y = p[1];
      float theta = atan2(x, radius-y);
      float p_norm = (p-c).norm();
      // Obstacle detected
      if (p_norm >= (r1-kEpsilon) && p_norm <= (r2+kEpsilon) && theta > 0){
        // recalculate free path length 
        float omega = atan2(car_length, radius - car_width);
        float phi = theta - omega;
        if (radius * phi < free_path_length) {
          free_path_length = radius * phi; 
          std::cout << p << std::endl;
        }
      }
    }
  }
  
  
}

void Navigation::Run() {
  // Create Helper functions here
  // Milestone 1 will fill out part of this class.
  // Milestone 3 will complete the rest of navigation.
  // float current_distance = std::pow((robot_loc_ * robot_loc_.transpose()).norm(),0.5);
  float current_speed = robot_vel_.norm();

  AckermannCurvatureDriveMsg msg;
  msg.curvature = 0;
  // using odometry calculations for 1-D TOC
  msg.velocity = toc->getVelocity(distance_travelled, current_speed);
  msg.curvature = toc->getCurvature();
  drive_pub_.publish(msg);
}

}  // namespace navigation
