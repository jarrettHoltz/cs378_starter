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

Navigation::Navigation(const string& map_file, ros::NodeHandle* n, const float x, const float y) :
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    odom_loc_(0,0),
    nav_complete_(true),
    nav_goal_loc_(x, y),
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

    // Weihan
    float global_x = cos(angle)*nav_goal_loc_[0] - sin(angle)*nav_goal_loc_[1];
    float global_y = cos(angle)*nav_goal_loc_[1] + sin(angle)*nav_goal_loc_[0];
    nav_goal_loc_ = Vector2f(global_x, global_y); 
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

// Weihan
void Navigation::FindBestPath() {
float best_cost = -1;
  float max_free_path_length, max_clearance, distance_to_goal;
  // cap clearance to some value
  max_clearance = 0.5;
  float w1 = 1.0;
  float w2 = -3.0;
  for (float this_curvature=-1.0; this_curvature<=1.0; this_curvature+=0.1){
    CalculatePath(point_cloud, this_curvature, &max_free_path_length, &max_clearance, &distance_to_goal);
    float this_cost = max_free_path_length + w1 * max_clearance + w2 * distance_to_goal;
    if (this_curvature == -1.0){
      best_cost = this_cost - 1;}
    std::cout << "#####curvature: " << this_curvature << ", cost: " <<  this_cost << ", max_free_path_length: " << max_free_path_length << ", max_clearance: " << max_clearance << ", distance_to_goal: " << distance_to_goal << std::endl;
    if (this_cost > best_cost){
      curvature = this_curvature;
      free_path_length = max_free_path_length;
      best_cost = this_cost;
    }
  }
}

// Weihan
void Navigation::CalculatePath(const vector<Vector2f>& cloud, const float curvature, float *max_free_path_length, float *max_clearance, float *distance_to_goal){
  // default free path length
  float relative_x = nav_goal_loc_[0] - robot_loc_[0];
  float relative_y = nav_goal_loc_[1] - robot_loc_[1];
  float rotated_x = cos(-robot_angle_)*relative_x - sin(-robot_angle_)*relative_y;
  float rotated_y = cos(-robot_angle_)*relative_y + sin(-robot_angle_)*relative_x;
  // std::cout<<"robot angle: "<<robot_angle_<<", rotated (x,y): ("<<rotated_x<<","<<rotated_y<<")"<<std::endl;
  Vector2f rotated_goal(rotated_x, rotated_y);
  float this_free_path_length = 10;
  if (std::abs(curvature) < 0.01){
    // std::cout<<"Going straight"<<std::endl;
    for (std::vector<Vector2f>::const_iterator i = cloud.begin(); i != cloud.end(); ++i){
      Vector2f p = *i;
      float x = p[0];
      float y = p[1];
      if (std::abs(y) < car_width/2) {
        this_free_path_length = std::min(this_free_path_length, x - car_length);
      } else {
        *max_clearance = std::min(*max_clearance, std::abs(y) - (car_width/2));
      }
    }

    //calculate closest point of approach for straight
    if (this_free_path_length > rotated_x){
      // free path length is longer closest point of approach
      *max_free_path_length = rotated_x;
      *distance_to_goal = std::abs(rotated_y);
    } else {
      *max_free_path_length = this_free_path_length;
      Vector2f end_point(this_free_path_length, 0);
      *distance_to_goal = (rotated_goal - end_point).norm();
    }

  } else {
    float radius = 1/curvature;
    Vector2f c(0, radius);
    float abs_r = std::abs(radius);
    float r1 = abs_r - car_width;
    float r2 = std::pow((abs_r+car_width)*(abs_r+car_width) + car_length*car_length, 0.5);
    for (std::vector<Vector2f>::const_iterator i = cloud.begin(); i != cloud.end(); ++i){
      Vector2f p = *i;
      float x = p[0];
      float y = p[1];
      float theta;
      if (curvature < 0){
        theta = atan2(x, y-radius);
      } else {
        theta = atan2(x, radius-y);
      }
      float p_norm = (p-c).norm();
      // Obstacle detected
      if (p_norm >= r1 && p_norm <= r2 && theta > 0){
        // recalculate free path length 
        float omega = atan2(car_length, abs_r - car_width);
        float phi = theta - omega;
        if(abs_r * phi < free_path_length){
          this_free_path_length = std::min(this_free_path_length, radius * phi);}
      } else {
        //TODO: different than slide 7 pg 29
        //calculate clearance
        if (p_norm > abs_r) {
          *max_clearance = std::min(*max_clearance, p_norm - r2);
        } else {
          *max_clearance = std::min(*max_clearance, r1 - p_norm);
        }
      }
    }
    // std::cout<<"this_free_path_length: "<<this_free_path_length<<std::endl;
    //calculate closest point of approach for curved
    Vector2f tangent_point = (rotated_goal - c).normalized() * abs_r + c;
    // std::cout<<"normalized: "<<(rotated_goal - c).normalized()<<", scaled: "<<(rotated_goal - c).normalized()*abs_r<<std::endl;
    // std::cout<<"TANGENT: "<<tangent_point<<std::endl;
    float tangent_theta;
    if (curvature < 0){
      tangent_theta = atan2(tangent_point[0], tangent_point[1]-radius);
    } else {
      tangent_theta = atan2(tangent_point[0], radius-tangent_point[1]);}
    if (tangent_theta * abs_r <= this_free_path_length) {
      // when cloest point of approach is in the free path length
      *max_free_path_length = tangent_theta * abs_r;
      *distance_to_goal = (rotated_goal - tangent_point).norm();
    } else {
      *max_free_path_length = this_free_path_length;
      tangent_point[0] = abs_r * cos(this_free_path_length/radius);
      tangent_point[1] = abs_r * sin(this_free_path_length/radius);
      *distance_to_goal = (rotated_goal - tangent_point).norm();
    }
  }
}

// aaditya
float Navigation::CalculateFreePathLength() {
  // calculate the free path length using the point cloud data
  // default free path length = range of the sensor
  float free_path_length = 10;
  float w = (car_width / 2) + w_safety_margin;
  float h = car_length + h_safety_margin;
  if (std::abs(curvature) <= kEpsilon){
    // std::cout<<"Going straight"<<std::endl;
    for (std::vector<Vector2f>::const_iterator i = point_cloud.begin(); i != point_cloud.end(); ++i){
      Vector2f p = *i;
      float x = p[0];
      float y = p[1];
      if (std::abs(y) < w) {
        if ((x - h) < free_path_length) {
          free_path_length = x - h;
       }
      }
    }
  } else {
    float radius = 1/curvature;
    Vector2f c(0, radius);
    float abs_r = std::abs(radius);
    float r1 = abs_r - car_width;
    float r2 = std::pow((abs_r+w)*(abs_r+w) + h*h, 0.5);
    for (std::vector<Vector2f>::const_iterator i = point_cloud.begin(); i != point_cloud.end(); ++i) {
      Vector2f p = *i;
      float x = p[0];
      float y = p[1];
      float theta;
      if (curvature > 0) {
        theta = atan2(x, radius-y);
      } else {
        theta = atan2(x, y - radius);
      }
      float p_norm = (p-c).norm();
      // Obstacle detected
      if (p_norm >= (r1-kEpsilon) && p_norm <= (r2+kEpsilon) && theta > 0){
        // recalculate free path length 
        float omega = atan2(h, abs_r - w);
        float phi = theta - omega;
        if (abs_r * phi < free_path_length) {
          free_path_length = abs_r * phi;
        }
      }
    }
  }
  std::cout << free_path_length << std::endl;
  return free_path_length;
}

void Navigation::Run() {
  // Create Helper functions here
  // Milestone 1 will fill out part of this class.
  // Milestone 3 will complete the rest of navigation.
  float current_speed = robot_vel_.norm();
  FindBestPath();

  

  AckermannCurvatureDriveMsg msg;
  // using odometry calculations for 1-D TOC
  msg.velocity = toc->getVelocity(distance_travelled, current_speed, free_path_length);
  msg.curvature = curvature;
  drive_pub_.publish(msg);
}

}  // namespace navigation
