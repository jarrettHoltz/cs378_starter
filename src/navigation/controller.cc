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
#include "controller.h"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using f1tenth_course::AckermannCurvatureDriveMsg;
using f1tenth_course::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

Controller::Controller(const float total_distance) :
  current_speed(0.0),
  distance_travelled(0.0) {
    this->total_distance = total_distance;
  }

bool Controller::distance_left(Phase p) {
  switch (p) {
    case ACCEL: {
      float dist_left = current_speed * timestep + 0.5 * max_acceleration * std::pow(timestep, 2) +
        std::pow(std::min(max_speed, current_speed + max_acceleration * timestep), 2) / (2 * max_acceleration); 
      return dist_left <= (total_distance - distance_travelled) + kEpsilon;
    }
    case CRUISE: {
      float dist_left = max_speed * timestep + std::pow(max_speed, 2) / (2 * max_acceleration); 
      return dist_left <= (total_distance - distance_travelled) + kEpsilon;
    }
    default:
      return false; 
  }
}

// code used for Dead Reckoning, part 1
float Controller::getVelocity(){
  if (current_speed < max_speed - kEpsilon && distance_left(ACCEL)) {
    // update distance travelled
    distance_travelled += current_speed * timestep + 0.5 * max_acceleration * std::pow(timestep, 2);
    // accelerate, while capping at max_speed
    current_speed = std::min(current_speed + max_acceleration * timestep, max_speed);
  } else if (std::abs(current_speed - max_speed) <= kEpsilon && distance_left(CRUISE)) {
    // update distance travelled
    distance_travelled += max_speed * timestep;
    // cruise
  } else {
    // calculate deceleration given the distance left
    float distance_left = total_distance - distance_travelled;
    float deceleration = std::pow(current_speed, 2) / (2 * distance_left);

    distance_travelled += current_speed * timestep - 0.5 * deceleration * std::pow(timestep, 2);
    current_speed = std::max(0.0f, current_speed - deceleration * timestep); 
  }
  return current_speed;
}

// Code used for odometry given distance and speed, part 2
float Controller::getVelocity(float distance, float speed){
  current_speed = speed;
  distance_travelled = distance + current_speed * latency;
  // if travelled distance, simply return 0
  if (distance_travelled > total_distance + kEpsilon) {
    return 0.0;
  }


  // check if we can accelerate, cruise or, decelerate
  if (current_speed < max_speed - kEpsilon && distance_left(ACCEL)) {
    // accelerate
    current_speed = std::min(current_speed + max_acceleration * timestep, max_speed);
  } else if (std::abs(current_speed - max_speed) <= kEpsilon && distance_left(CRUISE)) {
    // Do nothing, since speed is at cruise speed
  } else {
    // calculate deceleration given the distance left
    float dist_left = total_distance - distance_travelled;
    float deceleration = std::pow(current_speed, 2) / (2 * dist_left);
    current_speed = std::max(0.0f, current_speed - deceleration * timestep);
  }
  // std::cout<<curvature<<std::endl;
  return current_speed;
}

// Code used for obstacle avoidance, Part 3
float Controller::getVelocity(float distance, float speed, float free_path_length){
  total_distance = free_path_length + distance;
  return getVelocity(distance, speed); 
}
